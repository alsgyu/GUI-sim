from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
import asyncio
import json
import os
from ssh_manager import ssh_manager
from ros_bridge import init_ros, ros_bridge
from glob import glob

# =========================================================
# 시뮬레이션 모드 플래그
# True  → Isaac Sim 환경: SSH 불필요, ROS2 토픽으로 직접 전략 배포
# False → 실제 로봇 환경: SSH + UDP 기반
# =========================================================
SIM_MODE = True

# FastAPI 앱 초기화
app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 캐쉬 컨트롤
@app.middleware("http")
async def add_no_cache_header(request, call_next): 
    response = await call_next(request)
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

# 프론트엔드 빌드 결과물 경로
FRONTEND_DIST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../frontend/dist")

if os.path.exists(FRONTEND_DIST_DIR):
    app.mount("/assets", StaticFiles(directory=os.path.join(FRONTEND_DIST_DIR, "assets")), name="assets")

@app.get("/")
async def serve_index():
    if os.path.exists(os.path.join(FRONTEND_DIST_DIR, "index.html")):
        return FileResponse(os.path.join(FRONTEND_DIST_DIR, "index.html"))
    return {"message": "Frontend build not found. Please run 'npm run build' in frontend directory."}

# ROS 2 노드 초기화
ros_bridge_node = None
try:
    ros_bridge_node = init_ros()
    print("[ROS] ROS2 bridge initialized successfully")
except Exception as e:
    print(f"[WARN] ROS 2 init failed: {e}")
    if SIM_MODE:
        print("[WARN] SIM_MODE is True but ROS2 failed - strategy deploy will not work!")

# ───────────────────────────────────────────────
# 데이터 모델
# ───────────────────────────────────────────────

class RobotConfig(BaseModel):
    id: str
    ip: str
    username: str
    password: str

class Command(BaseModel):
    robot_id: str
    cmd: str

class StrategyDeploy(BaseModel):
    robot_id: str = "all"
    strategy_xml: str

class StrategySave(BaseModel):
    name: str
    xml: str

# ───────────────────────────────────────────────
# SSH 연결 (실제 로봇 전용)
# ───────────────────────────────────────────────

@app.post("/api/connect")
def connect_robot(config: RobotConfig):
    if SIM_MODE:
        # 시뮬 모드에서는 SSH 불필요 → 항상 성공으로 응답
        print(f"[SIM] Connect request ignored (SIM_MODE): {config.id}")
        return {"status": "connected", "id": config.id}
    
    print(f"[API] Connect request: {config.id}@{config.ip}")
    success = ssh_manager.connect(config.id, config.ip, config.username, config.password)
    if not success:
        raise HTTPException(status_code=400, detail="Connection Failed")
    return {"status": "connected", "id": config.id}

# ───────────────────────────────────────────────
# Shell 명령 (실제 로봇 전용)
# ───────────────────────────────────────────────

@app.post("/api/command")
def send_command(command: Command):
    if SIM_MODE:
        # 시뮬 모드에서는 SSH 명령 불필요
        print(f"[SIM] Command request ignored (SIM_MODE): {command.cmd}")
        return {"stdout": "[SIM_MODE] Command skipped", "stderr": ""}
    
    print(f"[API] Command request: {command.cmd} -> {command.robot_id}")
    if "brain_nohup.log" in command.cmd:
        command.cmd = command.cmd.replace("brain_nohup.log", "/home/booster/Workspace/GUI/INHA-Player/launcher.log")
        command.cmd = command.cmd.replace("Workspace/Soccer", "Workspace/GUI/INHA-Player")
    stdout, stderr = ssh_manager.execute_command(command.robot_id, command.cmd)
    if stdout is None:
        raise HTTPException(status_code=500, detail=stderr)
    return {"stdout": stdout, "stderr": stderr}

# ───────────────────────────────────────────────
# 전략 배포 (핵심 API)
# 시뮬: ROS2 토픽으로 직접 배포 (/robot0/strategy/deploy 등)
# 실제: SSH 경유 배포
# ───────────────────────────────────────────────

@app.post("/api/deploy_strategy")
def deploy_strategy_endpoint(data: StrategyDeploy):
    xml_content = data.strategy_xml
    print(f"[API] Deploy request to '{data.robot_id}'")

    # ── 시뮬 모드: ROS2 직접 배포 ──
    if SIM_MODE:
        if ros_bridge_node is None:
            raise HTTPException(status_code=503, detail="ROS2 bridge not available (check if ROS2 is sourced)")
        
        success = ros_bridge_node.publish_strategy(data.robot_id, xml_content)
        if success:
            return {"status": "success", "message": f"[SIM] Deployed via ROS2 to '{data.robot_id}'"}
        else:
            raise HTTPException(status_code=500, detail="ROS2 publish failed")

    # ── 실제 로봇 모드: SSH 배포 ──
    connected_robots = list(ssh_manager.clients.keys())
    
    if not connected_robots:
        # SSH 연결이 없지만 ROS2가 있는 경우 폴백
        if ros_bridge_node:
            ros_bridge_node.publish_strategy(data.robot_id, xml_content)
        return {"status": "skipped", "message": "No robot connected via SSH"}

    success_count = 0
    targets = connected_robots if data.robot_id == "all" else [data.robot_id]
    
    for robot_id in targets:
        if robot_id not in ssh_manager.clients: continue
        success, msg = ssh_manager.deploy_strategy(robot_id, xml_content)
        if success:
            success_count += 1
            print(f"[Deploy] Success to {robot_id}")
        else:
            print(f"[Deploy] Failed to {robot_id}: {msg}")

    if success_count > 0:
        return {"status": "success", "message": f"Deployed to {success_count} robots"}
    else:
        raise HTTPException(status_code=500, detail="Failed to deploy to any robot")

# ───────────────────────────────────────────────
# 전략 파일 관리
# ───────────────────────────────────────────────

STRATEGY_DIR = "strategies"

@app.get("/api/strategies")
async def list_strategies():
    files = glob(os.path.join(STRATEGY_DIR, "*.xml"))
    return {"strategies": [os.path.basename(f) for f in files]}

@app.post("/api/strategies")
async def save_strategy(data: StrategySave):
    filename = data.name if data.name.endswith(".xml") else f"{data.name}.xml"
    path = os.path.join(STRATEGY_DIR, filename)
    with open(path, "w") as f:
        f.write(data.xml)
    return {"status": "saved", "name": filename}

@app.get("/api/strategies/{name}")
async def load_strategy(name: str):
    path = os.path.join(STRATEGY_DIR, name)
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail="Strategy not found")
    with open(path, "r") as f:
        return {"xml": f.read()}

# ───────────────────────────────────────────────
# 비상 정지
# ───────────────────────────────────────────────

@app.post("/api/emergency_stop")
def emergency_stop():
    print("[API] Emergency Stop Requested!")
    stop_xml = """
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <SetVelocity vx="0.0" vy="0.0" w="0.0"/>
        </BehaviorTree>
    </root>
    """
    
    if SIM_MODE:
        if ros_bridge_node is None:
            raise HTTPException(status_code=503, detail="ROS2 bridge not available")
        ros_bridge_node.publish_strategy("all", stop_xml)
        return {"status": "executed", "message": "[SIM] Emergency stop sent via ROS2"}
    
    connected_robots = list(ssh_manager.clients.keys())
    if not connected_robots:
        raise HTTPException(status_code=400, detail="No robots connected")
        
    results = {}
    for robot_id in connected_robots:
        success, msg = ssh_manager.deploy_strategy(robot_id, stop_xml)
        results[robot_id] = "Stopped" if success else f"Failed: {msg}"
        
    return {"status": "executed", "results": results}

# ───────────────────────────────────────────────
# 로그 조회
# ───────────────────────────────────────────────

@app.get("/api/logs/{robot_id}")
def get_logs(robot_id: str):
    if SIM_MODE:
        return {"id": robot_id, "log": "[SIM_MODE] Log not available via SSH in simulation.\nCheck ROS2 terminal output."}
    log_content = ssh_manager.fetch_log(robot_id, lines=100)
    return {"id": robot_id, "log": log_content}

# ───────────────────────────────────────────────
# 실시간 상태 스트리밍 WebSocket
# ───────────────────────────────────────────────

from udp_monitor import udp_monitor
from gc_monitor import gc_monitor

@app.websocket("/ws/status")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            status = {}
            
            # 로봇 상태: 시뮬에서는 UDP 데이터가 없으므로 ROS 브릿지 우선
            if SIM_MODE:
                # 시뮬에서는 로봇 상태 데이터 없음 → 빈 dict (카드는 표시되지만 데이터 없음)
                # 추후 ROS2 토픽 구독으로 확장 가능
                status["robots"] = ros_bridge_node.get_status() if ros_bridge_node else {}
            else:
                udp_data = udp_monitor.get_status()
                if udp_data:
                    status["robots"] = udp_data
                elif ros_bridge_node:
                    status["robots"] = ros_bridge_node.get_status()
                else:
                    status["robots"] = {}

            # GameController 상태
            # SIM_MODE: game_controller ROS 노드가 port 3838을 점유하므로 ROS 토픽으로 수신
            if SIM_MODE and ros_bridge_node and ros_bridge_node.is_gc_available():
                status["game_info"] = ros_bridge_node.get_gc_status()
            else:
                status["game_info"] = gc_monitor.get_status()
            
            await websocket.send_json(status)
            await asyncio.sleep(0.5)
    except Exception as e:
        print(f"WebSocket closed: {e}")

# 서버 직접 실행 시 진입점
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)