from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
import asyncio
import json
import os
import time
from ssh_manager import ssh_manager
from ros_bridge import init_ros, ros_bridge
from glob import glob

# FastAPI 앱 초기화
# 이 파일(app.py)은 웹 관제 시스템의 백엔드 서버 메인 파일로 프론트엔드(React)와 로봇(ROS, SSH) 사이의 중계 역할을 담당한다
# REST API: 로봇 연결, 전략 배포, 파일 저장/로드
# WebSocket: 실시간 로봇 상태(위치, 배터리 등) 스트리밍
app = FastAPI()

# CORS 설정 (Cross-Origin Resource Sharing) 보안 정책 상, 다른 포트(3000번 React)에서 이 서버(8000번)로 요청을 보낼 때 차단되지 않도록 허용한다
# 웹 브라우저의 보안 정책으로 인해 다른 포트(React 3000번 vs API 8000번) 간의 통신은 기본적으로 차단되는데 이를 허용해주는 설정
app.add_middleware(
    CORSMiddleware, # 프론트엔드(React, 보통 3000포트)와 백엔드(FastAPI, 8000포트)의 포트가 달라서 발생하는 CORS(교차 출처) 에러를 막아주는 역할
    allow_origins=["*"],  # 모든 출처(Origin)에서의 접근을 허용 (개발 편의성)
    allow_credentials=True, # 쿠키 등 인증 정보 포함 허용
    allow_methods=["*"],  # 모든 HTTP 메서드(GET, POST, PUT, DELETE 등) 허용
    allow_headers=["*"],  # 모든 HTTP 헤더 허용
)

# 캐쉬 컨트롤 -> 브라우저가 예전 데이터 캐싱(기억)하지 못하게 막는 역할
@app.middleware("http")
async def add_no_cache_header(request, call_next): 
    response = await call_next(request)
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

# 프론트엔드 빌드 결과물 경로 설정 -> React를 빌드한 결과물(dist 폴더)을 서버가 인식하게 한다
FRONTEND_DIST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../frontend/dist")

# 정적 파일 서빙 (CSS, JS, 이미지 등) HTML이 아닌 부가 리소스들을 브라우저가 가져갈 수 있도록 경로를 마운트한다
if os.path.exists(FRONTEND_DIST_DIR):
    app.mount("/assets", StaticFiles(directory=os.path.join(FRONTEND_DIST_DIR, "assets")), name="assets")

# 메인 페이지 라우팅 ("/") -> 브라우저로 접속했을 때 React 앱의 진입점(index.html)을 보여준다
@app.get("/")
async def serve_index():
    if os.path.exists(os.path.join(FRONTEND_DIST_DIR, "index.html")):
        return FileResponse(os.path.join(FRONTEND_DIST_DIR, "index.html"))
    return {"message": "Frontend build not found. Please run 'npm run build' in frontend directory."}

# ROS 2 노드 초기화 -> 서버 시작 시 ROS 2 노드를 생성하여 토픽 통신을 준비한다
# Mac 등 ROS가 없는 환경에서는 예외가 발생하며, 이 경우 자동으로 시뮬레이션 모드로 동작하거나 UDP 모니터를 사용한다 // 삭제 예정
try:
    ros_bridge_node = init_ros() # 서버가 켜질 때 ROS 2 노드 준비
except Exception as e:
    print(f"Warning: ROS 2 init failed (Sim Mode?): {e}")

# 로봇 연결 설정 -> 프론트엔드에서 보낸 JSON 데이터를 파싱하기 위한 Pydantic 모델
class RobotConfig(BaseModel):
    id: str       # 로봇 식별자 
    ip: str       # 로봇의 IP 주소 
    username: str # SSH 접속 계정명
    password: str # SSH 접속 비밀번호

# 쉘 명령어 실행 요청
class Command(BaseModel):
    robot_id: str # 명령을 수행할 로봇 ID
    cmd: str      # 실행할 리눅스 쉘 명령어
    force_local: bool = False # 시뮬레이션 버튼 등으로 무조건 로컬 실행을 강제할지 여부

# 프론트엔드에서 입력한 IP와 비번으로 로봇에 SSH 원격 접속을 시도함 -> 사용자가 'Connect' 버튼을 눌렀을 때 호출된다
@app.post("/api/connect")
def connect_robot(config: RobotConfig):
    print(f"[API] Connect request: {config.id}@{config.ip}")
    success = ssh_manager.connect(config.id, config.ip, config.username, config.password)
    if not success:
        raise HTTPException(status_code=400, detail="Connection Failed")
    return {"status": "connected", "id": config.id}

# 쉘 명령어 전송 -> 'Start Program', 'Reboot' 등 버튼을 눌렀을 때 호출된다
@app.post("/api/command") # 로봇에게 직접 리눅스 명령어를 내림
def send_command(command: Command):
    print(f"[API] Command request: {command.cmd} -> {command.robot_id}")
    
    # 로그 출력 파일 생성
    if "brain_nohup.log" in command.cmd:
        # Absolute path hardcoding breaks simulation computers. Write to relative launcher.log
        command.cmd = command.cmd.replace("brain_nohup.log", "launcher.log")
        
        # Fallback chain for changing directories: Real Robot -> User Simulation PC -> Mac Dev Env
        dynamic_cd = 'cd /home/booster/Workspace/GUI/INHA-Player 2>/dev/null || cd ~/Workspace/GUI-sim/simul-bridge-gui/INHA-Player 2>/dev/null || cd ~/Workspace/INHA/simul-bridge-gui/INHA-Player 2>/dev/null'
        command.cmd = command.cmd.replace("cd /home/booster/Workspace/Soccer", dynamic_cd)
            
        print("[API] Redirected output to launcher.log, switched workspace dynamically")
    stdout, stderr = ssh_manager.execute_command(command.robot_id, command.cmd, force_local=command.force_local)
    if stdout is None:
        raise HTTPException(status_code=500, detail=stderr)
    return {"stdout": stdout, "stderr": stderr}

# 전략 배포 요청
class StrategyDeploy(BaseModel):
    robot_id: str = "all" # 특정 로봇 ID 또는 전체 로봇
    strategy_xml: str     # 배포할 Behavior Tree XML 내용

# 전략 배포 (Hot-Swap) -> 토글의 전략(XML)을 로봇에게 전송하여 즉시 적용시킨다 -> 로봇 내부에서 'ros2 topic pub' 명령을 실행하는 방식으로 동작한다
@app.post("/api/deploy_strategy")
def deploy_strategy_endpoint(data: StrategyDeploy):
    # 1. 파일 내용(XML) 추출
    xml_content = data.strategy_xml
    print(f"[API] Deploy request to {data.robot_id}")
    
    # 2. 현재 SSH로 연결된 로봇 목록 확인
    connected_robots = list(ssh_manager.clients.keys())
    
    # 연결된 로봇이 없으면 경고 출력
    if not connected_robots:
         print("[WARN] No robot connected via SSH. Assuming Simulation mode.")
         # 시뮬레이션: game.xml 직접 덮어쓰기 (src 원본 및 install 실행경로 모두)
         src_xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../INHA-Player/src/brain/behavior_trees/game.xml")
         install_xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../INHA-Player/install/brain/share/brain/behavior_trees/game.xml")
         
         try:
             # src 반영 (소스코드 보존용)
             with open(src_xml_path, "w") as f:
                 f.write(xml_content)
                 
             # install 반영 (실제 시뮬레이션 노드가 즉시 읽어들일 수 있도록 함)
             if os.path.exists(os.path.dirname(install_xml_path)):
                 with open(install_xml_path, "w") as f:
                     f.write(xml_content)
             
             print(f"[SIM] Deployed directly to {src_xml_path} and {install_xml_path}")
             return {"status": "success", "message": "Strategy deployed to simulation (game.xml). Please restart sim_start.sh to apply."}
         except Exception as e:
             print(f"[SIM] Failed to write game.xml: {e}")
             return {"status": "error", "message": f"Strategy deployment failed in simulation: {e}"}

    # 3. 대상 로봇들에게 전략 배포
    success_count = 0
    targets = connected_robots if data.robot_id == "all" else [data.robot_id]
    
    for robot_id in targets:
        if robot_id not in ssh_manager.clients: continue
        
        # ssh_manager를 통해 원격 명령 실행
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



# 전략 파일들이 저장될 디렉토리
STRATEGY_DIR = "strategies"

# 저장된 전략 목록 조회 -> strategies 폴더 내의 모든 .xml 파일 이름을 반환한다
@app.get("/api/strategies")
async def list_strategies():
    files = glob(os.path.join(STRATEGY_DIR, "*.xml"))
    return {"strategies": [os.path.basename(f) for f in files]}

# 전략 저장 요청
class StrategySave(BaseModel):
    name: str # 파일명
    xml: str  # 파일 내용

# 전략 저장
# Blockly로 작성한 전략을 서버에 파일로 저장한다 // 삭제 예정
@app.post("/api/strategies")
async def save_strategy(data: StrategySave):
    # 연결된 로봇들에게 xml_content를 전송
    filename = data.name if data.name.endswith(".xml") else f"{data.name}.xml"
    path = os.path.join(STRATEGY_DIR, filename)
    with open(path, "w") as f:
        f.write(data.xml)
    return {"status": "saved", "name": filename}

# 특정 전략 불러오기 -> 저장된 XML 파일의 내용을 읽어서 반환한다
@app.get("/api/strategies/{name}")
async def load_strategy(name: str):
    path = os.path.join(STRATEGY_DIR, name)
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail="Strategy not found")
    with open(path, "r") as f:
        return {"xml": f.read()}

from udp_monitor import udp_monitor

from gc_monitor import gc_monitor

# 비상 정지 (Emergency Stop) -> 모든 연결된 로봇에게 SetVelocity<0,0,0> 전략을 배포하는 방식으로 정지시킴
@app.post("/api/emergency_stop")
def emergency_stop():
    print("[API] Emergency Stop Requested!")
    
    # # 모든 로봇에게 stop_xml 배포
    stop_xml = """
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <SetVelocity vx="0.0" vy="0.0" w="0.0"/>
        </BehaviorTree>
    </root>
    """
    connected_robots = list(ssh_manager.clients.keys())
    if not connected_robots:
        raise HTTPException(status_code=400, detail="No robots connected")
        
    results = {}
    for robot_id in connected_robots:
        success, msg = ssh_manager.deploy_strategy(robot_id, stop_xml)
        results[robot_id] = "Stopped" if success else f"Failed: {msg}"
        
    return {"status": "executed", "results": results}

# 로그 조회
@app.get("/api/logs/{robot_id}")
def get_logs(robot_id: str, force_local: bool = False):
    log_content = ssh_manager.fetch_log(robot_id, lines=100, force_local=force_local)
    return {"id": robot_id, "log": log_content}

# 실시간 상태 스트리밍 엔드포인트 -> 프론트엔드가 이 주소로 웹소켓을 연결하면, 0.5초마다 로봇들의 최신 상태를 JSON으로 전송한다
@app.websocket("/ws/status")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept() # 연결 수락
    last_print = time.time()
    try:
        while True:
            # udp_monitor와 gc_monitor에서 상태를 가져와 JSON으로 묶음
            status = {}
            
            # 1. UDP Monitor 데이터 우선 사용 (실제 로봇 데이터)
            udp_data = udp_monitor.get_status()
            if udp_data:
                status["robots"] = udp_data
            elif ros_bridge_node:
                status["robots"] = ros_bridge_node.get_status()
            else:
                status["robots"] = {}

            # 2. GameController 데이터 추가
            gc_data = gc_monitor.get_status()
            status["game_info"] = gc_data
            
            if time.time() - last_print > 2.0:
                print(f"[WS] Sending {len(status['robots'])} robots, GC state: {gc_data.get('state')}")
                last_print = time.time()

            # 클라이언트에게 JSON 전송
            await websocket.send_json(status)
            await asyncio.sleep(0.5) # 업데이트 주기 (0.5초)
    except Exception as e:
        print(f"WebSocket closed: {e}")

# 서버 직접 실행 시 진입점
if __name__ == "__main__":
    import uvicorn
    # uvicorn 서버 실행 (호스트 0.0.0.0은 외부 접속 허용을 의미)
    uvicorn.run(app, host="0.0.0.0", port=8000)

# 정리 ) 데이터 모델 : 프론트엔드와의 데이터 통신을 위한 모델, API : 버튼을 누르는 등의 일회성 동작, 웹소켓 : 실시간 데이터 통신