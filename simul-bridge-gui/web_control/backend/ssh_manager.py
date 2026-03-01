import paramiko # ssh 연결을 위한 파이썬 라이브러리
import asyncio
import logging
import base64

# 로봇들과의 SSH 연결 세션을 관리하고, 원격 명령을 실행하는 역할 -> 싱글톤 패턴처럼 하나의 인스턴스(ssh_manager)를 전역에서 공유하여 사용
class SSHManager:
    def __init__(self):
        # 연결된 SSH 클라이언트 객체들을 저장하는 딕셔너리 -> Key: 로봇 ID (예: "robot_1"), Value: paramiko.SSHClient 객체
        self.clients = {} 
        self.logger = logging.getLogger("SSHManager")

    # [SSH 연결] -> 사용자가 입력한 IP, ID, PW로 로봇에 접속을 시도 - 디폴트값 지정해둘 수 있음
    
    def connect(self, robot_id, ip, username, password):
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
            
            # IP 문자열에 ':'이 있다면 IP와 포트를 분리
            port = 22
            if ":" in ip:
                ip, port_str = ip.split(":")
                port = int(port_str)
                
            # 실제 연결 시도 (포트 파라미터 추가)
            client.connect(ip, port=port, username=username, password=password, timeout=3.0)

    # def connect(self, robot_id, ip, username, password):
    #     try:
    #         client = paramiko.SSHClient()
    #         client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
            
    #         # 실제 연결 시도 (타임아웃 3초) - 잘못된 연결임을 알기에는 충분
    #         client.connect(ip, username=username, password=password, timeout=3.0)
            
            # [검증] 실제 명령이 먹히는지 확인 - 단순 연결 후 whoami 명령어 실행으로 다시 검증
            stdin, stdout, stderr = client.exec_command("whoami")
            user = stdout.read().decode().strip()
            if not user:
                raise Exception("SSH Connection verification failed (whoami no output)")
                
            self.logger.info(f"Connected to {robot_id} ({ip}) as {user}")
            
            # 연결 성공 시 클라이언트 목록에 저장
            self.clients[robot_id] = client
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to {robot_id}: {e}")
            return False

    # 원격 명령 실행 -> 특정 로봇에게 기본적인 리눅스 쉘 명령어를 전송하고 결과를 받아오기
    def execute_command(self, robot_id, command):
        if robot_id not in self.clients:
            return None, "Not connected"
        
        try:
            client = self.clients[robot_id]
            # exec_command: 비차단(Non-blocking) 방식으로 명령 실행 -> stdin, stdout, stderr 스트림을 반환받음
            print(f"[SSH Cmd] Executing on {robot_id}: {command}")
            stdin, stdout, stderr = client.exec_command(command)
            
            # 실행 결과를 문자열로 디코딩하여 반환
            out_str = stdout.read().decode()
            err_str = stderr.read().decode()
            print(f"[SSH Result] OUT: {out_str[:100]}... / ERR: {err_str[:100]}...")
            return out_str, err_str 
        except Exception as e:
            print(f"[SSH Error] {e}")
            return None, str(e)

    # [연결 해제]
    def disconnect(self, robot_id):
        if robot_id in self.clients:
            self.clients[robot_id].close()
            del self.clients[robot_id]


    # -----------------------------------------------------------------------------------------
    # [전략 배포 기능] (Hot-Swap)
    # 작성된 Behavior Tree XML을 로봇에게 전송하고 즉시 실행시키는 기능(재부팅 없이 실시간 교체를 위해)
    # 1. XML 내용을 Base64로 인코딩 (XML 내 특수문자로 인해 쉘 통신에서 깨지는 것을 방지하기 위함)
    # 2. 로봇의 임시 경로(/tmp/strategy_deploy.xml)에 디코딩된 XML 파일 생성
    # 3. 파이썬 스크립트(deploy.py)를 동적으로 생성하여 로봇에서 실행
    # 4. deploy.py가 /tmp/strategy_deploy.xml 파일을 읽어 ROS 2 토픽(/{robot_id}/strategy/deploy)으로 발행(Publish)
    # 5. 로봇 내부의 Brain 노드가 이 토픽을 구독하여 새로운 행동 트리(XML)를 즉시 적용
    # -----------------------------------------------------------------------------------------
    def deploy_strategy(self, robot_id, xml_content):
        if robot_id not in self.clients:
            return False, "Not connected"

        try:
            # XML 데이터를 해석하기 위한 라이브러리인 ElementTree
            import xml.etree.ElementTree as ET

            # 문자열 형태의 xml_content를 XML 객체 트리 구조로 변환
            root = ET.fromstring(xml_content)

            # XML 내에서 <BehaviorTree> 태그를 가진 노드를 찾는다
            bt_node = root.find('BehaviorTree')
            strategy_id = bt_node.get('ID') if bt_node is not None else None
            print(f"[Deploy] Runtime-only deployment (ID: {strategy_id})")

            # ROS2 환경 소스 설정 명령어
            setup_cmd = "source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null"
            setup_cmd += "; source /home/booster/Workspace/GUI/INHA-Player/install/setup.bash 2>/dev/null"
            setup_cmd += "; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/booster/Workspace/GUI/INHA-Player/configs/fastdds.xml"
            setup_cmd += "; export ROS_LOG_DIR=/tmp"

            # STEP 1: 어떤 topic으로 brain이 수신 대기중인지 자동 탐지
            # ros2 topic list 에서 /strategy/deploy 로 끝나는 토픽을 찾는다
            # 예: /robot_1/strategy/deploy 또는 /robot1/strategy/deploy
            client = self.clients[robot_id]
            probe_cmd = f"bash -c '{setup_cmd}; ros2 topic list 2>/dev/null | grep strategy/deploy'"
            stdin, stdout, stderr = client.exec_command(probe_cmd, timeout=8)
            topic_list_raw = stdout.read().decode().strip()
            print(f"[Deploy] Detected strategy topics: {topic_list_raw}")

            # 탐지된 토픽 목록에서 알맞은 것을 고른다
            # 우선순위: robot_id 정확히 일치 > 유일한 토픽 하나 > fallback
            detected_topic = None
            if topic_list_raw:
                topics = [t.strip() for t in topic_list_raw.splitlines() if t.strip()]
                # robot_id 포함 여부 우선 탐색 (예: robot1, robot_1)
                for t in topics:
                    if robot_id.replace('_', '') in t.replace('_', ''):
                        detected_topic = t
                        break
                # 못 찾으면 첫번째꺼
                if not detected_topic and topics:
                    detected_topic = topics[0]

            # fallback: 원래 방식대로 robot_id 그대로 사용
            if not detected_topic:
                detected_topic = f"/{robot_id}/strategy/deploy"
                print(f"[Deploy] Topic not auto-detected. Using fallback: {detected_topic}")
            else:
                print(f"[Deploy] Using detected topic: {detected_topic}")

            # STEP 2: XML을 Base64로 인코딩해서 /tmp에 파일로 저장
            xml_b64 = base64.b64encode(xml_content.encode('utf-8')).decode('utf-8')
            write_xml_cmd = f"echo '{xml_b64}' | base64 -d > /tmp/strategy_deploy.xml"

            # STEP 3: ros2 topic pub 명령으로 직접 전송 (Python 스크립트 없이)
            # --once: 한 번만 발행
            # data: XML 파일의 내용을 읽어서 전송 (쉘 치환 사용)
            pub_cmd = f"XML=$(cat /tmp/strategy_deploy.xml); ros2 topic pub --once {detected_topic} std_msgs/msg/String \"{{data: '$XML'}}\" 2>&1"

            full_cmd = f"bash -c '{setup_cmd}; {write_xml_cmd}; {pub_cmd}; echo Publish complete.'"
            # Note: XML 내에 따옴표 있으면 깨질 수 있으므로 base64 파이썬 스크립트로 fallback
            # 만약 위 방식이 안되면 아래 파이썬 방식으로 fallback
            py_code = f"""
import sys, os
os.environ['ROS_LOG_DIR'] = '/tmp'
import rclpy
from std_msgs.msg import String
import time

print("[Deploy] Starting deployment script...")
try:
    rclpy.init()
    node = rclpy.create_node('web_deployer_{robot_id}')
    pub = node.create_publisher(String, '{detected_topic}', 10)
    msg = String()
    if not os.path.exists('/tmp/strategy_deploy.xml'):
        sys.exit(1)
    with open('/tmp/strategy_deploy.xml', 'r') as f:
        msg.data = f.read()
    print(f"[Deploy] Publishing to {detected_topic}...")
    for i in range(5):
        pub.publish(msg)
        time.sleep(0.2)
    print("[Deploy] Publish complete.")
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"[Deploy Error] {{e}}")
    sys.exit(1)
"""
            py_b64 = base64.b64encode(py_code.encode('utf-8')).decode('utf-8')
            write_py_cmd = f"echo '{py_b64}' | base64 -d > /tmp/deploy.py"
            full_cmd = f"bash -c '{setup_cmd}; {write_xml_cmd}; {write_py_cmd}; python3 /tmp/deploy.py'"
            print(f"[Deploy] full_cmd topic = {detected_topic}")
            
            # 원격 실행
            client = self.clients[robot_id]
            stdin, stdout, stderr = client.exec_command(full_cmd)
            
            # 결과 처리
            out_str = stdout.read().decode()
            err_str = stderr.read().decode()
            
            print(f"[Deploy Log] {out_str}")
            if err_str: print(f"[Deploy Err] {err_str}")
                
            if "Publish complete" in out_str:
                 return True, "Deploy Success"
            else:
                 return False, f"Deploy Failed: {err_str if err_str else out_str}"
                 
        except Exception as e:
            self.logger.error(f"Deploy exception: {e}")
            return False, f"Deploy exception: {e}"

    # 시스템 로그 가져오기 -> 'tail' 명령어로 launcher.log 파일의 뒷부분만 짤라서 가져온다
    def fetch_log(self, robot_id, lines=50):
        if robot_id not in self.clients:
            return "Not connected"
            
        try:
            client = self.clients[robot_id]
            cmd = f"tail -n {lines} /home/booster/Workspace/GUI/INHA-Player/launcher.log"
            stdin, stdout, stderr = client.exec_command(cmd)
            
            log_content = stdout.read().decode()
            if not log_content:
                err = stderr.read().decode()
                return f"[Log Error] {err}" if err else "[Log] Empty"
            return log_content
        except Exception as e:
            return f"[SSH Error] {e}"

# 전역에서 하나만 존재해야 하는 인스턴스 (Singleton)
ssh_manager = SSHManager()

