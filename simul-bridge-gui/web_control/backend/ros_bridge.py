import threading
import json
import asyncio
import math
import random
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    
    ROS_AVAILABLE = True
except ImportError:
    # ROS 2 환경이 아닐 경우
    ROS_AVAILABLE = False
    class Node: pass # Dummy class

# 웹 서버와 ROS 2 시스템 간의 브릿지 역할을 하는 클래스 - 로봇의 상태(위치, 배터리 등)를 수신하고, 웹에서 내린 명령을 ROS 토픽으로 발행
class ROSBridge(Node):
    def __init__(self):
        self.robot_status = {} # 로봇들의 현재 상태 저장소
        
        if ROS_AVAILABLE:
            super().__init__('web_bridge_node')
            
            # ROS 2 콜백 처리를 위한 별도 스레드 시작
            self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.spin_thread.start()
        else:
            print("[WARN] ROS 2 not detected. Running in Idle Mode.")

    # ROS 2 이벤트 루프 실행
    def spin_ros(self):
        if ROS_AVAILABLE:
            rclpy.spin(self)

    # 현재 모든 로봇의 상태 반환
    def get_status(self):
        return self.robot_status

    # 전략 XML을 ROS 토픽으로 발행하여 로봇에게 전송
    def publish_strategy(self, robot_id, xml_content):
        if ROS_AVAILABLE:
            topic = f"/{robot_id}/strategy/deploy"
            
            # 퍼블리셔가 없으면 생성
            if not hasattr(self, 'strat_pubs'):
                self.strat_pubs = {}
            
            if robot_id not in self.strat_pubs:
                self.strat_pubs[robot_id] = self.create_publisher(String, topic, 10)
            
            msg = String()
            msg.data = xml_content
            self.strat_pubs[robot_id].publish(msg)
            print(f"[ROS] Published strategy to {topic}")
        else:
            # 시뮬레이션 모드에서는 로그만 출력
            print(f"[SIM] Deployed strategy to {robot_id} (ROS Not Available)")

# ROS 초기화 함수
def init_ros():
    if ROS_AVAILABLE:
        rclpy.init(args=None)
    return ROSBridge()

ros_bridge = None
