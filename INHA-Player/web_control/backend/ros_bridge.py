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

# 웹 서버와 ROS 2 시스템 간의 브릿지 역할을 하는 클래스
# 시뮬 환경에서는 SSH 없이 ROS2 토픽으로 직접 전략 배포
class ROSBridge(Node):
    def __init__(self):
        self.robot_status = {} # 로봇들의 현재 상태 저장소
        self.strat_pubs = {}   # 퍼블리셔 캐시: {topic: publisher}
        
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

    # -----------------------------------------------------------------------------------------
    # 전략 XML을 ROS 토픽으로 발행하여 시뮬 brain에게 전송
    #
    # 시뮬에서 brain은 ns:=robot0 처럼 namespace를 붙여 실행됨.
    # 따라서 실제 토픽은 /robot0/strategy/deploy 형태가 됨.
    #
    # robot_id 예시:
    #   "all"    → 등록된 모든 시뮬 로봇에 broadcast
    #   "robot0" → /robot0/strategy/deploy 에만 발행
    # -----------------------------------------------------------------------------------------
    def publish_strategy(self, robot_id, xml_content):
        if not ROS_AVAILABLE:
            print(f"[SIM] Deployed strategy to {robot_id} (ROS Not Available)")
            return False

        # 대상 로봇 목록 결정
        # "all" 이면 현재 알려진 시뮬 로봇 네임스페이스 전체에 발행
        SIM_NAMESPACES = ["robot0", "robot1", "robot2", "robot3", "robot4"]
        
        if robot_id == "all":
            targets = SIM_NAMESPACES
        else:
            # robot_id가 이미 네임스페이스 형태(robot0 등)인 경우 그대로 사용
            targets = [robot_id]

        success_count = 0
        for ns in targets:
            topic = f"/{ns}/strategy/deploy"

            # 퍼블리셔가 없으면 새로 생성 (캐시)
            if topic not in self.strat_pubs:
                self.strat_pubs[topic] = self.create_publisher(String, topic, 10)
                print(f"[ROS] Created publisher for {topic}")
                # 퍼블리셔가 구독자를 찾을 시간 확보
                time.sleep(0.2)

            msg = String()
            msg.data = xml_content
            self.strat_pubs[topic].publish(msg)
            print(f"[ROS] Published strategy to {topic}")
            success_count += 1

        return success_count > 0

# ROS 초기화 함수
def init_ros():
    if ROS_AVAILABLE:
        rclpy.init(args=None)
    return ROSBridge()

ros_bridge = None
