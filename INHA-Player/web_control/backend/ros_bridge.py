import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    
    # game_controller_interface 메시지 임포트
    try:
        from game_controller_interface.msg import GameControlData
        GC_MSG_AVAILABLE = True
    except ImportError:
        print("[WARN] game_controller_interface not found. GC data via ROS disabled.")
        GC_MSG_AVAILABLE = False

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    GC_MSG_AVAILABLE = False
    class Node: pass

# 웹 서버와 ROS 2 시스템 간의 브릿지 역할을 하는 클래스
# 시뮬 환경에서는 SSH 없이 ROS2 토픽으로 직접 전략 배포
class ROSBridge(Node):
    def __init__(self):
        self.robot_status = {}
        self.strat_pubs = {}

        # GC 상태 저장소 (ROS 토픽에서 수신)
        self.gc_status = {
            "state": "UNKNOWN",
            "secsRemaining": 0,
            "teams": [
                {"teamNumber": 0, "color": 0, "score": 0, "penaltyCount": 0, "messageBudget": 12000},
                {"teamNumber": 0, "color": 1, "score": 0, "penaltyCount": 0, "messageBudget": 12000}
            ],
            "secondaryState": "NONE",
            "secondaryTime": 0
        }
        self.gc_available = False  # ROS GC 구독 성공 여부

        if ROS_AVAILABLE:
            super().__init__('web_bridge_node')

            # game_controller ROS 토픽 구독 (UDP 3838 대신 사용)
            if GC_MSG_AVAILABLE:
                try:
                    self.gc_sub = self.create_subscription(
                        GameControlData,
                        '/robocup/game_controller',
                        self._gc_callback,
                        10
                    )
                    self.gc_available = True
                    print("[ROS] Subscribed to /robocup/game_controller for GC data")
                except Exception as e:
                    print(f"[WARN] GC subscription failed: {e}")

            self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.spin_thread.start()
        else:
            print("[WARN] ROS 2 not detected. Running in Idle Mode.")

    def spin_ros(self):
        if ROS_AVAILABLE:
            rclpy.spin(self)

    def get_status(self):
        return self.robot_status

    def get_gc_status(self):
        """GC 상태 반환 (ROS 토픽에서 수신한 데이터)"""
        return self.gc_status

    def is_gc_available(self):
        return self.gc_available

    def _gc_callback(self, msg):
        """/robocup/game_controller 토픽 콜백"""
        try:
            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(int(msg.state), "UNKNOWN")

            SEC_STATE_MAP = {
                0: "NONE", 1: "PENALTY_SHOOT", 2: "OVERTIME", 3: "TIMEOUT",
                4: "DIRECT_FREEKICK", 5: "INDIRECT_FREEKICK", 6: "PENALTY_KICK",
                7: "CORNER_KICK", 8: "GOAL_KICK", 9: "THROW_IN"
            }
            sec_state_str = SEC_STATE_MAP.get(int(msg.secondary_state), "NONE")

            parsed_teams = []
            for team in msg.teams:
                penalty_count = sum(1 for p in team.players if p.penalty != 0)
                players_info = [
                    {
                        "penalty": int(p.penalty),
                        "secs_till_unpenalised": int(p.secs_till_unpenalised),
                        "yellow_cards": 0,
                        "red_cards": int(p.red_card_count) if hasattr(p, 'red_card_count') else 0,
                        "is_goalie": False
                    }
                    for p in team.players
                ]
                parsed_teams.append({
                    "teamNumber": int(team.team_number),
                    "color": int(team.field_player_colour) if hasattr(team, 'field_player_colour') else 0,
                    "score": int(team.score),
                    "penaltyCount": penalty_count,
                    "messageBudget": 12000,
                    "players": players_info
                })

            self.gc_status = {
                "state": state_str,
                "secsRemaining": int(msg.secs_remaining),
                "teams": parsed_teams,
                "secondaryState": sec_state_str,
                "secondaryTime": int(msg.secondary_time) if hasattr(msg, 'secondary_time') else 0
            }
        except Exception as e:
            print(f"[GC Callback] Parse error: {e}")

    # -----------------------------------------------------------------------------------------
    # 전략 XML을 ROS 토픽으로 발행하여 시뮬 brain에게 전송
    # -----------------------------------------------------------------------------------------
    def publish_strategy(self, robot_id, xml_content):
        if not ROS_AVAILABLE:
            print(f"[SIM] Deployed strategy to {robot_id} (ROS Not Available)")
            return False

        # brain namespace → strategy 토픽 매핑
        # brain이 ns:=robot1 로 실행 → /robot1/strategy/deploy
        # brain이 namespace 없이 실행   → /strategy/deploy
        # 현재 설정: robot1/2/3 사용
        SIM_NAMESPACES = ["robot1", "robot2", "robot3"]

        if robot_id == "all":
            targets = SIM_NAMESPACES
        else:
            # robot_id가 "all"이 아닌 경우 그대로 사용
            targets = [robot_id] if robot_id else SIM_NAMESPACES

        success_count = 0
        for ns in targets:
            if ns == "":
                topic = "/strategy/deploy"
            else:
                topic = f"/{ns}/strategy/deploy"

            if topic not in self.strat_pubs:
                self.strat_pubs[topic] = self.create_publisher(String, topic, 10)
                print(f"[ROS] Created publisher for {topic}")
                time.sleep(0.2)

            msg = String()
            msg.data = xml_content
            self.strat_pubs[topic].publish(msg)
            print(f"[ROS] Published strategy to {topic}")
            success_count += 1

        return success_count > 0


def init_ros():
    if ROS_AVAILABLE:
        rclpy.init(args=None)
    return ROSBridge()

ros_bridge = None
