
import socket
import struct
import threading
import time
import json
import math

# 팀 통신 패킷 가져오는 파일

# C++ TeamCommunicationMsg Structure Layout (Standard Alignment)
# int validation;        // 0-3
# int communicationId;   // 4-7
# int teamId;            // 8-11
# int playerId;          // 12-15
# int playerRole;        // 16-19
# bool isAlive;          // 20
# bool isLead;           // 21
# bool ballDetected;     // 22
# bool ballLocationKnown;// 23
# double ballConfidence; // 24-31 (8-byte aligned)
# double ballRange;      // 32-39
# double cost;           // 40-47
# Point ballPosToField;  // 48-71 (3 doubles)
# Pose2D robotPoseToField;// 72-95 (3 doubles)
# double kickDir;        // 96-103
# double thetaRb;        // 104-111
# int cmdId;             // 112-115
# int cmd;               // 116-119
# bool passSignal;       // 120
# -- PADDING --          // 121-127 (7 bytes) to align next double to 128
# double passTargetX;    // 128-135
# double passTargetY;    // 136-143
# Total Size: 144 bytes

# Format definition for struct.unpack
# < : Little Endian
# 5i: 5 integers
# 4?: 4 bools
# 3d: 3 doubles (ball info)
# 3d: 3 doubles (Point ballPos)
# 3d: 3 doubles (Pose2D robotPos)
# 2d: 2 doubles (kickDir, thetaRb)
# 2i: 2 integers (cmdId, cmd)
# ?: 1 bool (passSignal)
# 7x: 7 pad bytes
# 2d: 2 doubles (passTarget)
FMT = "<5i4?3d3d3d2d2i?7x2d" # FMT는 수신된 UDP 바이너리 패킷을 struct.unpack으로 풀기 위한 포맷 문자열
EXPECTED_SIZE = struct.calcsize(FMT) # 이 구조체 정확한 바이트 크기가 144니까 수신 데이터 길이가 이 값이랑 다르면 무시하도록

class UDPMonitor:
    def __init__(self, team_id=1):
        self.team_id = team_id
        
        # 팀 통신 수신 포트 (시뮬레이션 환경용: 30000 + 팀ID * 100 + 수신자ID)
        # C++ 코드 참조 (BrainCommunication::initCommunication)
        self.port = 30000 + self.team_id * 100 + 99
        
        # UDP 소켓 생성 및 바인딩
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 포트 재사용 허용
        try:
            self.sock.bind(('0.0.0.0', self.port)) # 모든 인터페이스에서 수신
            print(f"[UDP] Listening on port {self.port}")
        except Exception as e:
            print(f"[UDP] Bind error: {e}")
            self.sock = None

        self.robots = {} # 수신된 로봇 상태 데이터 저장소 { "robot_ID": { ... } }
        self.running = True # 스레드 종료 flag
        
        # 1. 수신 스레드 시작
        # 들어오는 패킷을 계속해서 받아서 파싱
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        
        # 2. Discovery(생존 신고) 방송 스레드 시작
        # 로봇은 '아는 팀원'에게만 데이터를 보냄
        # 따라서 Mac이 주기적으로 "나도 팀원(99번)이야"라고 알려줘야 로봇이 데이터를 보내줌
        self.discovery_thread = threading.Thread(target=self.broadcast_discovery, daemon=True)
        self.discovery_thread.start()

    # 즉 init을 호출하면 바로 수신 스레드 + Discovery 송신 스레드가 동작함

    # [수신 루프]
    def loop(self):
        if not self.sock: return
        while self.running:
            try:
                # 1024 버퍼 크기로 데이터 수신
                data, addr = self.sock.recvfrom(1024)
                
                # 데이터 크기가 예상된 구조체 크기와 일치하는지 확인 - 길이가 EXPECTED_SIZE 이상이면 일치하는 만큼 잘라서 parse_packet() 호출
                if len(data) >= EXPECTED_SIZE:
                    self.parse_packet(data[:EXPECTED_SIZE], addr)
                else:
                    # 크기가 안 맞으면 무시 (다른 버전이거나 깨진 패킷)
                    pass
            except Exception as e:
                print(f"[UDP] Error: {e}")
                time.sleep(1)

    # [Discovery 브로드캐스트]
    # 로봇들에게 Mac의 존재를 알리는 함수
    def broadcast_discovery(self):
        # Discovery Port 규칙: 20000 + team_id (RoboCup 프로토콜)
        dest_port = 20000 + self.team_id
        
        # 송신용 소켓 생성
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # 브로드캐스트 패킷 허용
        
        # TeamDiscoveryMsg 구조체 (16 bytes)
        # int validation = 41203; (매직 넘버)
        # int communicationId = 0;
        # int teamId;
        # int playerId; (Mac을 99번 선수로 위장하여 구별)
        validation = 41203
        comm_id = 0
        player_id = 99 # 99번 선수로 위장
        
        print(f"[UDP] Broadcasting Discovery to port {dest_port}...")
        
        while self.running:
            try:
                # 데이터를 리틀 엔디안(<) 정수(i) 4개로 패킹 - struct.pack("<4i", ...) 해서 16바이트 전송.
                # -> 로봇들의 팀원 목록에 이 GUI를 포함하게 됨
                msg = struct.pack("<4i", validation, comm_id, self.team_id, player_id)
                # 브로드캐스트 주소로 전송
                sock.sendto(msg, ('<broadcast>', dest_port))
                # Ubuntu 등 동일 PC 환경에서 브로드캐스트가 로컬호스트 스택을 타지 않는 경우를 대비해 127.0.0.1로도 직접 전송
                sock.sendto(msg, ('127.0.0.1', dest_port))
                
                comm_id += 1
                time.sleep(1.0) # 1초마다 생존 신고 (너무 자주 보내면 대역폭 낭비)
            except Exception as e:
                print(f"[UDP] Discovery Broadcast Error: {e}")
                time.sleep(1.0)

    # [패킷 파싱]
    # 바이너리 데이터를 python 딕셔너리로 변환 - 수신한 바이너리 패킷을 의미 있는 값으로 해석
    def parse_packet(self, data, addr):
        print(f"[UDP] Received {len(data)} bytes from {addr}") # Verbose debug
        try:
            # struct.unpack을 사용하여 바이너리 데이터 해독 (튜플)
            unpacked = struct.unpack(FMT, data)

            # 데이터 추출
            # <5i 4? 3d 3d 3d 2d 2i ? 7x 2d
            # 0-4: int (val, commId, teamId, playerId, role)
            # 5-8: bool (isAlive, isLead, ballDetected, ballLocKnown)
            # 9-11: double (ballConf, ballRange, cost)
            # 12-14: double (ballPos.x, .y, .z)
            # 15-17: double (robotPos.x, .y, .theta)
            # 20-21: int (cmdId, cmd)

            team_id = unpacked[2]
            player_id = unpacked[3]
            role_int = unpacked[4]
            
            is_alive = unpacked[5]
            is_lead = unpacked[6]
            ball_detected = unpacked[7]
            ball_range = unpacked[10]
            
            cmd = unpacked[21]

            # 로봇 ID 생성
            robot_id = f"robot_{player_id}"
            
            # Role 매핑 (TeamCommunicationMsg) - role_map으로 정수 → 문자열
            role_map = {0: "Unknown", 1: "Striker", 2: "Defender", 3: "Goalkeeper", 4: "Support"}
            role_str = role_map.get(role_int, f"Role {role_int}")

            # 위치 정보
            rx = unpacked[15]
            ry = unpacked[16]
            rtheta = unpacked[17]
            
            # 공 정보
            bx = unpacked[12]
            by = unpacked[13] # Ball Z is unpacked[14]

            # 상태 설명
            if not is_alive:
                state_desc = "Fallen"
            elif ball_detected:
                state_desc = f"Ball Found ({ball_range:.2f}m)"
            else:
                state_desc = "Searching"

            # [Packet Monitor] PPS & Size Calculation
            now = time.time()
            packet_size = len(data)
            
            # Initialize metrics for new robot
            if robot_id not in self.robots:
                # Store temporary metrics in separate dict if needed, or just partial update
                # Since we overwrite self.robots[robot_id], we need to check previous value or maintain separate state
                # Let's use a separate dict for tracking pps state
                pass
            
            # Update PPS tracking
            self._update_pps(robot_id, now)

            # 이제 마지막으로 self.robots[robot_id]에 아래 정보들 저장
            self.robots[robot_id] = {
                "id": robot_id,
                "role": role_str,
                "state": state_desc, # Frontend displays this
                "is_alive": is_alive,
                "ball_detected": ball_detected,
                "cmd": cmd,
                "x": rx,
                "y": ry,
                "theta": rtheta,
                "ball_x": bx,
                "ball_y": by,
                "ball_confidence": unpacked[9],
                "last_seen": now,
                "ip": addr[0],
                "packet_size": packet_size,
                "pps": self.packet_rates.get(robot_id, 0.0) # 초당 패킷수
            }
        except Exception as e:
            print(f"[UDP] Parse error: {e}")

    def _update_pps(self, robot_id, now): # 각 로봇별로 최근 1초 동안 받은 패킷 수 계산
        if not hasattr(self, 'packet_stats'):
            self.packet_stats = {} 
            self.packet_rates = {}

        if robot_id not in self.packet_stats:
            self.packet_stats[robot_id] = []
        
        # self.packet_stats에 타임스탬프를 축적
        self.packet_stats[robot_id].append(now)
        
        # 1초 넘은 것은 제거
        self.packet_stats[robot_id] = [t for t in self.packet_stats[robot_id] if now - t <= 1.0]
        
        # 결과 카운트를 self.packet_rates에 저장
        self.packet_rates[robot_id] = len(self.packet_stats[robot_id])

    def get_status(self):
        # last_seen이 3초 이상 지난 로봇은 삭제 - 남아있는 self.robots 반환
        now = time.time()
        expired = [rid for rid, r in self.robots.items() if now - r['last_seen'] > 3.0]
        for rid in expired:
            del self.robots[rid]
            # Also clean up stats
            if hasattr(self, 'packet_stats') and rid in self.packet_stats:
                del self.packet_stats[rid]
                del self.packet_rates[rid]
                
        return self.robots

udp_monitor = UDPMonitor(team_id=13) # 이 줄이 실행되면서 실제 모니터가 즉시 동작
