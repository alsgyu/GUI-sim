import socket
import struct
import threading
import time

GAMECONTROLLER_DATA_PORT = 3838 # 겜컨이 브로드캐스트하는 기본 포트
GAMECONTROLLER_STRUCT_VERSION = 15 # RoboCupGameControlData.h
MAX_NUM_PLAYERS = 20

# -----------------------------------------------------------------------------------------
# [GameController 모니터]
# RoboCup 공식 GameController(v15)에서 보내는 UDP 패킷을 수신하고 파싱하는 클래스
# -----------------------------------------------------------------------------------------
class GCMonitor:
    def __init__(self):
        # UDP 통신하겠다 선언 - 소켓 생성(TCP와 달리 연결 확인 없이 빠르게 데이터만 던지는 방식)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass 
        self.sock.settimeout(0.5)  # recvfrom()이 0.5초마다 타임아웃 되도록 해서 무한 블로킹 방지
        
        try:
            # 심판 서버가 쓰는 3838번 포트 연결
            self.sock.bind(('', GAMECONTROLLER_DATA_PORT))
            print(f"[GC] Listening on port {GAMECONTROLLER_DATA_PORT}")
        except Exception as e:
            print(f"[GC] Bind error: {e}")
            self.sock = None

        self.running = True
        
        # 파싱된 데이터 저장소 (2팀) - GUI 기본 필드 구성
        self.data = {
            "state": "UNKNOWN",
            "secsRemaining": 0,
            "teams": [
                {"score": 0, "penaltyCount": 0, "totalPenaltyCount": 0},
                {"score": 0, "penaltyCount": 0, "totalPenaltyCount": 0}
            ],
            "secondaryState": "NONE",
            "secondaryTime": 0
        }

        # 누적 페널티 카운트 추적용 (재시작 시 초기화됨)
        self.team_total_penalties = [0, 0]

        # 이전 프레임의 선수별 페널티 상태 [팀0[20명], 팀1[20명]]
        self.prev_players_penalty = [[0]*MAX_NUM_PLAYERS, [0]*MAX_NUM_PLAYERS]

        # 통신을 기다리는 동안 멈추면 안되므로 스레드를 시켜 돌려둠
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self): # UDP 수신 루프 - 서버가 켜져있는 동안 계속 돔
        if not self.sock: return
        while self.running:
            try:
                # 데이터가 올 때까지 기다림 - 데이터가 오면 data 변수에 넣음
                data, addr = self.sock.recvfrom(4096)
                
                # 심판이 보내는 데이터는 항상 'RGme'로 시작함 - 받은 패킷이 b'RGme' 헤더로 시작하면 parse_packet() 호출
                if len(data) >= 4 and data[0:4] == b'RGme':
                     self.parse_packet(data)

            except socket.timeout:
                pass 
            except Exception as e:
                print(f"[GC] Error: {e}")
                time.sleep(1)

    # 여기서 패킷 종류 판별과 버전 분기를 담당
    def parse_packet(self, data):
        print(f"[GC] Received {len(data)} bytes")
        try:
            # 1. 헤더 확인 'RGme'
            if len(data) < 4: 
                print("[GC] Data too short")
                return
            if data[0:4] != b'RGme': 
                print(f"[GC] Invalid header: {data[0:4]}")
                return

            print(f"[GC] Header OK. Version Byte: {data[4]}")

            # 2. 버전 확인
            if len(data) < 6: return
            
            # SPL v15 또는 v18 확인 - data[4] 값이 15 또는 18이면 SPL 패킷, data[4:6]를 uint16으로 읽었을 때 12이면 HL 패킷
            if data[4] == 15 or data[4] == 18:
                print(f"[GC] Detected SPL v{data[4]} Packet")
                self.parse_spl_packet(data)
                return

            # HL v12
            version = struct.unpack("<H", data[4:6])[0]
            
            # If version is 12, it is Humanoid League
            if version == 12:
                print(f"[GC] Detected HL v12 Packet (ver={version})")
                self.parse_hl_packet(data)
            else:
                print(f"[GC] Unknown version: {version} (uint16) or {data[4]} (uint8)")

        except Exception as e:
            print(f"[GC] Parse error: {e}")

    def parse_spl_packet(self, data): # SPL(v15 또는 v18) 패킷 파서
        try:
            if len(data) < 18: return
            (header, version, packet_num, players_per_team, 
             comp_phase, comp_type, game_phase, 
             state, set_play, first_half, kicking_team, 

             # 통신으로 들어오는 이진수 덩어리를 다시 글자로 바꾸기 - 데이터의 앞 4바이트는 글자(s)로 읽고 그다음은 숫자(B)로 읽는 규칙 정의
             secs_remaining, secondary_time) = struct.unpack("<4sBBBBBBBBBBhh", data[0:18]) # 헤더 파싱

            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")
            
            SET_PLAY_MAP = {0: "NONE", 1: "GOAL_KICK", 2: "PUSHING_FREE_KICK", 
                            3: "CORNER_KICK", 4: "KICK_IN", 5: "PENALTY_KICK"}
            set_play_str = SET_PLAY_MAP.get(set_play, "NONE")

            # INITIAL 상태로 돌아가면 누적 페널티 카운터 초기화
            if state_str == "INITIAL":
               
                if self.data["state"] != "INITIAL": # 상태가 변경된 순간에만 로그 출력
                     print("[GC] State changed to INITIAL. Resetting Team Penalty Counts.")
                     self.team_total_penalties = [0, 0]
                     self.prev_players_penalty = [[0]*MAX_NUM_PLAYERS, [0]*MAX_NUM_PLAYERS]

            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = set_play_str
            self.data["secondaryTime"] = secondary_time
            self.data["gameType"] = "SPL"
            # 프론트엔드 오류 방지를 위해 HL 관련 필드는 기본값으로 채움
            self.data["dropInTime"] = 0
            self.data["dropInTeam"] = 0
            
            # SPL 팀 정보 파싱 - 각 팀마다 팀번호, 색상, 골키퍼 색상, 점수 등
            offset = 18
            TEAM_INFO_SIZE = 10 + (20 * 2) # 50 바이트
            parsed_teams = []
            
            for i in range(2):
                if len(data) < offset + TEAM_INFO_SIZE: break
                (team_num, field_color, gk_color, gk_num, score, penalty_shot, 
                 single_shots, msg_budget) = struct.unpack("<6B2H", data[offset : offset+10])
                
                players_offset = offset + 10
                penalty_count = 0
                players_info = []

                for p in range(20):
                    p_base = players_offset + (p * 2)
                    p_penalty, p_secs = struct.unpack("BB", data[p_base : p_base+2])
                    
                    # 유효한 선수만 처리
                    if p < players_per_team:
                        if p_penalty != 0: penalty_count += 1
                        
                        # [누적 페널티 로직]
                        # 누적되어야 하는 특정 페널티만 카운트 (30s, 45s, 60s...)
                        # 1: Illegal Ball Contact, 2: Pushing, 6: Leaving Field, 10: Stance
                        # 7(PickUp)은 규칙서상 누적이지만, 사용자가 Incapable과 혼용할 수 있어 제외함
                        # 3(Motion), 4(Incapable), 5(Illegal Position), 8(Local Stuck)은 규칙서상 누적 안됨
                        cumulative_codes = {1, 2, 6, 10} 
                        
                        # 이전에 페널티가 없었다가(0) 이번에 새로운 페널티 발생
                        if self.prev_players_penalty[i][p] == 0 and p_penalty != 0:
                            is_cumulative = p_penalty in cumulative_codes
                            print(f"[GC] New Penalty: Team {i}, Player {p+1}, Code {p_penalty}, Cumulative: {is_cumulative}")
                            
                            if is_cumulative:
                                self.team_total_penalties[i] += 1
                                print(f"[GC] -> Total Penalties Increment to {self.team_total_penalties[i]}")
                        
                        # 이전 상태 업데이트
                        self.prev_players_penalty[i][p] = p_penalty

                        players_info.append({
                            "penalty": p_penalty, 
                            "secs_till_unpenalised": p_secs,
                            "yellow_cards": 0,
                            "red_cards": 0,
                            "is_goalie": False
                        })
                    else:
                        pass

                parsed_teams.append({
                    "teamNumber": team_num,
                    "color": field_color,
                    "score": score,
                    "penaltyCount": penalty_count,
                    "totalPenaltyCount": self.team_total_penalties[i],
                    "messageBudget": msg_budget,
                    "coachMessage": "",
                    "players": players_info
                })
                offset += TEAM_INFO_SIZE
            
            self.data["teams"] = parsed_teams
            print(f"[GC] SPL Parsed: {state_str}, Time: {secs_remaining}")
            
        except Exception as e:
            print(f"[GC] SPL Parse error: {e}")

    def parse_hl_packet(self, data): # HL(Humanoid League) v12 패킷 파서
        try:
            if len(data) < 24: return
            
            (header, version, packet_num, players_per_team,
             game_type, state, first_half, kick_off_team,
             sec_state, sec_state_info_bytes, drop_in_team,
             drop_in_time, secs_remaining, secondary_time) = struct.unpack("<4sHBBBBBBB4sBHHH", data[0:24]) # 들어온 바이너리 데이터를 리그 규격으로 잘라서 숫자나 문자로 변환

            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")
            
            # HL Secondary States (RoboCupGameControlData.h line 48)
            SEC_STATE_MAP = {
                0: "NONE", 1: "PENALTY_SHOOT", 2: "OVERTIME", 3: "TIMEOUT",
                4: "DIRECT_FREEKICK", 5: "INDIRECT_FREEKICK", 6: "PENALTY_KICK",
                7: "CORNER_KICK", 8: "GOAL_KICK", 9: "THROW_IN"
            }
            sec_state_str = SEC_STATE_MAP.get(sec_state, "NONE")

            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = sec_state_str
            self.data["secondaryTime"] = secondary_time
            self.data["dropInTime"] = drop_in_time
            self.data["dropInTeam"] = drop_in_team
            self.data["gameType"] = "HL"

            # Teams (HL)
            # Offset 24 starts TeamInfo[2]
            offset = 24
            TEAM_INFO_SIZE = 332
            parsed_teams = []
            
            for i in range(2):
                if len(data) < offset + TEAM_INFO_SIZE: break
                
                # Header: 7 bytes
                # teamNumber(1), fieldPlayerColour(1), score(1), penaltyShot(1), singleShots(2), coachSequence(1)
                (team_num, color, score, penalty_shot, single_shots, coach_seq) = struct.unpack("<BBBBHB", data[offset:offset+7])

                # Coach Message: 253 bytes (Offset + 7)
                coach_msg_bytes = data[offset+7 : offset+7+253]
                try:
                    coach_msg = coach_msg_bytes.decode('utf-8').rstrip('\x00')
                except:
                    coach_msg = ""
                    
                # 팀 시작 위치에서 266바이트(7+253+6)를 건너뛰면, 거기서부터 선수들 정보가 나온다
                # Players: Offset + 7 + 253 + 6
                # Players start at Offset + 266
                players_offset = offset + 266
                penalty_count = 0
                players_info = []

                # HL_MAX_NUM_PLAYERS = 11
                for p in range(11):
                    p_base = players_offset + (p * 6)
                    # HlRobotInfo: penalty(1), secsTillUnpenalised(1), warnings(1), yellow(1), red(1), goalie(1)
                    (p_penalty, p_secs, p_warn, p_yellow, p_red, p_goalie) = struct.unpack("BBBBBB", data[p_base : p_base+6])
                    
                    if p_penalty != 0: penalty_count += 1
                    players_info.append({
                        "penalty": p_penalty,
                        "secs_till_unpenalised": p_secs,
                        "yellow_cards": p_yellow,
                        "red_cards": p_red,
                        "is_goalie": (p_goalie > 0)
                    })

                parsed_teams.append({
                    "teamNumber": team_num,
                    "color": color,
                    "score": score,
                    "penaltyCount": penalty_count,
                    "coachMessage": coach_msg,
                    "players": players_info
                })
                
                offset += TEAM_INFO_SIZE

            self.data["teams"] = parsed_teams

        except Exception as e:
            print(f"[GC] HL Parse error: {e}")

    def get_status(self):
        return self.data

gc_monitor = GCMonitor()

# 
