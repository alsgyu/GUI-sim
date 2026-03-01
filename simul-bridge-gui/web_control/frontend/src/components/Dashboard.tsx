import { useState, useEffect, useRef } from 'react';
import { Box, Grid, Paper, Typography, Button, FormControl, Select, MenuItem, Chip } from '@mui/material';
import useWebSocket from 'react-use-websocket';
import axios from 'axios';
import FieldVisualizer from './FieldVisualizer';
import CommandPanel from './CommandPanel';
import SSHConnectionDialog from './SSHConnectionDialog';
import GameInfoBoard, { GameInfo } from './GameInfoBoard';
import LogViewer from './LogViewer';
import StateHistoryBoard, { StateLog } from './StateHistoryBoard';
import GameLogBoard, { GameLog } from './GameLogBoard';
import { PENALTY_MAP } from '../constants/PenaltyTypes';

// 대시보드 컴포넌트 -> 로봇 상태 카드와 SSH 명령 패널을 통합하여 보여주는 메인 화면
const DashboardComp = () => {
    // 1. 로봇 상태 데이터 (웹소켓 수신)
    const [robots, setRobots] = useState<{ [key: string]: any }>({});
    const prevRobotsRef = useRef<{ [key: string]: any }>({}); // 이전 상태 추적용

    // History 기록
    const [historyLogs, setHistoryLogs] = useState<StateLog[]>([]);
    const logIdCounter = useRef(0);

    // 버그 방지 디폴트 값 넣어주기
    const defaultGameInfo: GameInfo = {
        state: "WAITING",
        secsRemaining: 0,
        teams: [
            { teamNumber: 0, color: 0, score: 0, penaltyCount: 0, messageBudget: 12000 },
            { teamNumber: 0, color: 1, score: 0, penaltyCount: 0, messageBudget: 12000 }
        ],
        secondaryState: "NONE",
        secondaryTime: 0
    };
    const [gameInfo, setGameInfo] = useState<GameInfo | null>(defaultGameInfo);

    // 2. SSH 연결 상태
    const [connectedRobots, setConnectedRobots] = useState<string[]>([]);

    // 3. 현재 제어 패널이 열려있는 로봇 ID
    const [controlTarget, setControlTarget] = useState<string | null>(null);

    // 4. SSH 연결 다이얼로그 상태
    const [sshDialogOpen, setSSHDialogOpen] = useState(false);
    const [sshTarget, setSshTarget] = useState<string | null>(null);

    // 5. 로그 뷰어 상태
    const [logViewerOpen, setLogViewerOpen] = useState(false);

    // 6. 전략 목록 및 각 로봇별 선택된 전략
    const [strategies, setStrategies] = useState<string[]>([]);
    const [selectedStrategies, setSelectedStrategies] = useState<{ [key: string]: string }>({});

    const [gameLogs, setGameLogs] = useState<GameLog[]>([]);
    const gameLogIdCounter = useRef(0);
    const prevGameInfoRef = useRef<GameInfo | null>(defaultGameInfo);

    // 웹소켓 연결
    const WS_URL = 'ws://localhost:8000/ws/status';
    const { lastMessage } = useWebSocket(WS_URL, {
        shouldReconnect: () => true,
        retryOnError: true,
    });

    useEffect(() => {
        if (lastMessage !== null) {
            try {
                const data = JSON.parse(lastMessage.data);

                // 로봇 상태 업데이트 (data.robots)
                if (data.robots) {
                    setRobots(prev => {
                        const newRobots = { ...prev, ...data.robots };

                        // History Logging Logic
                        Object.keys(data.robots).forEach(robotId => {
                            const newRobot = data.robots[robotId];
                            const prevRobot = prevRobotsRef.current[robotId];

                            if (prevRobot) {
                                const now = new Date().toLocaleTimeString('en-US', { hour12: false });

                                // Check State Refresh (Ball Found vs Searching)
                                if (newRobot.state !== prevRobot.state) {
                                    const newLog: StateLog = {
                                        id: logIdCounter.current++,
                                        timestamp: now,
                                        robotId: robotId,
                                        type: 'STATE',
                                        oldValue: prevRobot.state || 'Unknown',
                                        newValue: newRobot.state || 'Unknown'
                                    };
                                    setHistoryLogs(prevLogs => [...prevLogs.slice(-99), newLog]); // Keep last 100
                                }

                                // Check Action Change
                                if (newRobot.action !== prevRobot.action && newRobot.action !== undefined) {
                                    const newLog: StateLog = {
                                        id: logIdCounter.current++,
                                        timestamp: now,
                                        robotId: robotId,
                                        type: 'ACTION',
                                        oldValue: prevRobot.action || 'NONE',
                                        newValue: newRobot.action
                                    };
                                    setHistoryLogs(prevLogs => [...prevLogs.slice(-99), newLog]);
                                }
                            }
                        });

                        prevRobotsRef.current = newRobots;
                        return newRobots;
                    });
                }

                if (data.game_info) {
                    const newInfo = data.game_info as GameInfo;
                    const prevInfo = prevGameInfoRef.current;

                    if (prevInfo && prevInfo.teams && newInfo.teams) {
                        const teamColors = ["BLUE", "RED"];

                        newInfo.teams.forEach((newTeam, tIdx) => {
                            const prevTeam = prevInfo.teams[tIdx];
                            if (!prevTeam || !newTeam.players) return;

                            newTeam.players.forEach((newPlayer, pIdx) => {
                                const prevPlayer = prevTeam.players ? prevTeam.players[pIdx] : null;

                                if (prevPlayer && prevPlayer.penalty === 0 && newPlayer.penalty !== 0) {
                                    const minutes = Math.floor(newInfo.secsRemaining / 60);
                                    const seconds = newInfo.secsRemaining % 60;
                                    const gameTime = `${minutes}:${seconds.toString().padStart(2, '0')}`;

                                    const reason = PENALTY_MAP[newPlayer.penalty] || `Unknown Penalty`;

                                    const newLog: GameLog = {
                                        id: gameLogIdCounter.current++,
                                        timestamp: gameTime,
                                        team: teamColors[tIdx],
                                        playerNum: pIdx + 1,
                                        eventType: "PENALTY",
                                        description: `${reason} (${newPlayer.secs_till_unpenalised}s)`
                                    };
                                    setGameLogs(prev => [...prev.slice(-49), newLog]); // Keep last 50
                                }
                            });
                        });
                    }

                    setGameInfo(newInfo);
                    prevGameInfoRef.current = newInfo;
                }
            } catch (e) {
                console.error("WS Parse Error", e);
            }
        }
    }, [lastMessage]);

    // 초기 전략 목록 로드
    useEffect(() => {
        axios.get('http://localhost:8000/api/strategies')
            .then(res => setStrategies(res.data.strategies))
            .catch(err => console.error("Strategy Fetch Error", err));
    }, []);

    // 핸들러 -> 전략 선택 변경
    const handleStrategyChange = (robotId: string, strategy: string) => {
        setSelectedStrategies(prev => ({ ...prev, [robotId]: strategy }));
    };

    // 핸들러 -> 전략 적용
    const handleApplyStrategy = async (robotId: string) => {
        const strategy = selectedStrategies[robotId];
        if (!strategy) {
            alert("전략을 먼저 선택해주세요!");
            return;
        }
        try {
            // XML 내용 가져오기
            const res = await axios.get(`http://localhost:8000/api/strategies/${strategy}`);
            // 배포 요청
            await axios.post('http://localhost:8000/api/deploy_strategy', {
                robot_id: robotId,
                strategy_xml: res.data.xml
            });
            alert(`[SUCCESS] ${strategy} -> ${robotId}`);
        } catch (e: any) {
            alert(`[FAIL] ${e.response?.data?.detail || e.message}`);
        }
    };

    // 핸들러 -> 비상 정지
    const handleEmergencyStop = async () => {
        if (!window.confirm("EMERGENCY STOP: Are you sure you want to stop ALL robots?")) return;

        try {
            await axios.post('http://localhost:8000/api/emergency_stop');
            alert("Emergency Stop Command Sent!");
        } catch (e: any) {
            alert(`E-Stop Failed: ${e.message}`);
        }
    };

    // 핸들러 -> SSH 연결 완료
    const handleSSHConnected = (robotId: string) => {
        if (!connectedRobots.includes(robotId)) {
            setConnectedRobots([...connectedRobots, robotId]);
        }
        setSSHDialogOpen(false);
    };

    return (
        <Box sx={{ flexGrow: 1, p: 3 }}>
            {/* 상단 바: 타이틀 & E-STOP */}
            <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
                <Typography variant="h4" sx={{ fontWeight: 'bold', color: '#333' }}>
                    INHA Player Control
                </Typography>
                <Box>
                    <Button
                        variant="contained"
                        color="secondary"
                        sx={{ mr: 2 }}
                        onClick={() => setLogViewerOpen(true)}
                    >
                        VIEW SYSTEM LOGS
                    </Button>
                    <Button
                        variant="contained"
                        color="error"
                        size="large"
                        sx={{ fontWeight: 'bold', px: 4, py: 1.5, fontSize: '1.2rem', boxShadow: 3 }}
                        onClick={handleEmergencyStop}
                    >
                        EMERGENCY STOP
                    </Button>
                </Box>
            </Box>

            {/* 1. 게임 정보 보드 & 이벤트 로그 */}
            <Grid container spacing={3} sx={{ mb: 6, height: { xs: 'auto', md: '200px' } }}>
                <Grid item xs={12} md={8} sx={{ height: '100%' }}>
                    {gameInfo && <GameInfoBoard info={gameInfo} />}
                </Grid>
                <Grid item xs={12} md={4} sx={{ height: '100%' }}>
                    <GameLogBoard logs={gameLogs} />
                </Grid>
            </Grid>

            <Grid container spacing={3}>
                {/* 2. 로봇 상태 카드 목록 */}
                {/* 연결된 로봇들의 개별 상태(배터리, 역할, 전략)를 카드 형태로 나열 */}
                {['robot1', 'robot2', 'robot3', 'robot4', 'robot5'].map(id => {
                    const robot = robots[id] || {};
                    const isConnected = connectedRobots.includes(id);
                    // 역할 뱃지 & 상세 상태
                    const roleLabel = robot.role || (id === 'robot1' ? 'GK' : 'Field');

                    return (
                        <Grid item xs={12} sm={6} md={2.4} key={id}>
                            <Paper sx={{ p: 2, border: controlTarget === id ? '2px solid #1976d2' : '1px solid #ddd', position: 'relative' }}>
                                {/* 연결 상태 표시 (점) */}
                                <Box sx={{
                                    position: 'absolute', top: 10, right: 10,
                                    width: 12, height: 12, borderRadius: '50%',
                                    bgcolor: isConnected ? '#4caf50' : '#bdbdbd'
                                }} />

                                {/* 헤더: ID & Role */}
                                <Box sx={{ mb: 1 }}>
                                    <Typography variant="h6" sx={{ fontWeight: 'bold' }}>{id.toUpperCase()}</Typography>
                                    <Chip label={roleLabel} size="small" color="primary" variant="outlined" sx={{ mt: 0.5 }} />
                                </Box>

                                <Typography variant="body2" sx={{ mb: 0.5 }}>
                                    Battery: {robot.battery ? `${robot.battery.toFixed(1)} V` : 'N/A'}
                                </Typography>
                                <Typography variant="caption" display="block" sx={{ color: '#555', mb: 1, fontFamily: 'monospace' }}>
                                    Comm: {robot.pps ? robot.pps.toFixed(1) : '0.0'} Hz | {robot.packet_size || 0} B
                                </Typography>

                                {/* 추가 정보 표시: 공 감지, 킥 등 */}
                                {robot.ball_x !== undefined && (
                                    <Chip label="Ball Found" size="small" color="success" sx={{ mb: 1, mr: 0.5 }} />
                                )}

                                {/* 전략 선택 & 적용 섹션 */}
                                {/* 사용자가 드롭다운에서 전략(.xml)을 선택하고 APPLY 버튼을 누르면 해당 로봇에 배포됨 */}
                                <Paper variant="outlined" sx={{ p: 1, mb: 2, bgcolor: '#f9f9f9' }}>
                                    <FormControl fullWidth size="small" sx={{ mb: 1 }}>
                                        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                                            <Select
                                                value={selectedStrategies[id] || ''}
                                                onChange={(e) => handleStrategyChange(id, e.target.value)}
                                                displayEmpty
                                                sx={{ fontSize: '0.8rem', flexGrow: 1 }}
                                            >
                                                <MenuItem value="" disabled>Strategy</MenuItem>
                                                {strategies.map(s => <MenuItem key={s} value={s}>{s}</MenuItem>)}
                                            </Select>
                                            <Button
                                                size="small"
                                                sx={{ minWidth: '30px', p: 0 }}
                                                onClick={() => {
                                                    axios.get('http://localhost:8000/api/strategies')
                                                        .then(res => setStrategies(res.data.strategies))
                                                        .catch(() => alert("Refresh Failed"));
                                                }}
                                            >
                                                ↻
                                            </Button>
                                        </div>
                                    </FormControl>
                                    <Button
                                        variant="contained"
                                        fullWidth
                                        size="small"
                                        sx={{ bgcolor: '#9c27b0', '&:hover': { bgcolor: '#7b1fa2' } }} // 보라색 버튼
                                        onClick={() => handleApplyStrategy(id)}
                                    >
                                        APPLY
                                    </Button>
                                </Paper>



                                {/* 연결 및 제어 버튼 */}
                                {/* 로봇과 아직 SSH 연결이 안되어 있으면 CONNECT, 되어 있으면 CONTROL 버튼 표시 */}
                                {isConnected ? (
                                    <Button
                                        variant="contained"
                                        color="success" // 초록색 버튼
                                        fullWidth
                                        size="small"
                                        onClick={() => setControlTarget(id)}
                                    >
                                        CONTROL
                                    </Button>
                                ) : (
                                    <Button
                                        variant="contained"
                                        color="primary" // 파란색 버튼
                                        fullWidth
                                        size="small"
                                        onClick={() => {
                                            setSshTarget(id);
                                            setSSHDialogOpen(true);
                                        }}
                                    >
                                        CONNECT
                                    </Button>
                                )}
                            </Paper>
                        </Grid>
                    );
                })}

                {/* 3. 하단 영역: 경기장 시각화 & 선택된 로봇의 커맨드 센터 */}
                <Grid item xs={12}>
                    <Grid container spacing={3}>
                        {/* 좌측: Field Visualizer */}

                        <Grid item xs={12} md={controlTarget ? 6 : 12}>
                            <Paper sx={{ p: 2, display: 'flex', justifyContent: 'center' }}>
                                <FieldVisualizer robots={robots} />
                            </Paper>
                        </Grid>

                        {/* 우측 (또는 전체): SSH Command Center + History Board */}
                        {controlTarget && (
                            <Grid item xs={12} md={6}>
                                <Grid container spacing={2}>
                                    {/* Left half: Command Panel */}
                                    <Grid item xs={12}>
                                        <CommandPanel
                                            robotId={controlTarget}
                                            strategies={strategies}
                                            selectedStrategy={selectedStrategies[controlTarget] || ''}
                                            onStrategyChange={(robotId: string, strategy: string) => setSelectedStrategies({ ...selectedStrategies, [robotId]: strategy })}
                                        />
                                    </Grid>
                                    {/* Right half: State History Panel (filtered for this robot) */}
                                    <Grid item xs={12}>
                                        <StateHistoryBoard logs={historyLogs.filter(log => log.robotId === controlTarget)} />
                                    </Grid>
                                    <Grid item xs={12}>
                                        <Button onClick={() => setControlTarget(null)} fullWidth variant="outlined" sx={{ mt: 1 }}>
                                            Close Control Panel
                                        </Button>
                                    </Grid>
                                </Grid>
                            </Grid>
                        )}
                    </Grid>
                </Grid>
            </Grid>

            {/* SSH 연결 다이얼로그 */}
            <SSHConnectionDialog
                open={sshDialogOpen}
                onClose={() => setSSHDialogOpen(false)}
                onConnected={handleSSHConnected}
                initialRobotId={sshTarget || 'robot_1'}
            />

            {/* 로그 뷰어 다이얼로그 */}
            <LogViewer
                open={logViewerOpen}
                onClose={() => setLogViewerOpen(false)}
                robots={connectedRobots}
            />
        </Box>
    );
};
export default DashboardComp;
