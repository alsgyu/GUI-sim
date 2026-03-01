import { useState, useEffect } from 'react';
import { Paper, Typography, Button, Box, TextField, FormControl, Select, MenuItem } from '@mui/material';
import axios from 'axios';

// 컴포넌트 프로퍼티
interface CommandPanelProps {
    robotId: string; // 제어할 단일 로봇 ID
    strategies: string[];
    selectedStrategy: string;
    onStrategyChange: (robotId: string, strategy: string) => void;
}

// SSH 명령 제어 패널 -> 특정 로봇에 대해 쉘 명령을 내리고 터미널 출력을 확인하는 컴포넌트
const CommandPanel = ({ robotId, strategies, selectedStrategy, onStrategyChange }: CommandPanelProps) => {
    const [customCmd, setCustomCmd] = useState('');
    const [logs, setLogs] = useState<string[]>([]);

    // 로봇 ID가 바뀌면 로그 초기화
    useEffect(() => {
        setLogs([]);
    }, [robotId]);

    const addLog = (msg: string) => {
        const timestamp = new Date().toLocaleTimeString();
        setLogs(prev => [`[${timestamp}] ${msg}`, ...prev]);
    };

    const sendCommand = async (cmd: string) => {
        if (!robotId) return;

        addLog(`Sending: ${cmd}`);
        try {
            const res = await axios.post('http://localhost:8000/api/command', {
                robot_id: robotId,
                cmd: cmd
            });
            if (res.data.stdout) addLog(`OUT: ${res.data.stdout}`);
            if (res.data.stderr) addLog(`ERR: ${res.data.stderr}`);
        } catch (e: any) {
            addLog(`Error: ${e.response?.data?.detail || e.message}`);
        }
    };

    const handleApplyStrategy = async () => {
        if (!selectedStrategy) {
            alert("전략을 먼저 선택해주세요!");
            return;
        }
        try {
            const res = await axios.get(`http://localhost:8000/api/strategies/${selectedStrategy}`);
            await axios.post('http://localhost:8000/api/deploy_strategy', {
                robot_id: robotId,
                strategy_xml: res.data.xml
            });
            alert(`[SUCCESS] ${selectedStrategy} -> ${robotId}`);
        } catch (e: any) {
            alert(`[FAIL] ${e.response?.data?.detail || e.message}`);
        }
    };

    return (
        <Paper elevation={3} sx={{ p: 2, height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Typography variant="h6" gutterBottom>
                Control Panel ({robotId})
            </Typography>

            {/* 전략 선택 부분 */}
            <Paper variant="outlined" sx={{ p: 1, mb: 2, bgcolor: '#f5f5f5' }}>
                <Typography variant="subtitle2" gutterBottom>Strategy Control</Typography>
                <Box sx={{ display: 'flex', gap: 1 }}>
                    <FormControl size="small" fullWidth>
                        <Select
                            value={selectedStrategy || ''}
                            onChange={(e) => onStrategyChange(robotId, e.target.value)}
                            displayEmpty
                            sx={{ bgcolor: 'white' }}
                        >
                            <MenuItem value="" disabled>Select Strategy</MenuItem>
                            {strategies.map(s => <MenuItem key={s} value={s}>{s}</MenuItem>)}
                        </Select>
                    </FormControl>
                    <Button
                        variant="contained"
                        sx={{ bgcolor: '#9c27b0', '&:hover': { bgcolor: '#7b1fa2' } }}
                        onClick={handleApplyStrategy}
                    >
                        APPLY
                    </Button>
                </Box>
            </Paper>

            {/* SSH 명령어 부분 */}
            <Typography variant="subtitle2" gutterBottom>System Commands</Typography>
            <Box sx={{ display: 'flex', gap: 1, mb: 2, flexWrap: 'wrap' }}>
                <Button
                    variant="contained"
                    color="primary"
                    size="small"
                    onClick={() => sendCommand(`cd /home/booster/Workspace/Soccer || echo "Dir not found"; source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null; nohup ./scripts/start.sh ns:=${robotId} > brain_nohup.log 2>&1 & echo "Executed start.sh from $PWD"`)}
                >
                    START
                </Button>
                <Button
                    variant="contained"
                    color="error"
                    size="small"
                    onClick={() => sendCommand('pkill -f brain_node')}
                >
                    STOP
                </Button>
                <Button
                    variant="outlined"
                    color="warning"
                    size="small"
                    onClick={() => sendCommand('echo "123456" | sudo -S reboot')}
                >
                    REBOOT
                </Button>
                <Button
                    variant="outlined"
                    color="info"
                    size="small"
                    onClick={() => sendCommand('cd /home/booster/Workspace/Soccer; echo "=== brain_nohup.log ==="; tail -n 20 brain_nohup.log; echo "\\n=== brain.log ==="; tail -n 20 brain.log')}
                >
                    LOGS
                </Button>
            </Box>

            {/* Custom Command */}
            <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                <TextField
                    fullWidth
                    size="small"
                    placeholder="Custom command..."
                    value={customCmd}
                    onChange={(e) => setCustomCmd(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && sendCommand(customCmd)}
                />
                <Button variant="contained" size="small" onClick={() => sendCommand(customCmd)}>
                    SEND
                </Button>
            </Box>

            {/* 터미널 출력 */}
            <Box sx={{
                bgcolor: 'black',
                color: '#00ff00',
                p: 2,
                borderRadius: 1,
                flexGrow: 1,
                minHeight: '200px',
                overflowY: 'auto',
                fontFamily: 'monospace',
                fontSize: '0.85rem'
            }}>
                {logs.length === 0 ? (
                    <span style={{ color: 'gray' }}>// Ready for commands...</span>
                ) : (
                    logs.map((log, i) => (
                        <div key={i}>{log}</div>
                    ))
                )}
            </Box>
        </Paper>
    );
};

export default CommandPanel;
