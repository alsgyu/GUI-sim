import { useState, useEffect } from 'react';
import { Dialog, DialogTitle, DialogContent, Box, Typography, Button, FormControl, Select, MenuItem, Paper } from '@mui/material';
import axios from 'axios';

interface Props {
    open: boolean;
    onClose: () => void;
    robots: string[];
}

const LogViewer = ({ open, onClose, robots }: Props) => {
    const [selectedRobot, setSelectedRobot] = useState<string>('');
    const [logContent, setLogContent] = useState<string>('Select a robot to view logs.');

    useEffect(() => {
        if (robots.length > 0 && !selectedRobot) {
            setSelectedRobot(robots[0]);
        }
    }, [robots, open]);

    // 로그 새로고침 함수 -> 백엔드의 GET /api/logs/{id} API를 호출하여 최신 로그 데이터를 가져옴
    const fetchLogs = async () => {
        if (!selectedRobot) return;
        try {
            // launcher.log의 마지막 100줄을 응답받음
            const res = await axios.get(`http://localhost:8000/api/logs/${selectedRobot}`);
            setLogContent(res.data.log);
        } catch (e) {
            setLogContent("Failed to fetch logs.");
            console.error(e);
        }
    };

    // 로봇 선택이 바뀌면 자동으로 새로운 로그를 가져옴
    useEffect(() => {
        if (open && selectedRobot) {
            fetchLogs();
        }
    }, [selectedRobot, open]);

    return (
        <Dialog open={open} onClose={onClose} maxWidth="md" fullWidth>
            <DialogTitle>
                <Box display="flex" justifyContent="space-between" alignItems="center">
                    <Typography variant="h6">System Logs</Typography>
                    <Box display="flex" gap={1}>
                        <FormControl size="small" sx={{ minWidth: 120 }}>
                            <Select
                                value={selectedRobot}
                                onChange={(e) => setSelectedRobot(e.target.value)}
                                displayEmpty
                            >
                                <MenuItem value="" disabled>Select Robot</MenuItem>
                                {robots.map(id => <MenuItem key={id} value={id}>{id}</MenuItem>)}
                            </Select>
                        </FormControl>
                        {/* Refresh 버튼을 누르면 fetchLogs()가 재실행되어 최신 로그를 반영 */}
                        <Button variant="outlined" onClick={fetchLogs}>Refresh</Button>
                        <Button onClick={onClose}>Close</Button>
                    </Box>
                </Box>
            </DialogTitle>
            <DialogContent dividers>
                <Paper component="pre" sx={{
                    p: 2,
                    my: 0,
                    bgcolor: '#1e1e1e',
                    color: '#00ff00',
                    fontFamily: 'monospace',
                    overflowX: 'auto',
                    minHeight: '400px',
                    maxHeight: '600px',
                    fontSize: '0.85rem'
                }}>
                    {logContent}
                </Paper>
            </DialogContent>
        </Dialog>
    );
};

export default LogViewer;
