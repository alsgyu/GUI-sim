import { useState, useEffect } from 'react';
import { Dialog, DialogTitle, DialogContent, TextField, DialogActions, Button, CircularProgress } from '@mui/material';
import axios from 'axios';

// SSH 연결 다이얼로그 프로퍼티
interface SSHConnectionDialogProps {
    open: boolean;              // 다이얼로그 열림 여부
    onClose: () => void;        // 닫기 핸들러
    onConnected: (id: string) => void; // 연결 성공 시 콜백
    initialRobotId?: string;    // 초기 로봇 ID (선택 사항)
}

// SSH 연결 다이얼로그 컴포넌트 -> 사용자가 로봇의 IP, 계정, 비밀번호를 입력하여 SSH 연결을 요청하는 팝업 창
const SSHConnectionDialog = ({ open, onClose, onConnected, initialRobotId }: SSHConnectionDialogProps) => {
    // 입력 필드 상태 관리
    const [id, setId] = useState(initialRobotId || 'robot_1');
    const [ip, setIp] = useState('192.168.0.233'); // 기본값: 대회용 로봇 IP 대역
    const [username, setUsername] = useState('booster'); // 기본 계정명
    const [password, setPassword] = useState('123456');  // 기본 비밀번호
    const [loading, setLoading] = useState(false);       // 로딩 상태 (연결 중일 때 true)

    // initialRobotId가 변경되면 ID 필드 자동 업데이트
    useEffect(() => {
        if (initialRobotId) {
            setId(initialRobotId);
        }
    }, [initialRobotId]);

    // 연결 요청 핸들러 -> Connect 버튼 클릭 시 백엔드(/api/connect)로 연결 요청 보냄
    const handleConnect = async () => {
        setLoading(true);
        try {
            // 백엔드 API 호출: 로봇 연결 시도
            await axios.post('http://localhost:8000/api/connect', {
                id,
                ip,
                username,
                password
            });
            // 성공 시 알림 및 콜백 호출
            alert(`Connected to ${id} successfully!`);
            onConnected(id);
            onClose();
        } catch (e: any) {
            // 실패 시 에러 메시지 표시
            alert(`Failed to connect: ${e.response?.data?.detail || e.message}`);
        } finally {
            setLoading(false);
        }
    };

    return (
        <Dialog open={open} onClose={onClose}>
            <DialogTitle>Connect to Robot (SSH)</DialogTitle>
            <DialogContent>
                {/* 로봇 ID 입력 */}
                <TextField
                    autoFocus
                    margin="dense"
                    label="Robot ID"
                    type="text"
                    fullWidth
                    value={id}
                    onChange={(e) => setId(e.target.value)}
                />
                {/* IP 주소 입력 */}
                <TextField
                    margin="dense"
                    label="IP Address"
                    type="text"
                    fullWidth
                    value={ip}
                    onChange={(e) => setIp(e.target.value)}
                />
                {/* SSH 사용자명 입력 */}
                <TextField
                    margin="dense"
                    label="Username"
                    type="text"
                    fullWidth
                    value={username}
                    onChange={(e) => setUsername(e.target.value)}
                />
                {/* SSH 비밀번호 입력 */}
                <TextField
                    margin="dense"
                    label="Password"
                    type="password"
                    fullWidth
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                />
            </DialogContent>
            <DialogActions>
                {/* 취소 버튼 */}
                <Button onClick={onClose} disabled={loading}>Cancel</Button>

                {/* 연결 버튼, 로딩 중에는 스피너 표시 */}
                <Button onClick={handleConnect} disabled={loading} variant="contained">
                    {loading ? <CircularProgress size={24} /> : "Connect"}
                </Button>
            </DialogActions>
        </Dialog>
    );
};

export default SSHConnectionDialog;
