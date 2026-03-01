import { useEffect, useRef } from 'react';
import { Paper, Typography } from '@mui/material';

// [상수 정의] 경기장 및 캔버스 설정
const FIELD_WIDTH = 800;  // 캔버스 가로 크기 (픽셀)
const FIELD_HEIGHT = 600; // 캔버스 세로 크기 (픽셀)
const REAL_WIDTH = 14.0;  // 실제 경기장 가로 크기 (미터)
const REAL_HEIGHT = 9.0;  // 실제 경기장 세로 크기 (미터)

// 컴포넌트 프로퍼티
interface FieldVisualizerProps {
    robots: { [key: string]: any }; // 로봇 상태 데이터 객체 (위치, 역할, 공 정보 등)
}

// 경기장 시각화 컴포넌트
const FieldVisualizer = ({ robots }: FieldVisualizerProps) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);

    // 렌더링 루프
    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // 1. 경기장 배경 및 라인 그리기
        drawField(ctx);

        // 2. 각 로봇 그리기
        Object.entries(robots).forEach(([id, data]: [string, any]) => {
            if (data.x !== undefined && data.y !== undefined) {
                drawRobot(ctx, id, data.x, data.y, data.role);
            }
        });

        // 3. 공 그리기 - 가장 신뢰도가 높은 공 하나만 표시
        let bestBall: { x: number, y: number, conf: number, robotId: string } | null = null;

        Object.entries(robots).forEach(([id, data]: [string, any]) => {
            if (data.ball_detected && data.ball_x !== undefined && data.ball_y !== undefined) {
                const conf = data.ball_confidence || 0;
                // 기존 베스트보다 신뢰도가 높으면 교체
                if (!bestBall || conf > bestBall.conf) {
                    bestBall = { x: data.ball_x, y: data.ball_y, conf: conf, robotId: id };
                }
            }
        });

        if (bestBall) {
            drawBall(ctx, bestBall);
        }

    }, [robots]);

    // 좌표 변환 함수
    const toCanvasCoords = (x: number, y: number) => {
        const cx = (x + REAL_WIDTH / 2) / REAL_WIDTH * FIELD_WIDTH;
        const cy = ((-y) + REAL_HEIGHT / 2) / REAL_HEIGHT * FIELD_HEIGHT;
        return { cx, cy };
    };

    // 경기장 그리기
    const drawField = (ctx: CanvasRenderingContext2D) => {
        ctx.fillStyle = '#4CAF50';
        ctx.fillRect(0, 0, FIELD_WIDTH, FIELD_HEIGHT);

        ctx.strokeStyle = 'white';
        ctx.lineWidth = 4;
        ctx.strokeRect(20, 20, FIELD_WIDTH - 40, FIELD_HEIGHT - 40);

        ctx.beginPath();
        ctx.moveTo(FIELD_WIDTH / 2, 20);
        ctx.lineTo(FIELD_WIDTH / 2, FIELD_HEIGHT - 20);
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(FIELD_WIDTH / 2, FIELD_HEIGHT / 2, 60, 0, Math.PI * 2);
        ctx.stroke();
    };

    // 로봇 그리기
    const drawRobot = (ctx: CanvasRenderingContext2D, id: string, x: number, y: number, role: string) => {
        const { cx, cy } = toCanvasCoords(x, y);

        ctx.beginPath();
        ctx.arc(cx, cy, 15, 0, Math.PI * 2);

        if (id === 'robot_1') ctx.fillStyle = 'red';      // GK
        else if (id === 'robot_2') ctx.fillStyle = 'blue'; // Striker
        else ctx.fillStyle = 'yellow';                    // Others

        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = 'white';
        ctx.font = '12px Arial';
        ctx.fillText(id, cx - 15, cy - 20);
        ctx.fillText(role || '?', cx - 15, cy + 30);
    };

    // 공 그리기 함수
    const drawBall = (ctx: CanvasRenderingContext2D, ball: { x: number, y: number, conf: number, robotId: string }) => {
        const { cx, cy } = toCanvasCoords(ball.x, ball.y);

        // 공 본체 (주황색)
        ctx.beginPath();
        ctx.arc(cx, cy, 10, 0, Math.PI * 2);
        // 신뢰도가 높을수록 불투명하게 (최소 0.4)
        ctx.fillStyle = `rgba(255, 140, 0, ${0.4 + ball.conf * 0.6})`;
        ctx.fill();
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 2;
        ctx.stroke();

        // 텍스트 (신뢰도 %)
        ctx.fillStyle = 'white';
        ctx.font = '10px Arial';
        ctx.fillText(`C:${(ball.conf * 100).toFixed(0)}%`, cx + 12, cy + 5);
        ctx.fillText(`by ${ball.robotId}`, cx + 12, cy + 15);
    };

    return (
        <Paper elevation={3} sx={{ p: 2, display: 'inline-block' }}>
            <Typography variant="h6" gutterBottom>Field View</Typography>
            <canvas
                ref={canvasRef}
                width={FIELD_WIDTH}
                height={FIELD_HEIGHT}
                style={{ border: '1px solid #ccc', borderRadius: '4px' }}
            />
        </Paper>
    );
};

export default FieldVisualizer;
