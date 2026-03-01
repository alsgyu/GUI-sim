import { Paper, Typography, Box, Grid, Chip } from '@mui/material';

interface PlayerInfo {
    penalty: number;
    secs_till_unpenalised: number;
    yellow_cards: number;
    red_cards: number;
    is_goalie: boolean;
}

interface TeamInfo {
    teamNumber: number;
    color: number; // 0: Blue, 1: Red
    score: number;
    penaltyCount: number;
    totalPenaltyCount?: number;
    messageBudget?: number;
    coachMessage?: string;
    players?: PlayerInfo[];
}

export interface GameInfo {
    state: string;
    secsRemaining: number;
    teams: TeamInfo[];
    secondaryState: string;
    secondaryTime: number;
    dropInTime?: number;
    dropInTeam?: number;
    gameType?: string;
}

interface Props {
    info: GameInfo;
}

const GameInfoBoard = ({ info }: Props) => {
    if (!info) return null;

    const formatTime = (secs: number) => {
        const m = Math.floor(secs / 60);
        const s = secs % 60;
        return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    };

    const teamColors = ['#2196f3', '#f44336']; // Blue, Red
    const teamNames = ['BLUE', 'RED'];

    return (
        <Paper sx={{ p: 2, height: '100%', bgcolor: '#f5f5f5', overflowY: 'auto' }}>
            <Grid container alignItems="center" spacing={2}>
                {/* [경기 시간 및 상태] */}
                {/* 남은 시간(분:초)과 현재 게임 단계(READY, PLAYING 등)를 가운데 정렬로 표시 */}
                <Grid item xs={4} sx={{ textAlign: 'center' }}>
                    <Typography variant="h3" sx={{ fontWeight: 'bold', fontFamily: 'monospace' }}>
                        {formatTime(info.secsRemaining)}
                    </Typography>

                    {/* Secondary Timer */}
                    {(info.secondaryTime > 0 || info.secondaryState !== 'NONE') && (
                        <Box sx={{ mt: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 1 }}>
                            <Chip
                                label={info.secondaryState}
                                size="small"
                                color="secondary"
                                variant="outlined"
                            />
                            {info.secondaryTime > 0 && (
                                <Typography variant="h5" color="secondary" sx={{ fontWeight: 'bold', fontFamily: 'monospace' }}>
                                    {info.secondaryTime}s
                                </Typography>
                            )}
                        </Box>
                    )}

                    <Box sx={{ mt: 1 }}>
                        <Chip
                            label={info.state}
                            color={info.state === 'PLAYING' ? 'success' : (info.state === 'READY' ? 'warning' : 'default')}
                            sx={{ fontWeight: 'bold' }}
                        />
                        {info.gameType === 'HL' && info.dropInTime !== undefined && (
                            <Chip
                                label={`DropIn: ${info.dropInTime}s`}
                                size="small"
                                sx={{ ml: 1, bgcolor: '#ddd' }}
                            />
                        )}
                    </Box>
                </Grid>

                {/* 스코어보드 */}
                {/* BLUE팀과 RED팀의 점수를 나란히 표시 */}
                {info.teams && info.teams.map((team, idx) => (
                    <Grid item xs={4} key={idx} sx={{ textAlign: 'center', borderLeft: idx === 1 ? '1px solid #ddd' : 'none' }}>
                        <Typography variant="h6" sx={{ color: teamColors[idx] || 'gray', fontWeight: 'bold' }}>
                            {teamNames[idx] || `TEAM ${team.teamNumber}`}
                        </Typography>
                        <Typography variant="h2" sx={{ fontWeight: 'bold' }}>
                            {team.score}
                        </Typography>

                        {/* 페널티 정보 */}
                        {team.players && team.players.some(p => p.penalty !== 0) && (
                            <Box sx={{ mt: 1, maxHeight: 60, overflowY: 'auto' }}>
                                {team.players.map((p, pIdx) => (
                                    p.penalty !== 0 && p.secs_till_unpenalised > 0 && (
                                        <Typography key={pIdx} variant="caption" display="block" color="error" sx={{ fontWeight: 'bold' }}>
                                            P{pIdx + 1}: {p.secs_till_unpenalised}s
                                        </Typography>
                                    )
                                ))}
                            </Box>
                        )}

                        {/* 페널티 총 횟수 */}
                        {team.totalPenaltyCount !== undefined && (
                            <Typography variant="body2" sx={{ mt: 1, color: '#d32f2f', fontWeight: 'bold' }}>
                                Total Penalties: {team.totalPenaltyCount}
                            </Typography>
                        )}

                        {/* Summary */}
                        {(!team.players && team.penaltyCount > 0) && (
                            <Typography variant="caption" color="error" display="block">
                                Penalties: {team.penaltyCount}
                            </Typography>
                        )}

                        {team.messageBudget !== undefined && (
                            <Typography variant="caption" color="textSecondary" display="block">
                                Msg Budget: {team.messageBudget}
                            </Typography>
                        )}

                        {/* Coach Message - 삭제 */}
                        {team.coachMessage && (
                            <Typography variant="caption" display="block" sx={{ mt: 0.5, fontStyle: 'italic', color: '#666', border: '1px dashed #ccc', borderRadius: 1, p: 0.5 }}>
                                Coach: "{team.coachMessage}"
                            </Typography>
                        )}
                    </Grid>
                ))}
            </Grid>
        </Paper>
    );
};

export default GameInfoBoard;
