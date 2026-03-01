import { useEffect, useRef } from 'react';
import { Paper, Typography, Box, Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Chip } from '@mui/material';

export interface StateLog {
    id: number;
    timestamp: string;
    robotId: string;
    type: 'STATE' | 'ACTION';
    oldValue: string;
    newValue: string;
}

interface Props {
    logs: StateLog[];
}

const StateHistoryBoard = ({ logs }: Props) => {
    const scrollRef = useRef<HTMLDivElement>(null);

    // Auto-scroll to bottom when logs update
    useEffect(() => {
        if (scrollRef.current) {
            scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
        }
    }, [logs]);

    return (
        <Paper sx={{ p: 2, height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Typography variant="h6" gutterBottom>
                State/Action History
            </Typography>

            <TableContainer component={Box} sx={{ flexGrow: 1, overflow: 'auto', maxHeight: 300 }} ref={scrollRef}>
                <Table size="small" stickyHeader>
                    <TableHead>
                        <TableRow>
                            <TableCell>Time</TableCell>
                            <TableCell>Robot</TableCell>
                            <TableCell>Type</TableCell>
                            <TableCell>Change</TableCell>
                        </TableRow>
                    </TableHead>
                    <TableBody>
                        {logs.length === 0 ? (
                            <TableRow>
                                <TableCell colSpan={4} align="center" sx={{ color: 'text.secondary', py: 3 }}>
                                    No history yet...
                                </TableCell>
                            </TableRow>
                        ) : (
                            logs.map((log) => (
                                <TableRow key={log.id} hover>
                                    <TableCell sx={{ fontFamily: 'monospace', fontSize: '0.85rem' }}>
                                        {log.timestamp}
                                    </TableCell>
                                    <TableCell>
                                        <Chip
                                            label={log.robotId}
                                            size="small"
                                            variant="outlined"
                                            sx={{ borderRadius: 1, height: 20, fontSize: '0.75rem' }}
                                        />
                                    </TableCell>
                                    <TableCell>
                                        <Chip
                                            label={log.type}
                                            size="small"
                                            color={log.type === 'ACTION' ? 'primary' : 'default'}
                                            sx={{ height: 20, fontSize: '0.7rem' }}
                                        />
                                    </TableCell>
                                    <TableCell sx={{ fontSize: '0.9rem' }}>
                                        <span style={{ color: '#888' }}>{log.oldValue}</span>
                                        {' ➔ '}
                                        <span style={{ fontWeight: 'bold' }}>{log.newValue}</span>
                                    </TableCell>
                                </TableRow>
                            ))
                        )}
                    </TableBody>
                </Table>
            </TableContainer>
        </Paper>
    );
};

export default StateHistoryBoard;
