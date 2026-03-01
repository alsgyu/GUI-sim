import React, { useState } from 'react';
import { AppBar, Toolbar, Typography, Container, Box, Tabs, Tab } from '@mui/material';
import Dashboard from './components/Dashboard';
import BlocklyEditor from './components/BlocklyEditor';

// 메인 애플리케이션 컴포넌트 -> 전체적인 레이아웃(헤더, 탭)을 잡고, 선택된 탭에 따라 하위 컴포넌트를 렌더링
function App() {
    const [tabIndex, setTabIndex] = useState(0);

    const handleTabChange = (_: React.SyntheticEvent, newValue: number) => {
        setTabIndex(newValue);
    };

    return (
        <Box sx={{ flexGrow: 1 }}>
            {/* 상단 헤더 바 */}
            <AppBar position="static">
                <Toolbar>
                    <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
                        INHA United Control Center
                    </Typography>
                </Toolbar>
            </AppBar>

            {/* 메인 컨텐츠 영역 */}
            <Container maxWidth="lg" sx={{ mt: 4 }}>
                {/* 기능 전환 탭 (대시보드 vs 전략 빌더) */}
                <Tabs value={tabIndex} onChange={handleTabChange} centered sx={{ mb: 3 }}>
                    <Tab label="Dashboard" />
                    <Tab label="Strategy Builder" />
                </Tabs>

                {/* 탭 인덱스에 따라 조건부 렌더링 */}
                {tabIndex === 0 && <Dashboard />}
                {tabIndex === 1 && <BlocklyEditor />}
            </Container>
        </Box>
    );
}

export default App;
