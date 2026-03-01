import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App.tsx'

// React 애플리케이션의 진입점, index.html의 'root' div 요소에 메인 App 컴포넌트 렌더링
ReactDOM.createRoot(document.getElementById('root')!).render(
    <React.StrictMode>
        <App />
    </React.StrictMode>,
)
