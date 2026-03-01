import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Vite 빌드 도구 설정 파일
// React 플러그인을 사용하여 프로젝트를 빌드하고 개발 서버를 실행
// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  build: {
    rollupOptions: {
      output: {
        entryFileNames: `assets/[name]-[hash]-${Date.now()}.js`,
        chunkFileNames: `assets/[name]-[hash]-${Date.now()}.js`,
        assetFileNames: `assets/[name]-[hash]-${Date.now()}.[ext]`
      }
    }
  }
})
