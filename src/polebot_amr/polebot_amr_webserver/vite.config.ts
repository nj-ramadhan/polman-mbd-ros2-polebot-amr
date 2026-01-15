import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import tailwindcss from '@tailwindcss/vite'
import path from 'path';

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    vue(),
    tailwindcss()
  ],
  server: {
    allowedHosts: [
      'flexional-lorie-consanguineously.ngrok-free.dev'
    ],
    host: true
  },
  resolve: {
    alias: {
      roslib: path.resolve(__dirname, 'src/shims-roslib.ts'),
    },
  },
})
