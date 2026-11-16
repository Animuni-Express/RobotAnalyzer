import react from '@vitejs/plugin-react';
import { defineConfig } from 'vite';

export default defineConfig({
  base: './',
  plugins: [react()],
  build: { minify: 'oxc', sourcemap: false, reportCompressedSize: false },
  server: { watch: { ignored: ['**/src-tauri/**', '**/.cache/**'] } },
});
