import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';

export default defineConfig({
  plugins: [react()],
  test: {
    environment: 'jsdom',
    setupFiles: ['./vitest.setup.ts'],
    globals: true,
    alias: {
      '@site': path.resolve(__dirname, '.'),
    },
  },
  resolve: {
    alias: {
      '@site': path.resolve(__dirname, '.'),
    },
  },
});