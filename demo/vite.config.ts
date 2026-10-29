import { defineConfig } from 'vite';

// The demo imports the engine from ../src, outside the demo root.
export default defineConfig({
  root: import.meta.dirname,
  base: './',
  server: { fs: { allow: ['..'] }, port: 5173 },
  build: { outDir: 'dist', emptyOutDir: true, target: 'es2022' },
});
