import js from '@eslint/js';
import tseslint from 'typescript-eslint';

export default tseslint.config(
  { ignores: ['node_modules', 'dist', 'demo/dist', 'coverage', 'bench/results', '.claude'] },
  js.configs.recommended,
  ...tseslint.configs.recommended,
  { rules: { '@typescript-eslint/no-explicit-any': 'error' } },
  // TypeScript already reports undefined names; the core rule only knows Node globals.
  { files: ['**/*.ts'], rules: { 'no-undef': 'off' } },
  {
    files: ['bench/**', 'tests/**'],
    languageOptions: {
      globals: { console: 'readonly', process: 'readonly', performance: 'readonly' },
    },
  },
);
