import js from '@eslint/js';
import tseslint from 'typescript-eslint';

export default tseslint.config(
  { ignores: ['node_modules', 'dist', 'coverage', 'bench/results'] },
  js.configs.recommended,
  ...tseslint.configs.recommended,
  { rules: { '@typescript-eslint/no-explicit-any': 'error' } },
  {
    files: ['bench/**', 'tests/**'],
    languageOptions: {
      globals: { console: 'readonly', process: 'readonly', performance: 'readonly' },
    },
  },
);
