import { spawnSync } from 'bun';
import { applyQuietJob } from './quiet-job';

const local = ['run', '--bun'];
const commands = new Map<string, string[]>([
  ['dev', [...local, 'vite', '--host', '127.0.0.1', '--port', '1420', '--strictPort']],
  ['typecheck', [...local, 'tsrs', '--noEmit', '--singleThreaded']],
  ['tsc', [...local, 'tsc', '--noEmit', '--singleThreaded']],
  ['lint', [...local, 'oxlint', '--threads=1']],
  ['test', ['test']],
  ['bundle', [...local, 'vite', 'build']],
  ['desktop-dev', [...local, 'tauri', 'dev']],
  ['desktop-build', [...local, 'tauri', 'build']],
  ['tinyjs-dev', ['tinyjs', 'dev']],
  ['tinyjs-build', ['tinyjs', 'build']],
]);
const [mode, ...extra] = process.argv.slice(2);
const command = mode && commands.get(mode);
if (!command) {
  console.error(`Choose one of ${[...commands.keys()].join(', ')}.`);
  process.exit(2);
}

const args = [...command, ...extra];
if (process.platform === 'win32') applyQuietJob(mode === 'desktop-build' ? 4_000_000_000n : 1_000_000_000n);
const env = { ...process.env, CARGO_BUILD_JOBS: '1', RAYON_NUM_THREADS: '1', GOMAXPROCS: '1',
  ...(process.platform === 'win32' ? { OS: 'Windows_NT' } : {}),
};
const result = spawnSync({
  cmd: mode!.startsWith('tinyjs-') ? args : [process.execPath, ...args],
  env,
  stdin: 'inherit', stdout: 'inherit', stderr: 'inherit',
});
process.exit(result.exitCode ?? 1);
