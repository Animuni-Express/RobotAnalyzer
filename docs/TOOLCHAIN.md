# TypeScript toolchain

Bun installs dependencies, runs scripts, and executes the tests with `bun test`. The normal type checker is tsrs, which reads the strict `tsconfig.json` and runs with `--noEmit --singleThreaded`. `bun run typecheck:tsc` runs the official TypeScript compiler for compatibility checks. [tsrs documentation](https://github.com/oxc-project/tsrs)

Vite 8 uses Rolldown for dependency optimization and production bundling. Oxc strips TypeScript, transforms JSX, supplies React Fast Refresh through the Vite React plugin, and minifies production JavaScript. Oxlint handles source linting without type-aware mode so type checking stays in tsrs. [Vite 8 architecture](https://vite.dev/blog/announcing-vite8) · [Oxlint CLI](https://oxc.rs/docs/guide/usage/linter/cli)

| Command | Work |
| --- | --- |
| `bun run dev` | Starts one Vite development server with React Fast Refresh. |
| `bun run typecheck` | Runs tsrs once with one checker. |
| `bun run typecheck:tsc` | Runs the official TypeScript compatibility check once. |
| `bun run lint` | Runs Oxlint once with one thread. |
| `bun run test` | Runs the Bun test suite once. |
| `bun run check` | Runs the type check, lint, and tests in sequence. |
| `bun run build` | Runs `check`, then emits the minified frontend to `dist`. |
| `bun run desktop:build` | Runs the compatibility check, then Tauri bundles the checked frontend with the native application. |
| `bun run tinyjs:dev` | Starts the TinyJS desktop window with Vite hot reload. |
| `bun run tinyjs:build` | Runs the compatibility check and checked frontend build, then packages the TinyJS portable app. |

On Windows the script wrapper starts each tool below normal process priority. Cargo builds one crate at a time and the wrapper sets `RAYON_NUM_THREADS=1`. The development server watches frontend files only. Production source maps and automatic gzip size reports are disabled to avoid extra build work. The packaged desktop application serves the bundled assets directly. [Windows scheduling priorities](https://learn.microsoft.com/en-us/windows/win32/procthread/scheduling-priorities)

## Build resource policy

Local Windows desktop commands configure a job-wide hard cap of 10% CPU before starting their child tools. Tauri commands use 4 GB committed memory because `tauri-utils` exhausted 1 GB and the Rust Windows bindings exhausted 2 GB. TinyJS commands use 1 GB. Nested frontend checks share the desktop command's budget. Standalone web commands do not apply the cap. GitHub Actions builds run remotely without the local job cap; Cargo still compiles one crate at a time. [CPU rate limits](https://learn.microsoft.com/en-us/windows/win32/api/winnt/ns-winnt-jobobject_cpu_rate_control_information) · [Job memory limits](https://learn.microsoft.com/en-us/windows/win32/api/winnt/ns-winnt-jobobject_basic_limit_information)

## Verification

Cloud Linux verification uses Bun 1.4.4. `bun run build` runs type checking, Oxlint, all robotics tests, and the Vite production bundle. `bun run typecheck:tsc` independently checks compatibility with the official TypeScript compiler. Browser QA uses the shared cloud Chromium. The current cloud browser cannot create a WebGL context, so compatibility projection, navigation, controls and plots can be tested there; hardware 3D performance requires a WebGL-capable machine.

Windows packaging runs through `.github/workflows/windows-desktop.yml` on pushes to `main` that affect application or build files, or on manual dispatch. Tauri uploads its executable and NSIS installer as `robot-analyzer-windows`; TinyJS uploads its portable ZIP as `robot-analyzer-tinyjs-windows`. The Tauri installer downloads the official WebView2 bootstrapper when needed. TinyJS requires WebView2 already installed.

Local Windows x64 verification uses Bun 1.4.4 and TinyJS 0.74.0. A separate check against the actual txiki.js runtime passed for file reads, replacement, cancellation, import size limits and window minimum sizing. Full native GUI interaction and hardware 3D performance have not been verified.
