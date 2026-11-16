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

On Windows the script wrapper starts each tool below normal process priority. Cargo builds one crate at a time and the wrapper sets `RAYON_NUM_THREADS=1`. The development server watches frontend files only. Production source maps and automatic gzip size reports are disabled to avoid extra build work. The packaged desktop application serves the bundled assets directly. [Windows scheduling priorities](https://learn.microsoft.com/en-us/windows/win32/procthread/scheduling-priorities)

## Verification and measurements

The Windows launcher configures a job-wide hard cap of 10% CPU and 1 GB of committed memory before starting Bun. Tauri desktop builds use 4 GB because the Rust Windows bindings exceed 2 GB during compilation. TinyJS builds and frontend tools retain the 1 GB budget. Nested tool invocations share the same budget. If Windows rejects the limits or refuses to attach the launcher to the job, the command stops before starting the tool. [CPU rate limits](https://learn.microsoft.com/en-us/windows/win32/api/winnt/ns-winnt-jobobject_cpu_rate_control_information) · [Job memory limits](https://learn.microsoft.com/en-us/windows/win32/api/winnt/ns-winnt-jobobject_basic_limit_information)

On 2026-11-16, `bun run tinyjs:build` passed on Windows x64 with Bun 1.4.4 and TinyJS 0.74.0: both TypeScript checkers, lint (with existing warnings), all 18 tests, the Vite production build and TinyJS executable packaging completed. A separate check against the actual txiki.js runtime passed for file reads, atomic replacement, cancellation, import size limits and window minimum sizing. Native GUI verification was unavailable because computer-use app approval timed out. Performance measurements below remain pending.

| Metric | Result |
| --- | --- |
| Cold and warm type-check wall time | Pending |
| Development startup to first served page | Pending |
| Checked production build wall time | Pending |
| Raw bundle size and largest chunks | Pending |
| Peak build process memory | Pending |

The prior workspace contained a Bun starter without a comparable frontend production pipeline. Record the tool versions, Windows power mode, and exact command with each measurement. Measure once with caches cold and again with the same project and caches warm. Keep the results separate from the app's render performance.
