# Robot Analyzer

A clean-room robotics workstation with one React/TypeScript frontend for the hosted website and two Windows desktop versions: Rust/Tauri and JavaScript/TinyJS. Both desktop versions use the same robotics calculations and project format.
This is a full clone of the app RoboAnalyzer6, which is closed source and unmaintained. The newer versions are paid, which is why i made the clone.

```sh
bun install --frozen-lockfile
bun run dev
bun run build
```

Open the development URL printed by Vite (port 1420). Choose an example robot, edit standard DH parameters, then select DH, FKin, IKin, Dynamics, Trajectory, Graphs or Virtual Robot. Lengths use meters and angles use radians. Projects save as `.robot.json`; numerical data exports as CSV.

The solver supports mixed revolute/prismatic chains, numerical pose IK, Newton–Euler dynamics, RK4 forward simulation and linear/cubic/quintic/cycloidal joint motion. It rejects polynomial paths that exceed joint limits, including interior overshoot from nonzero boundary conditions. Cartesian planning validates the path segment by segment; orientation hold is optional.

The 3D view renders on demand. A CPU projection is available when WebGL cannot initialize. Projection offers fixed camera views and joint selection; orbit, pan, DH animation and target dragging use the WebGL renderer.

## Desktop versions

Both versions require Windows x64 and WebView2. The Tauri installer installs WebView2 when needed; TinyJS portable users need the runtime already installed.

| Version | Develop | Build | Windows output |
| --- | --- | --- | --- |
| Tauri (Rust toolchain required) | `bun run desktop:dev` | `bun run desktop:build` | `src-tauri/target/release/bundle/nsis/Robot Analyzer_0.1.0_x64-setup.exe` |
| TinyJS 0.74.0 | `bun run tinyjs:dev` | `bun run tinyjs:build` | `dist/robot-analyzer.exe` and `dist/launcher.exe` together |

Install [TinyJS](https://tinyjs.app/docs/install) on Windows, then reopen the terminal:

```powershell
irm https://tinyjs.app/install.ps1 | iex
```

TinyJS packaging replaces `dist/` with the portable application. Build Tauri before TinyJS when building both. Published release downloads label each runtime; extract the TinyJS ZIP before running `robot-analyzer.exe` and keep its files together. Builds are unsigned.

Native dialogs open project JSON and save JSON/CSV exports. Version 1 projects and the 1 MiB import limit work across both versions. TinyJS saves replace files only after writing a temporary file; its page can call only the two file operations, and the backend owns file picking.

`bun run check` runs type checking, lint and tests; `bun run typecheck:tsc` independently checks the official TypeScript compiler. The tests cover robotics behavior and desktop file round trips, cancellation, invalid input and failed saves. macOS and Linux desktop builds have not been tested.

See [toolchain and resource policy](docs/TOOLCHAIN.md) and [comparison with RoboAnalyzer 6](docs/COMPARISON.md). This project has no affiliation with the original RoboAnalyzer team.
