# Robot Analyzer

A clean-room robotics workstation with a shared React/TypeScript frontend for the hosted website and Rust/Tauri Windows desktop app.

```sh
bun install --frozen-lockfile
bun run dev
bun run build
bun run desktop:build
```

Open the development URL printed by Vite (port 1420). Choose an example robot, edit standard DH parameters, then select DH, FKin, IKin, Dynamics, Trajectory, Graphs or Virtual Robot. Lengths use meters and angles use radians. Projects save as `.robot.json`; numerical data exports as CSV.

The solver supports mixed revolute/prismatic chains, numerical pose IK, Newton–Euler dynamics, RK4 forward simulation and linear/cubic/quintic/cycloidal joint motion. It rejects polynomial paths that exceed joint limits, including interior overshoot from nonzero boundary conditions. Cartesian planning validates the path segment by segment; orientation hold is optional.

The 3D view renders on demand. A CPU projection is available when WebGL cannot initialize. Projection offers fixed camera views and joint selection; orbit, pan, DH animation and target dragging use the WebGL renderer.

See [toolchain and resource policy](docs/TOOLCHAIN.md) and [comparison with RoboAnalyzer 6](docs/COMPARISON.md). This project has no affiliation with the original RoboAnalyzer team.
