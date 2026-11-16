# Robot Analyzer — TinyJS experiment

This separate checkout runs the existing React workspace, Three.js viewport and robotics calculations in TinyJS. Native dialogs open project JSON and save JSON/CSV exports. Project format version 1 and its 1 MiB import limit are unchanged; saves use a temporary file before replacing the destination.

On Windows, install [TinyJS](https://tinyjs.app/docs/install) and reopen your terminal:

```powershell
irm https://tinyjs.app/install.ps1 | iex
```

With Bun installed, run these commands from this checkout:

```powershell
bun install --frozen-lockfile
bun run tinyjs:dev
```

Build the portable app:

```powershell
bun run tinyjs:build
.\dist\robot-analyzer.exe
```

Distribute both `dist/robot-analyzer.exe` and `dist/launcher.exe` together. Windows requires WebView2. The build uses relative asset paths so the packaged UI works without a development server. TinyJS owns `dist/` after packaging; use `bun run dev` for frontend development.

`bun run check` runs the type checker, lint and Bun tests. `bun run typecheck:tsc` also checks compatibility with the official TypeScript compiler. Desktop file tests cover format compatibility, CSV exports, cancellation, invalid/oversized input, and preservation of existing files when replacement fails.

The original Tauri files and `desktop:dev` / `desktop:build` commands remain available for comparison. The TinyJS configuration uses a separate application ID and exposes only the two file operations to the page. Native pickers run in the backend, so the page cannot request arbitrary filesystem paths.

Validated with TinyJS 0.74.0 on Windows x64. macOS and Linux have not been tested. See [docs/TOOLCHAIN.md](docs/TOOLCHAIN.md) for the existing build resource limits.
