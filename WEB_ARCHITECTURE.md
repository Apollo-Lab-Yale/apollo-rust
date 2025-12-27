# Web Preprocessor Architecture

## Goal
The goal is to enable `apollo-rust-preprocessor` and related crates to compile to `wasm32-unknown-unknown` and run entirely within a web browser.
**Strategic Pivot:** We will use a "Headless Rust + Three.js" architecture. `apollo-rust` will provide the math/physics engine via WASM, and Three.js/TypeScript will handle the rendering and UI. We are *not* porting the Bevy UI to WASM.

## Current Architecture vs. Web Architecture

### 1. Execution Model
*   **Current (Native)**: The preprocessor operates as a "Multi-Process" system. The main process spawns child processes (using `std::process::Command`) to run specific heavy tasks, such as `link_shapes_skips_module_process` or `first_look_vis_module_process`. This relies on the OS scheduler and process isolation.
*   **Target (Web/WASM)**: The browser does not support spawning arbitrary OS processes. The architecture must shift to a **"Library Function"** model.
    *   **Refactoring**: Logic currently inside `src/bin/*.rs` `main()` functions must be moved into public library functions (e.g., inside `src/lib.rs`).
    *   **Concurrency**: Heavy tasks should be offloaded to **Web Workers** instead of child processes. The main thread will communicate with workers via message passing (e.g., `postMessage`), simulating the IPC of the native model.

### 2. Filesystem Access
*   **Current (Native)**: Uses `std::fs` and `std::path::PathBuf` to read/write directly to the user's disk.
*   **Target (Web/WASM)**: Direct disk access is restricted/sandboxed.
    *   **Abstraction**: We have introduced `ApolloPathBufTrait` to abstract path manipulation and file I/O.
    *   **Implementation**: A "Virtual Filesystem" struct `WebPathBuf` has been implemented in `apollo-rust-wasm`. It uses a global static `RwLock<HashMap<String, Vec<u8>>>` to store file contents in memory. JavaScript can inject files via `inject_file` (exposed via `wasm-bindgen`), allowing the engine to "read" URDFs and meshes as if they were on disk.

### 3. Rendering Architecture (Strategic Pivot)
*   **Current (Native)**: Uses `bevy` and `bevy_egui` for window management, 3D rendering, and UI.
*   **Target (Web)**:
    *   **Engine**: `apollo-rust` runs "headless" in WASM. It calculates kinematics, proximity, and skip matrices.
    *   **Renderer**: `Three.js` (JavaScript) handles the scene graph.
    *   **Interface**: A TypeScript frontend (`crates/apollo-rust-app`) queries the WASM engine for state (e.g., "Get joint positions", "Get skipped pairs") and updates the Three.js scene accordingly.
    *   **Bridge**: `wasm-bridge.ts` manages the WASM module lifecycle.

## Dependencies Strategy (Resolved)
To achieve WASM compilation, we implemented a strict dependency harmonization strategy:
*   **Core Math**: Pinned to `nalgebra` 0.33 and `parry3d-f64` 0.17 to ensure compatibility.
*   **Randomness**: Forced `getrandom` 0.2 (via `ahash` 0.8.11 and `uuid` 1.11.0) because `getrandom` 0.3 is not yet fully WASM-compatible without complex feature flags.
*   **Native Exclusion**: Used `#[cfg(not(target_arch = "wasm32"))]` to exclude native-only libraries:
    *   `nalgebra-lapack` & `ndarray-linalg` (OpenBLAS is native-only).
    *   `arboard` (Clipboard access).
    *   `pbr` (Console progress bars).
*   **Clipboard**: `bevy_egui` was upgraded to 0.30 and `manage_clipboard` was carefully managed to avoid `web-sys` instability.
