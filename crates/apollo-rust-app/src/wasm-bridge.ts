import init, { type InitOutput, WASMKinematics, WASMProximity, inject_file } from '../../apollo-rust-wasm/pkg';

let wasm: InitOutput | null = null;

export async function initWasm() {
    if (wasm) return wasm;
    wasm = await init();
    return wasm;
}

export { WASMKinematics, WASMProximity };

/**
 * Injects a file into the Rust side virtual filesystem.
 * This is needed because WASM doesn't have direct access to the local disk.
 * The Rust side implements a virtual filesystem via a global HashMap.
 */
export function injectFile(path: string, content: Uint8Array) {
    inject_file(path, Array.from(content));
}
