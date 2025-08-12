//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    let num_configs = std::env::var("NUM_CONFIGS").unwrap_or_else(|_| "3".to_string()); // Default value
    println!("cargo:rerun-if-env-changed=NUM_CONFIGS");
    let num_keys = std::env::var("NUM_KEYS").unwrap_or_else(|_| "42".to_string()); // Default value
    println!("cargo:rerun-if-env-changed=NUM_KEYS");
    let num_layers = std::env::var("NUM_LAYERS").unwrap_or_else(|_| "6".to_string()); // Default value
    println!("cargo:rerun-if-env-changed=NUM_LAYERS");
    let contents = format!(
        r#"
    pub const NUM_CONFIGS: usize = {};
    pub const NUM_KEYS: usize = {};
    pub const NUM_LAYERS: usize = {};
    "#,
        num_configs, num_keys, num_layers,
    );
    std::fs::write("src/config.rs", contents).expect("Failed to write config.rs");

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
