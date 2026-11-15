//! Run cbindgen at build time to emit `include/x3f.h` next to the
//! compiled artefacts. Downstream consumers (cargo-xcframework, NDK
//! jniLibs builds, plain `cc -lx3f`) read the header from this path.
//!
//! The generated header is committed-out-of-tree (lives in OUT_DIR
//! under `target/`) — that keeps it from drifting from the Rust
//! source, and lets us treat regeneration as a normal build step.

use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-env-changed=X3F_SKIP_CBINDGEN");

    // cbindgen shells out to `cargo metadata`, which resolves the *whole*
    // dependency graph — including the Windows-only `windows` crate. In an
    // offline / sandboxed build that cannot be fetched, so the header step
    // fails even though the library itself builds fine. Set
    // X3F_SKIP_CBINDGEN=1 to skip it; the header that matters to this
    // checkout is the one committed under
    // `Sources/CX3FRaw/include/` (refreshed by build_x3fuse.sh).
    if env::var("X3F_SKIP_CBINDGEN").is_ok() {
        println!("cargo:warning=skipping cbindgen (X3F_SKIP_CBINDGEN set); using committed header");
        return;
    }

    // We emit into both OUT_DIR (for unit-test consumers via env! ) and
    // <profile>/include/x3f.h next to the artefacts (for downstream tools).
    let header_out = out_dir.join("x3f.h");
    let mut config = cbindgen::Config::from_file(PathBuf::from(&crate_dir).join("cbindgen.toml"))
        .expect("cbindgen.toml must exist next to Cargo.toml");
    // cbindgen 0.27 defaults `language="C"` from the file; nothing extra to
    // override here.
    config.language = cbindgen::Language::C;

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("cbindgen failed to generate x3f.h")
        .write_to_file(&header_out);

    // Also publish to <profile>/include/x3f.h so downstream tooling that
    // doesn't know about OUT_DIR can find it. profile_dir = OUT_DIR /../../..
    // (build/x3f-ffi-c-XYZ/out → target/<profile>).
    let profile_dir = out_dir
        .ancestors()
        .nth(3)
        .expect("OUT_DIR depth")
        .to_path_buf();
    let pub_include = profile_dir.join("include");
    std::fs::create_dir_all(&pub_include).ok();
    std::fs::copy(&header_out, pub_include.join("x3f.h")).ok();
}
