use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=MUJOCO_LIB_DIR");
    let include = PathBuf::from(
        env::var("MUJOCO_INCLUDE_DIR")
            .expect("set MUJOCO_INCLUDE_DIR to the pinned MuJoCo include directory"),
    );
    let directory = PathBuf::from(
        env::var("MUJOCO_LIB_DIR")
            .expect("set MUJOCO_LIB_DIR to the matching shared-library directory"),
    );
    let header = include.join("mujoco/mujoco.h");
    assert!(
        header.is_file(),
        "MuJoCo header missing at {}",
        header.display()
    );
    let library = fs::read_dir(&directory)
        .unwrap()
        .map(|entry| entry.expect("read MuJoCo library directory entry").path())
        .find(|p| {
            let name = p.file_name().unwrap().to_string_lossy();
            name.starts_with("libmujoco.") && (name.ends_with(".dylib") || name.contains(".so"))
        })
        .expect("MuJoCo shared library missing in MUJOCO_LIB_DIR");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let extension = if env::var("CARGO_CFG_TARGET_OS").unwrap() == "macos" {
        "dylib"
    } else {
        "so"
    };
    let link = out.join(format!("libmujoco.{extension}"));
    if link.symlink_metadata().is_ok() {
        fs::remove_file(&link).unwrap();
    }
    std::os::unix::fs::symlink(&library, &link).unwrap();
    println!("cargo:rustc-link-search=native={}", out.display());
    println!("cargo:rustc-link-lib=dylib=mujoco");
    println!("cargo:rerun-if-changed={}", header.display());
    bindgen::Builder::default()
        .header(header.to_string_lossy())
        .clang_arg(format!("-I{}", include.display()))
        .allowlist_type("mj.*")
        .allowlist_function("mj.*")
        .allowlist_var("mj.*")
        .derive_debug(false)
        .layout_tests(false)
        .generate()
        .expect("generate version-matched MuJoCo bindings")
        .write_to_file(out.join("bindings.rs"))
        .unwrap();
}
