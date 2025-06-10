use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=../libcarla_c/include");

    // Tell cargo to look for shared libraries in the libcarla_c install directory
    let cargo_manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let manifest_path = PathBuf::from(&cargo_manifest_dir);
    let project_root = manifest_path.parent().unwrap();
    // Try both install and build directories
    let install_lib_path = project_root.join("libcarla_c/install/lib");
    let build_lib_path = project_root.join("libcarla_c/build");

    let libcarla_c_path =
        if install_lib_path.exists() && install_lib_path.join("libcarla_c.so").exists() {
            install_lib_path
        } else if build_lib_path.exists() && build_lib_path.join("libcarla_c.so").exists() {
            build_lib_path
        } else {
            eprintln!(
                "Warning: libcarla_c not found at {:?} or {:?}",
                install_lib_path, build_lib_path
            );
            eprintln!("Run: cd ../libcarla_c && ./build_libcarla_c.sh");
            return;
        };

    println!(
        "cargo:rustc-link-search=native={}",
        libcarla_c_path.display()
    );
    println!("cargo:rustc-link-lib=carla_c");

    // Also tell cargo to rebuild if the library changes
    println!(
        "cargo:rerun-if-changed={}",
        libcarla_c_path.join("libcarla_c.so").display()
    );

    // Set runtime library path for tests
    println!(
        "cargo:rustc-env=LD_LIBRARY_PATH={}",
        libcarla_c_path.display()
    );

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header("wrapper.h")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Add the include directory for our C headers
        .clang_arg(&format!(
            "-I{}",
            project_root.join("libcarla_c/include").display()
        ))
        // Generate bindings for all the C types and functions
        .allowlist_type("carla_.*")
        .allowlist_function("carla_.*")
        .allowlist_var("CARLA_.*")
        // Generate Debug traits for structs
        .derive_debug(true)
        // Generate Clone traits for structs (where possible)
        .derive_copy(true)
        // Generate PartialEq traits for structs
        .derive_eq(true)
        // Generate Hash traits for structs
        .derive_hash(true)
        // Generate Default traits for structs
        .derive_default(true)
        // Use core instead of std for no_std compatibility (optional)
        .use_core()
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
