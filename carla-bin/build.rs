use std::{env, fs, path::PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is not set"));
    let output_file = out_dir.join("OUT_DIR");
    fs::write(&output_file, out_dir.to_str().unwrap()).unwrap();
}
