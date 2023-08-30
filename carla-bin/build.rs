use std::{env, fs, path::PathBuf};

use serde::Deserialize;

#[derive(Deserialize)]
struct PrebuildConfig {
    #[allow(unused)]
    pub dir_name: PathBuf,
}

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let tag = format!(
        "{}.{}.{}",
        env::var("CARGO_PKG_VERSION").unwrap(),
        env::var("TARGET").unwrap(),
        env::var("PROFILE").unwrap()
    );
    let tag_file = out_dir.join("TAG");
    fs::write(tag_file, &tag).unwrap();

    #[cfg(all(any(unix, windows), feature = "generate-prebuild"))]
    {
        use carla_src::{
            libcarla_client::{build, install},
            Download,
        };

        let PrebuildConfig { dir_name } = json5::from_str(include_str!("prebuild.json5")).unwrap();
        let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
        let install_dir = manifest_dir.join(dir_name).join(&tag);
        let src_dir = Download::default().run().unwrap();
        build(&src_dir).unwrap();
        install(&src_dir, &install_dir).unwrap();
    }
}
