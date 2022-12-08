use std::{
    env, fs,
    path::{Path, PathBuf},
};

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is not set"));
    let manifest_dir =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set"));

    {
        let path = out_dir.join("OUT_DIR");
        fs::write(&path, out_dir.to_str().unwrap()).unwrap();
    }

    {
        let path = out_dir.join("CARGO_MANIFEST_DIR");
        fs::write(&path, manifest_dir.to_str().unwrap()).unwrap();
    }

    {
        let tag = format!(
            "{}.{}",
            env::var("TARGET").unwrap(),
            env::var("PROFILE").unwrap()
        );

        let tag_file = out_dir.join("TAG");
        fs::write(&tag_file, &tag).unwrap();

        #[cfg(all(any(unix, windows), feature = "generate-cache"))]
        {
            let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
            let cache_dir = manifest_dir.join("cache").join(&tag);
            generate_cache(&out_dir, &cache_dir);
        }
    }
}

#[cfg(all(any(unix, windows), feature = "generate-cache"))]
fn generate_cache(out_dir: &Path, cache_dir: &Path) {
    use carla_src::{build_libcarla_client_library, probe, Download, LibDirs};

    let src_dir = Download {
        out_dir: Some(out_dir.to_path_buf()),
        ..Default::default()
    }
    .run()
    .unwrap();
    build_libcarla_client_library(&src_dir).unwrap();

    fs::create_dir_all(cache_dir).unwrap();

    let LibDirs {
        recast: recast_lib_dir,
        rpclib: rpclib_lib_dir,
        boost: boost_lib_dir,
        libpng: libpng_lib_dir,
        libcarla_client: libcarla_client_lib_dir,
    } = probe(&src_dir).lib_dirs;

    let copy_lib_files = |src_dir: &Path, tgt_dir: &Path, libs: &[&str]| {
        for lib in libs {
            let name = lib_filename(lib);
            let src_file = src_dir.join(&name);
            let tgt_file = tgt_dir.join(&name);
            fs::copy(src_file, tgt_file).unwrap();
        }
    };

    copy_lib_files(
        &recast_lib_dir,
        cache_dir,
        &["Recast", "Detour", "DetourCrowd"],
    );
    copy_lib_files(&rpclib_lib_dir, cache_dir, &["rpc"]);
    copy_lib_files(&libpng_lib_dir, cache_dir, &["png"]);
    copy_lib_files(&boost_lib_dir, cache_dir, &["boost_filesystem"]);
    copy_lib_files(&libcarla_client_lib_dir, cache_dir, &["carla_client"]);
}

fn lib_filename(lib: &str) -> String {
    use cfg_if::cfg_if;
    {
        cfg_if! {
            if #[cfg(unix)] {
                format!("lib{}.a", lib)
            } else if #[cfg(windows)] {
                format!("{}.lib", lib)
            } else {
                compile_error!("The platform is not supported");
            }
        }
    }
}
