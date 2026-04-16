use std::path::{Path, PathBuf};

use anyhow::{Result, bail, ensure};

#[derive(Debug, Clone)]
pub struct Probe {
    pub prefix: PathBuf,
    pub include_dirs: IncludeDirs,
    pub lib_dirs: LibDirs,
}

#[derive(Debug, Clone)]
pub struct IncludeDirs {
    dirs: Vec<PathBuf>,
}

#[derive(Debug, Clone)]
pub struct LibDirs {
    dirs: Vec<PathBuf>,
}

impl IncludeDirs {
    pub fn into_vec(self) -> Vec<PathBuf> {
        self.dirs
    }
}

impl LibDirs {
    pub fn into_vec(self) -> Vec<PathBuf> {
        self.dirs
    }
}

pub fn probe<P>(carla_src_dir: P) -> Result<Probe>
where
    P: AsRef<Path>,
{
    let carla_version = std::env::var("CARLA_VERSION").unwrap_or_else(|_| "0.10.0".to_string());
    match carla_version.as_str() {
        "0.9.14" | "0.9.15" | "0.9.16" => probe_09x(carla_src_dir, &carla_version),
        "0.10.0" => probe_0_10_0(carla_src_dir),
        _ => bail!(
            "Unsupported CARLA version: {}. Supported versions: 0.9.14, 0.9.15, 0.9.16, 0.10.0",
            carla_version
        ),
    }
}

fn probe_09x<P>(carla_src_dir: P, carla_version: &str) -> Result<Probe>
where
    P: AsRef<Path>,
{
    let find_match = |pattern: &str| -> Result<_> {
        let mut iter = glob::glob(pattern)?;

        let path = match iter.next() {
            Some(Ok(path)) => path,
            Some(Err(err)) => return Err(err.into()),
            None => bail!("Unable to match '{pattern}'"),
        };
        ensure!(
            iter.next().is_none(),
            "'{pattern}' matches more than one file"
        );
        Ok(path)
    };

    let carla_src_dir = carla_src_dir.as_ref();
    let carla_source_dir = carla_src_dir.join("LibCarla").join("source");
    let carla_third_party_dir = carla_source_dir.join("third-party");
    let build_dir = carla_src_dir.join("Build");

    let (boost_pattern, recast_pattern, rpclib_pattern) = match carla_version {
        "0.9.14" => (
            "boost-1.80.0-c*-install",
            "recast-0b13b0-c*-install",
            "rpclib-v2.2.1_c5-c*-libstdcxx-install",
        ),
        "0.9.15" => (
            "boost-1.80.0-c*-install",
            "recast-c*-install",
            "rpclib-v2.2.1_c5-c*-libstdcxx-install",
        ),
        "0.9.16" => (
            "boost-1.84.0-c*-install",
            "recast-c*-install",
            "rpclib-v2.2.1_c5-c*-libstdcxx-install",
        ),
        _ => unreachable!(),
    };

    let recast_dir = find_match(build_dir.join(recast_pattern).to_str().unwrap())?;
    let rpclib_dir = find_match(build_dir.join(rpclib_pattern).to_str().unwrap())?;
    let boost_dir = find_match(build_dir.join(boost_pattern).to_str().unwrap())?;

    let libpng_dir = build_dir.join("libpng-1.6.37-install");
    ensure!(
        libpng_dir.exists(),
        "Unable to find '{}'",
        libpng_dir.display()
    );

    let libcarla_client_lib_dir = build_dir
        .join("libcarla-client-build.release")
        .join("LibCarla")
        .join("cmake")
        .join("client");
    ensure!(
        libcarla_client_lib_dir.exists(),
        "Unable to find '{}'",
        libcarla_client_lib_dir.display()
    );

    let include_dirs = IncludeDirs {
        dirs: vec![
            recast_dir.join("include"),
            rpclib_dir.join("include"),
            libpng_dir.join("include"),
            boost_dir.join("include"),
            carla_source_dir,
            carla_third_party_dir,
        ],
    };
    let lib_dirs = LibDirs {
        dirs: vec![
            recast_dir.join("lib"),
            rpclib_dir.join("lib"),
            libpng_dir.join("lib"),
            boost_dir.join("lib"),
            libcarla_client_lib_dir,
        ],
    };

    Ok(Probe {
        prefix: carla_src_dir.to_path_buf(),
        include_dirs,
        lib_dirs,
    })
}

fn probe_0_10_0<P>(carla_src_dir: P) -> Result<Probe>
where
    P: AsRef<Path>,
{
    let carla_src_dir = carla_src_dir.as_ref();
    let build_dir = carla_src_dir.join("Build/libstdcxx-release");
    let deps_dir = build_dir.join("_deps");

    ensure!(
        build_dir.exists(),
        "Build directory not found: '{}'. Run the 0.10.0 CMake build first.",
        build_dir.display()
    );

    let carla_source_dir = carla_src_dir.join("LibCarla/source");
    let carla_third_party_dir = carla_source_dir.join("third-party");

    // Verify key artifacts exist
    let libcarla_client = build_dir.join("LibCarla/libcarla-client.a");
    ensure!(
        libcarla_client.exists(),
        "libcarla-client.a not found at '{}'",
        libcarla_client.display()
    );

    // Collect include dirs
    let mut include_dirs = vec![carla_source_dir, carla_third_party_dir];

    // Boost: headers are scattered across libs/*/include and libs/*/*/include
    // (e.g. libs/numeric/conversion/include for boost::numeric_conversion)
    for pattern in [
        deps_dir.join("boost-src/libs/*/include"),
        deps_dir.join("boost-src/libs/*/*/include"),
    ] {
        for entry in glob::glob(pattern.to_str().unwrap())? {
            include_dirs.push(entry?);
        }
    }

    // Recast navigation includes
    for component in &["Recast", "Detour", "DetourCrowd"] {
        let dir = deps_dir.join(format!("recastnavigation-src/{component}/Include"));
        ensure!(
            dir.exists(),
            "Recast include dir not found: '{}'",
            dir.display()
        );
        include_dirs.push(dir);
    }

    // rpclib includes
    let rpclib_include = deps_dir.join("rpclib-src/include");
    ensure!(
        rpclib_include.exists(),
        "rpclib include dir not found: '{}'",
        rpclib_include.display()
    );
    include_dirs.push(rpclib_include);

    // libpng includes (png.h at top level of source, pnglibconf.h in build dir)
    let libpng_src = deps_dir.join("libpng-src");
    ensure!(
        libpng_src.join("png.h").exists(),
        "png.h not found in '{}'",
        libpng_src.display()
    );
    include_dirs.push(libpng_src);

    let libpng_build = deps_dir.join("libpng-build");
    if libpng_build.join("pnglibconf.h").exists() {
        include_dirs.push(libpng_build.clone());
    }

    // Collect lib dirs
    let lib_dirs = LibDirs {
        dirs: vec![
            build_dir.join("LibCarla"),
            deps_dir.join("boost-build/libs/filesystem"),
            deps_dir.join("recastnavigation-build/Recast"),
            deps_dir.join("recastnavigation-build/Detour"),
            deps_dir.join("recastnavigation-build/DetourCrowd"),
            deps_dir.join("rpclib-build"),
            libpng_build,
            deps_dir.join("zlib-build"),
        ],
    };

    Ok(Probe {
        prefix: carla_src_dir.to_path_buf(),
        include_dirs: IncludeDirs { dirs: include_dirs },
        lib_dirs,
    })
}
