use std::path::{Path, PathBuf};

use anyhow::{bail, ensure, Result};

#[derive(Debug, Clone)]
pub struct Probe {
    pub prefix: PathBuf,
    pub include_dirs: IncludeDirs,
    pub lib_dirs: LibDirs,
}

#[derive(Debug, Clone)]
pub struct IncludeDirs {
    pub carla_source: PathBuf,
    pub carla_third_party: PathBuf,
    pub recast: PathBuf,
    pub rpclib: PathBuf,
    pub boost: PathBuf,
    pub libpng: PathBuf,
}

#[derive(Debug, Clone)]
pub struct LibDirs {
    pub recast: PathBuf,
    pub rpclib: PathBuf,
    pub boost: PathBuf,
    pub libpng: PathBuf,
    pub libcarla_client: PathBuf,
}

impl IncludeDirs {
    pub fn into_vec(self) -> Vec<PathBuf> {
        let Self {
            recast,
            rpclib,
            boost,
            libpng,
            carla_source,
            carla_third_party,
        } = self;
        vec![
            recast,
            rpclib,
            libpng,
            boost,
            carla_source,
            carla_third_party,
        ]
    }
}

impl LibDirs {
    pub fn into_vec(self) -> Vec<PathBuf> {
        let Self {
            recast,
            rpclib,
            boost,
            libpng,
            libcarla_client,
        } = self;
        vec![recast, rpclib, libpng, boost, libcarla_client]
    }
}

pub fn probe<P>(carla_src_dir: P) -> Result<Probe>
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

    // Detect CARLA version to use appropriate boost version
    let carla_version = std::env::var("CARLA_VERSION").unwrap_or_else(|_| "0.9.16".to_string());
    let boost_pattern = match carla_version.as_str() {
        "0.9.14" => "boost-1.80.0-c*-install",
        "0.9.16" => "boost-1.84.0-c*-install",
        _ => bail!(
            "Unsupported CARLA version: {}. Supported versions: 0.9.14, 0.9.16",
            carla_version
        ),
    };

    let recast_dir = find_match(build_dir.join("recast-0b13b0-c*-install").to_str().unwrap())?;
    let rpclib_dir = find_match(
        build_dir
            .join("rpclib-v2.2.1_c5-c*-libstdcxx-install")
            .to_str()
            .unwrap(),
    )?;
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
        libpng_dir.exists(),
        "Unable to find '{}'",
        libcarla_client_lib_dir.display()
    );

    let include_dirs = IncludeDirs {
        carla_source: carla_source_dir,
        carla_third_party: carla_third_party_dir,
        recast: recast_dir.join("include"),
        rpclib: rpclib_dir.join("include"),
        boost: boost_dir.join("include"),
        libpng: libpng_dir.join("include"),
    };
    let lib_dirs = LibDirs {
        recast: recast_dir.join("lib"),
        rpclib: rpclib_dir.join("lib"),
        boost: boost_dir.join("lib"),
        libpng: libpng_dir.join("lib"),
        libcarla_client: libcarla_client_lib_dir,
    };

    Ok(Probe {
        prefix: carla_src_dir.to_path_buf(),
        include_dirs,
        lib_dirs,
    })
}
