use std::path::{Path, PathBuf};

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

pub fn probe(carla_src_dir: &Path) -> Probe {
    let carla_source_dir = carla_src_dir.join("LibCarla").join("source");
    let carla_third_party_dir = carla_source_dir.join("third-party");
    let build_dir = carla_src_dir.join("Build");
    let recast_dir = build_dir.join("recast-0b13b0-c10-install");
    let rpclib_dir = build_dir.join("rpclib-v2.2.1_c5-c10-libstdcxx-install");
    let boost_dir = build_dir.join("boost-1.80.0-c10-install");
    let libpng_dir = build_dir.join("libpng-1.6.37-install");
    let libcarla_client_lib_dir = build_dir
        .join("libcarla-client-build.release")
        .join("LibCarla")
        .join("cmake")
        .join("client");

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

    Probe {
        prefix: carla_src_dir.to_path_buf(),
        include_dirs,
        lib_dirs,
    }
}
