include!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/generated/bindings.",
    env!("CARLA_VERSION"),
    ".rs"
));

pub use ffi::*;
