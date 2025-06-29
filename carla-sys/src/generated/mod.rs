//! Generated types from CARLA Python API
//!
//! This module conditionally exports either FFI-based implementations
//! or stub implementations based on the docs-only feature flag.

#[cfg(not(feature = "docs-only"))]
pub use self::ffi::*;

#[cfg(feature = "docs-only")]
pub use self::stubs::*;

// Include pre-generated modules
#[cfg(not(feature = "docs-only"))]
mod ffi {
    include!("ffi/mod.rs");
}

#[cfg(feature = "docs-only")]
mod stubs {
    include!("stubs/mod.rs");
}
