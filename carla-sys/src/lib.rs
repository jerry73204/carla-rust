//! Low-level FFI bindings to the CARLA C++ client library.
//!
//! ⚠️ **Warning:** This crate provides unsafe, low-level FFI bindings.
//! Most users should use the [`carla`](https://docs.rs/carla) crate instead,
//! which provides safe, idiomatic Rust wrappers around these bindings.
//!
//! # Overview
//!
//! `carla-sys` exposes the C++ CARLA client library to Rust using
//! [autocxx](https://docs.rs/autocxx) for automatic binding generation.
//! This crate handles:
//!
//! - Building and linking the CARLA C++ client library
//! - Generating Rust FFI bindings to C++ classes and functions
//! - Providing minimal safe wrappers (e.g., `Send` and `Sync` implementations)
//!
//! # Safety
//!
//! **This crate is not intended for direct use.** The FFI bindings are largely
//! `unsafe` and require careful handling of:
//!
//! - Raw pointers and null pointer checks
//! - C++ object lifetimes and ownership
//! - Memory management across the FFI boundary
//! - Thread safety of C++ objects
//!
//! The [`carla`](https://docs.rs/carla) crate provides safe abstractions over
//! these unsafe operations.
//!
//! # Building
//!
//! This crate requires either:
//! 1. **Prebuilt binaries** - Downloaded automatically for supported platforms
//! 2. **Building from source** - Requires CARLA source and C++ build tools
//!
//! ## Environment Variables
//!
//! - `CARLA_DIR` - Path to CARLA installation or source directory
//! - `TARGET` - Rust target triple (set automatically by cargo)
//! - `OUT_DIR` - Build output directory (set automatically by cargo)
//!
//! ## Features
//!
//! - `build-prebuilt` - Build CARLA C++ library from source and save as prebuilt tarball with bindings for distribution
//! - `docs-only` - Generate documentation without C++ library (for docs.rs)
//!
//! # C++ Version Compatibility
//!
//! This crate is compatible with **CARLA 0.9.14**. Different CARLA versions
//! may have incompatible C++ APIs.
//!
//! # Thread Safety
//!
//! Most C++ types are wrapped with `unsafe impl Send` and `unsafe impl Sync`
//! in the `impls` module. See `carla-sys/src/impls.rs` for safety documentation
//! on each implementation.
//!
//! # Examples
//!
//! **Direct use of this crate is not recommended.** Use the [`carla`] crate instead.
//!
//! If you must use `carla-sys` directly (e.g., for custom bindings):
//!
//! ```ignore
//! use carla_sys::carla_rust::client::FfiClient;
//!
//! unsafe {
//!     // Create a client - requires manual memory management
//!     let client = FfiClient::new("localhost", 2000, 0);
//!     // ... use client (careful with lifetimes and ownership!)
//! }
//! ```
//!
//! # Links
//!
//! - [CARLA Simulator](https://carla.org/)
//! - [CARLA C++ Reference](https://carla.readthedocs.io/en/0.9.14/ref_cpp/)
//! - [`carla` crate](https://docs.rs/carla) - Safe Rust bindings
//! - [autocxx](https://docs.rs/autocxx) - C++ binding generator
//!
//! # See Also
//!
//! - [`carla`] - Safe, high-level Rust bindings (use this instead!)

#[cfg(feature = "docs-only")]
mod ffi_docs_only;
#[cfg(feature = "docs-only")]
pub use ffi_docs_only::*;

#[cfg(not(feature = "docs-only"))]
mod bindings;
#[cfg(not(feature = "docs-only"))]
pub use bindings::*;

mod impls;
