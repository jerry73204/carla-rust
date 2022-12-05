#[cfg(not(feature = "bindgen"))]
mod ffi_docs_only;
#[cfg(not(feature = "bindgen"))]
pub use ffi_docs_only::*;

#[cfg(feature = "bindgen")]
mod ffi;
#[cfg(feature = "bindgen")]
pub use ffi::*;
