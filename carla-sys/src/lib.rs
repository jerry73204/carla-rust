#[cfg(feature = "docs-only")]
mod ffi_docs_only;
#[cfg(feature = "docs-only")]
pub use ffi_docs_only::*;

#[cfg(not(feature = "docs-only"))]
mod bindings;
#[cfg(not(feature = "docs-only"))]
pub use bindings::*;

mod impls;
