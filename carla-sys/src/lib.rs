use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "docs-only")] {
        mod ffi_docs_only;
        pub use ffi_docs_only;
    } else {
        mod ffi;
        pub use ffi::*;
    }
}
