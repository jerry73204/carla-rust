use crate::ffi;
use cxx::{let_cxx_string, UniquePtr};

pub struct BlueprintLibrary {
    pub(crate) inner: UniquePtr<ffi::SharedBlueprintLibrary>,
}

impl BlueprintLibrary {
    pub fn filter(&self, pattern: &str) -> Self {
        let_cxx_string!(pattern = pattern);

        Self {
            inner: ffi::bp_filter(&self.inner, &pattern),
        }
    }
}
