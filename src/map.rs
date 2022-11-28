use crate::ffi;
use cxx::UniquePtr;

pub struct Map {
    pub(crate) inner: UniquePtr<ffi::SharedMap>,
}
