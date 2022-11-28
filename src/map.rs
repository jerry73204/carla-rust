use cxx::UniquePtr;
use crate::ffi;

pub struct Map {
    pub(crate) inner: UniquePtr<ffi::SharedMap>,
}
