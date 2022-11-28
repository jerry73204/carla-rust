use cxx::UniquePtr;
use crate::ffi;

pub struct Actor {
    pub(crate) inner: UniquePtr<ffi::SharedActor>,
}
