use crate::ffi;
use cxx::UniquePtr;

pub struct Actor {
    pub(crate) inner: UniquePtr<ffi::SharedActor>,
}
