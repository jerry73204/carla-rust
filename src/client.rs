use crate::{ffi, world::World};
use cxx::{let_cxx_string, UniquePtr};
use std::time::Duration;

pub struct Client {
    inner: UniquePtr<ffi::Client>,
}

impl Client {
    pub fn connect(host: &str, port: u16) -> Self {
        let_cxx_string!(host_cxx = host);

        Self {
            inner: ffi::client_new(&host_cxx, port),
        }
    }

    pub fn client_version(&self) -> String {
        ffi::client_get_client_version(&self.inner)
    }

    pub fn server_version(&self) -> String {
        ffi::client_get_server_version(&self.inner)
    }

    pub fn timeout(&mut self) -> Duration {
        let millis = ffi::client_get_timeout_millis(self.inner.pin_mut());
        Duration::from_millis(millis as u64)
    }

    pub fn set_timeout(&mut self, timeout: Duration) {
        ffi::client_set_timeout_millis(self.inner.pin_mut(), timeout.as_millis() as usize)
    }

    pub fn load_world(&self, map_name: &str) -> World {
        let_cxx_string!(map_name = map_name);
        let world = ffi::client_load_world(&self.inner, &map_name);
        World { inner: world }
    }
}

impl Default for Client {
    fn default() -> Self {
        Self::connect("localhost", 2000)
    }
}
