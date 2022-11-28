use crate::{ffi, world::World};
use cxx::{let_cxx_string, UniquePtr};
use std::time::Duration;

pub struct Client {
    inner: UniquePtr<ffi::Client>,
}

pub struct ClientBuilder {
    timeout: Option<Duration>,
}

impl Client {
    pub fn new(host: &str, port: u16) -> Self {
        let_cxx_string!(host_cxx = host);

        Self {
            inner: ffi::client_new(&host_cxx, port),
        }
    }

    pub fn builder() -> ClientBuilder {
        ClientBuilder::new()
    }

    pub fn client_version(&self) -> String {
        ffi::client_get_client_version(&self.inner)
    }

    pub fn server_version(&self) -> String {
        ffi::client_get_server_version(&self.inner)
    }

    pub fn load_world(&self, map_name: &str) -> World {
        let_cxx_string!(map_name = map_name);
        let world = ffi::client_load_world(&self.inner, &map_name);
        World { inner: world }
    }
}

impl ClientBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn build(host: &str, port: u16) -> Client {
        Client::new(host, port)
    }

    pub fn timeout(&mut self, timeout: Duration) {
        todo!();
    }
}

impl Default for ClientBuilder {
    fn default() -> Self {
        Self { timeout: None }
    }
}
