use autocxx::prelude::*;
use carla_sys::carla_rust::client::FfiClient;
use cxx::{let_cxx_string, UniquePtr};
use std::time::Duration;

pub struct Client {
    inner: UniquePtr<FfiClient>,
}

impl Client {
    pub fn connect<W>(host: &str, port: u16, worker_threads: W) -> Self
    where
        W: Into<Option<usize>>,
    {
        let_cxx_string!(host_cxx = host);
        let worker_threads = worker_threads.into().unwrap_or(0);

        Self {
            inner: FfiClient::new(&host_cxx, port, worker_threads).within_unique_ptr(),
        }
    }

    pub fn client_version(&self) -> String {
        self.inner.GetClientVersion().to_string()
    }

    pub fn server_version(&self) -> String {
        self.inner.GetServerVersion().to_string()
    }

    pub fn timeout(&mut self) -> Duration {
        let millis = self.inner.pin_mut().GetTimeout();
        Duration::from_millis(millis as u64)
    }

    pub fn set_timeout(&mut self, timeout: Duration) {
        let millis = timeout.as_millis() as usize;
        self.inner.pin_mut().SetTimeout(millis);
    }

    pub fn avaiable_maps(&self) -> Vec<String> {
        self.inner
            .GetAvailableMaps()
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    // pub fn load_world(&self, map_name: &str) -> World {
    //     let_cxx_string!(map_name = map_name);
    //     let world = ffi::client_load_world(&self.inner, &map_name);
    //     World { inner: world }
    // }
}

impl Default for Client {
    fn default() -> Self {
        Self::connect("localhost", 2000, None)
    }
}
