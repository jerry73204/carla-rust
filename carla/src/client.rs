use crate::rpc::OpendriveGenerationParameters;
use crate::World;
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

    pub fn load_world(&self, map_name: &str) -> World {
        self.load_world_opt(map_name, true)
    }

    pub fn load_world_opt(&self, map_name: &str, reset_settings: bool) -> World {
        let world = self
            .inner
            .LoadWorld(map_name, reset_settings)
            .within_unique_ptr();
        World::from_cxx(world)
    }

    pub fn reload_world(&self) -> World {
        self.reload_world_opt(true)
    }

    pub fn reload_world_opt(&self, reset_settings: bool) -> World {
        let world = self.inner.ReloadWorld(reset_settings).within_unique_ptr();
        World::from_cxx(world)
    }

    pub fn generate_open_drive_world(
        &self,
        opendrive: &str,
        params: &OpendriveGenerationParameters,
        reset_settings: bool,
    ) -> World {
        let world = self
            .inner
            .GenerateOpenDriveWorld(opendrive, params, reset_settings)
            .within_unique_ptr();
        World::from_cxx(world)
    }

    pub fn world(&self) -> World {
        let world = self.inner.GetWorld().within_unique_ptr();
        World::from_cxx(world)
    }
}

impl Default for Client {
    fn default() -> Self {
        Self::connect("localhost", 2000, None)
    }
}
