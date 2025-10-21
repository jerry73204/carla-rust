// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::World;
use crate::{
    rpc::OpendriveGenerationParameters,
    traffic_manager::{constants::Networking::TM_DEFAULT_PORT, TrafficManager},
};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::FfiClient;
use cxx::{let_cxx_string, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::time::Duration;

/// The client maintains the connection to the CARLA simulator server,
/// corresponding to `carla.Client` in Python API.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Client {
    #[derivative(Debug = "ignore")]
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
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    pub fn reload_world(&self) -> World {
        self.reload_world_opt(true)
    }

    pub fn reload_world_opt(&self, reset_settings: bool) -> World {
        let world = self.inner.ReloadWorld(reset_settings).within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
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
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    pub fn world(&self) -> World {
        let world = self.inner.GetWorld().within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    pub fn instance_tm<P>(&self, port: P) -> TrafficManager
    where
        P: Into<Option<u16>>,
    {
        let port = port.into().unwrap_or(TM_DEFAULT_PORT);
        let ptr = self.inner.GetInstanceTM(port).within_unique_ptr();
        unsafe { TrafficManager::from_cxx(ptr).unwrap_unchecked() }
    }
}

impl Default for Client {
    fn default() -> Self {
        Self::connect("localhost", 2000, None)
    }
}

assert_impl_all!(Client: Sync, Send);
