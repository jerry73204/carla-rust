use super::World;
use crate::{
    rpc::OpendriveGenerationParameters,
    traffic_manager::{constants::Networking::TM_DEFAULT_PORT, TrafficManager},
    utils::{check_carla_error, rust_string_to_c, c_string_to_rust, ArrayConversionExt},
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{
    ptr,
    time::Duration,
    ffi::CString,
};

/// The client maintains the connection to the CARLA simulator server,
/// corresponding to `carla.Client` in Python API.
#[derive(Debug)]
pub struct Client {
    inner: *mut carla_client_t,
}

impl Client {
    pub fn connect<W>(host: &str, port: u16, worker_threads: W) -> Result<Self>
    where
        W: Into<Option<usize>>,
    {
        let host_cstring = rust_string_to_c(host)?;
        let worker_threads = worker_threads.into().unwrap_or(0);

        let client_ptr = unsafe {
            carla_client_new(host_cstring.as_ptr(), port, worker_threads)
        };

        if client_ptr.is_null() {
            return Err(anyhow!("Failed to create CARLA client"));
        }

        Ok(Self {
            inner: client_ptr,
        })
    }

    pub fn client_version(&self) -> Result<String> {
        let version_ptr = unsafe { carla_client_get_client_version(self.inner) };
        if version_ptr.is_null() {
            return Err(anyhow!("Failed to get client version"));
        }
        let version = unsafe { c_string_to_rust(version_ptr)? };
        unsafe { carla_free_string(version_ptr as *mut _) };
        Ok(version)
    }

    pub fn server_version(&self) -> Result<String> {
        let version_ptr = unsafe { carla_client_get_server_version(self.inner) };
        if version_ptr.is_null() {
            return Err(anyhow!("Failed to get server version"));
        }
        let version = unsafe { c_string_to_rust(version_ptr)? };
        unsafe { carla_free_string(version_ptr as *mut _) };
        Ok(version)
    }

    pub fn timeout(&self) -> Duration {
        let millis = unsafe { carla_client_get_timeout(self.inner) };
        Duration::from_millis(millis)
    }

    pub fn set_timeout(&mut self, timeout: Duration) {
        let millis = timeout.as_millis() as u64;
        unsafe { carla_client_set_timeout(self.inner, millis) };
    }

    pub fn available_maps(&self) -> Result<Vec<String>> {
        let string_list_ptr = unsafe { carla_client_get_available_maps(self.inner) };
        if string_list_ptr.is_null() {
            return Err(anyhow!("Failed to get available maps"));
        }

        let list_size = unsafe { carla_string_list_size(string_list_ptr) };
        let mut maps = Vec::with_capacity(list_size);

        for i in 0..list_size {
            let map_ptr = unsafe { carla_string_list_get(string_list_ptr, i) };
            if !map_ptr.is_null() {
                let map_name = unsafe { c_string_to_rust(map_ptr)? };
                maps.push(map_name);
            }
        }

        unsafe { carla_free_string_list(string_list_ptr as *mut _) };
        Ok(maps)
    }

    pub fn load_world(&self, map_name: &str) -> Result<World> {
        self.load_world_opt(map_name, true)
    }

    pub fn load_world_opt(&self, map_name: &str, reset_settings: bool) -> Result<World> {
        let map_name_cstring = rust_string_to_c(map_name)?;
        let world_ptr = unsafe {
            carla_client_load_world(
                self.inner,
                map_name_cstring.as_ptr(),
                reset_settings,
                carla_map_layer_t_CARLA_MAP_LAYER_ALL,
            )
        };

        if world_ptr.is_null() {
            return Err(anyhow!("Failed to load world: {}", map_name));
        }

        World::from_raw_ptr(world_ptr)
    }

    pub fn reload_world(&self) -> Result<World> {
        self.reload_world_opt(true)
    }

    pub fn reload_world_opt(&self, reset_settings: bool) -> Result<World> {
        let world_ptr = unsafe {
            carla_client_reload_world(self.inner, reset_settings)
        };

        if world_ptr.is_null() {
            return Err(anyhow!("Failed to reload world"));
        }

        World::from_raw_ptr(world_ptr)
    }

    pub fn generate_open_drive_world(
        &self,
        opendrive: &str,
        params: &OpendriveGenerationParameters,
        reset_settings: bool,
    ) -> Result<World> {
        // Note: This method will need to be implemented once OpenDRIVE generation
        // is available in the C wrapper. For now, return an error.
        Err(anyhow!("OpenDRIVE world generation not yet implemented in C wrapper"))
    }

    pub fn world(&self) -> Result<World> {
        let world_ptr = unsafe { carla_client_get_world(self.inner) };
        if world_ptr.is_null() {
            return Err(anyhow!("Failed to get world"));
        }
        World::from_raw_ptr(world_ptr)
    }

    pub fn instance_tm<P>(&self, port: P) -> Result<TrafficManager>
    where
        P: Into<Option<u16>>,
    {
        let port = port.into().unwrap_or(TM_DEFAULT_PORT);
        let tm_port = unsafe { carla_client_get_traffic_manager_port(self.inner, port) };
        TrafficManager::new(tm_port)
    }
}

impl Default for Client {
    fn default() -> Self {
        Self::connect("localhost", 2000, None)
            .expect("Failed to create default CARLA client")
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_client_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: Client wraps a thread-safe C API
unsafe impl Send for Client {}
unsafe impl Sync for Client {}

