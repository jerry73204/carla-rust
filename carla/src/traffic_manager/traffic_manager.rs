use crate::{
    client::{ActorBase, Client},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Point3;
use std::{ptr, time::Duration};

/// Handle groups of autopilot vehicles with realistic urban traffic
/// conditions, corresponding to `carla.TrafficManager` in Python API.
#[derive(Clone, Debug)]
pub struct TrafficManager {
    inner: *mut carla_traffic_manager_t,
    port: u16,
}

impl TrafficManager {
    /// Create a new TrafficManager instance from a client.
    pub(crate) fn new(client: &Client, port: u16) -> Result<Self> {
        let tm_ptr = unsafe { carla_traffic_manager_get_instance(client.raw_ptr(), port) };
        if tm_ptr.is_null() {
            return Err(anyhow!("Failed to create traffic manager on port {}", port));
        }
        Ok(Self {
            inner: tm_ptr,
            port,
        })
    }

    /// Get the default TrafficManager instance (port 8000).
    pub(crate) fn default(client: &Client) -> Result<Self> {
        let tm_ptr = unsafe { carla_traffic_manager_get_default(client.raw_ptr()) };
        if tm_ptr.is_null() {
            return Err(anyhow!("Failed to get default traffic manager"));
        }
        Ok(Self {
            inner: tm_ptr,
            port: CARLA_TM_DEFAULT_PORT as u16,
        })
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn get_info(&self) -> Result<(u16, bool, usize)> {
        let mut info = carla_traffic_manager_info_t {
            port: 0,
            is_running: false,
            registered_vehicle_count: 0,
            config: unsafe { std::mem::zeroed() },
        };
        let error = unsafe { carla_traffic_manager_get_info(self.inner, &mut info) };
        check_carla_error(error)?;
        Ok((info.port, info.is_running, info.registered_vehicle_count))
    }

    pub fn set_osm_mode(&mut self, yes: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_osm_mode(self.inner, yes) };
        check_carla_error(error)
    }

    pub fn set_custom_path<A, P>(&mut self, actor: &A, path: &[P], empty_buffer: bool) -> Result<()>
    where
        A: ActorBase,
        P: AsRef<Point3<f32>>,
    {
        // Convert path to C array
        let c_locations: Vec<carla_vector3d_t> = path
            .iter()
            .map(|point| {
                let p = point.as_ref();
                carla_vector3d_t {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                }
            })
            .collect();

        let mut c_path = carla_traffic_manager_path_t {
            locations: ptr::null_mut(),
            location_count: 0,
            empty_buffer,
        };

        let error = unsafe {
            carla_traffic_manager_create_path(
                c_locations.as_ptr(),
                c_locations.len(),
                empty_buffer,
                &mut c_path,
            )
        };
        check_carla_error(error)?;

        let result = unsafe {
            carla_traffic_manager_set_vehicle_custom_path(self.inner, actor.raw_ptr(), &c_path)
        };

        unsafe {
            carla_traffic_manager_path_destroy(&mut c_path);
        }

        check_carla_error(result)
    }

    pub fn remove_custom_path<A: ActorBase>(&mut self, actor: &A) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_remove_vehicle_custom_path(self.inner, actor.raw_ptr())
        };
        check_carla_error(error)
    }

    pub fn update_custom_path<A, P>(&mut self, actor: &A, path: &[P]) -> Result<()>
    where
        A: ActorBase,
        P: AsRef<Point3<f32>>,
    {
        // Convert path to C array
        let c_locations: Vec<carla_vector3d_t> = path
            .iter()
            .map(|point| {
                let p = point.as_ref();
                carla_vector3d_t {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                }
            })
            .collect();

        let mut c_path = carla_traffic_manager_path_t {
            locations: ptr::null_mut(),
            location_count: 0,
            empty_buffer: false,
        };

        let error = unsafe {
            carla_traffic_manager_create_path(
                c_locations.as_ptr(),
                c_locations.len(),
                false,
                &mut c_path,
            )
        };
        check_carla_error(error)?;

        let result = unsafe {
            carla_traffic_manager_update_vehicle_custom_path(self.inner, actor.raw_ptr(), &c_path)
        };

        unsafe {
            carla_traffic_manager_path_destroy(&mut c_path);
        }

        check_carla_error(result)
    }

    pub fn set_imported_route<A: ActorBase>(
        &mut self,
        actor: &A,
        road_options: &[carla_road_option_t],
        empty_buffer: bool,
    ) -> Result<()> {
        let mut c_route = carla_traffic_manager_route_t {
            road_options: ptr::null_mut(),
            option_count: 0,
            empty_buffer,
        };

        let error = unsafe {
            carla_traffic_manager_create_route(
                road_options.as_ptr(),
                road_options.len(),
                empty_buffer,
                &mut c_route,
            )
        };
        check_carla_error(error)?;

        let result = unsafe {
            carla_traffic_manager_set_vehicle_imported_route(self.inner, actor.raw_ptr(), &c_route)
        };

        unsafe {
            carla_traffic_manager_route_destroy(&mut c_route);
        }

        check_carla_error(result)
    }

    pub fn remove_imported_route<A: ActorBase>(&mut self, actor: &A) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_remove_vehicle_imported_route(self.inner, actor.raw_ptr())
        };
        check_carla_error(error)
    }

    pub fn update_imported_route<A: ActorBase>(
        &mut self,
        actor: &A,
        road_options: &[carla_road_option_t],
    ) -> Result<()> {
        let mut c_route = carla_traffic_manager_route_t {
            road_options: ptr::null_mut(),
            option_count: 0,
            empty_buffer: false,
        };

        let error = unsafe {
            carla_traffic_manager_create_route(
                road_options.as_ptr(),
                road_options.len(),
                false,
                &mut c_route,
            )
        };
        check_carla_error(error)?;

        let result = unsafe {
            carla_traffic_manager_update_vehicle_imported_route(
                self.inner,
                actor.raw_ptr(),
                &c_route,
            )
        };

        unsafe {
            carla_traffic_manager_route_destroy(&mut c_route);
        }

        check_carla_error(result)
    }

    pub fn set_respawn_dormant_vehicles(&mut self, yes: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_respawn_dormant_vehicles(self.inner, yes) };
        check_carla_error(error)
    }

    pub fn set_boundaries_respawn_dormant_vehicles(
        &mut self,
        lower_bound: f32,
        upper_bound: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_respawn_boundaries(self.inner, lower_bound, upper_bound)
        };
        check_carla_error(error)
    }

    pub fn set_max_boundaries(&mut self, lower: f32, upper: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_max_boundaries(self.inner, lower, upper) };
        check_carla_error(error)
    }

    pub fn set_hybrid_physics_mode(&mut self, yes: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_hybrid_physics_mode(self.inner, yes) };
        check_carla_error(error)
    }

    pub fn set_hybrid_physics_radius(&mut self, radius: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_hybrid_physics_radius(self.inner, radius) };
        check_carla_error(error)
    }

    pub fn register_vehicles<A>(&mut self, actors: &[A]) -> Result<()>
    where
        A: ActorBase,
    {
        let mut actor_ptrs: Vec<*mut carla_actor_t> =
            actors.iter().map(|actor| actor.raw_ptr()).collect();
        let error = unsafe {
            carla_traffic_manager_register_vehicles(
                self.inner,
                actor_ptrs.as_mut_ptr(),
                actor_ptrs.len(),
            )
        };
        check_carla_error(error)
    }

    pub fn unregister_vehicles<A>(&mut self, actors: &[A]) -> Result<()>
    where
        A: ActorBase,
    {
        let mut actor_ptrs: Vec<*mut carla_actor_t> =
            actors.iter().map(|actor| actor.raw_ptr()).collect();
        let error = unsafe {
            carla_traffic_manager_unregister_vehicles(
                self.inner,
                actor_ptrs.as_mut_ptr(),
                actor_ptrs.len(),
            )
        };
        check_carla_error(error)
    }

    pub fn set_percentage_speed_difference<A>(&mut self, actor: &A, percentage: f32) -> Result<()>
    where
        A: ActorBase,
    {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_speed_percentage(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_lane_offset<A: ActorBase>(&mut self, actor: &A, offset: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_lane_offset(self.inner, actor.raw_ptr(), offset)
        };
        check_carla_error(error)
    }

    pub fn set_desired_speed<A: ActorBase>(&mut self, actor: &A, value: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_desired_speed(self.inner, actor.raw_ptr(), value)
        };
        check_carla_error(error)
    }

    pub fn set_global_percentage_speed_difference(&mut self, percentage: f32) -> Result<()> {
        let error =
            unsafe { carla_traffic_manager_set_global_speed_percentage(self.inner, percentage) };
        check_carla_error(error)
    }

    pub fn set_global_lane_offset(&mut self, offset: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_global_lane_offset(self.inner, offset) };
        check_carla_error(error)
    }

    pub fn set_update_vehicle_lights<A: ActorBase>(
        &mut self,
        actor: &A,
        do_update: bool,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_update_lights(self.inner, actor.raw_ptr(), do_update)
        };
        check_carla_error(error)
    }

    pub fn set_collision_detection<A1: ActorBase, A2: ActorBase>(
        &mut self,
        reference_actor: &A1,
        other_actor: &A2,
        detect_collision: bool,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_collision_detection(
                self.inner,
                reference_actor.raw_ptr(),
                other_actor.raw_ptr(),
                detect_collision,
            )
        };
        check_carla_error(error)
    }

    pub fn set_force_lane_change<A: ActorBase>(
        &mut self,
        actor: &A,
        direction: bool,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_force_vehicle_lane_change(self.inner, actor.raw_ptr(), direction)
        };
        check_carla_error(error)
    }

    pub fn set_auto_lane_change<A: ActorBase>(&mut self, actor: &A, enable: bool) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_auto_lane_change(self.inner, actor.raw_ptr(), enable)
        };
        check_carla_error(error)
    }

    pub fn set_distance_to_leading_vehicle<A: ActorBase>(
        &mut self,
        actor: &A,
        distance: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_distance_to_leading_vehicle(
                self.inner,
                actor.raw_ptr(),
                distance,
            )
        };
        check_carla_error(error)
    }

    pub fn set_percentage_ignore_walkers<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_percentage_ignore_walkers(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_percentage_ignore_vehicles<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_percentage_ignore_vehicles(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_percentage_running_light<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_percentage_running_light(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_synchronous_mode(&mut self, yes: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_synchronous_mode(self.inner, yes) };
        check_carla_error(error)
    }

    pub fn set_synchronous_mode_time_out(&mut self, time: Duration) -> Result<()> {
        let timeout_ms = time.as_secs_f64() * 1000.0;
        let error =
            unsafe { carla_traffic_manager_set_synchronous_timeout(self.inner, timeout_ms) };
        check_carla_error(error)
    }

    pub fn synchronous_tick(&mut self) -> bool {
        unsafe { carla_traffic_manager_synchronous_tick(self.inner) }
    }

    pub fn set_global_distance_to_leading_vehicle(&mut self, distance: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_global_distance_to_leading_vehicle(self.inner, distance)
        };
        check_carla_error(error)
    }

    pub fn set_keep_right_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_keep_right_percentage(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_random_left_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_random_left_lane_change_percentage(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_random_right_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_vehicle_random_right_lane_change_percentage(
                self.inner,
                actor.raw_ptr(),
                percentage,
            )
        };
        check_carla_error(error)
    }

    pub fn set_random_device_seed(&mut self, seed: u64) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_random_device_seed(self.inner, seed) };
        check_carla_error(error)
    }

    pub fn shutdown(&mut self) {
        if !self.inner.is_null() {
            unsafe { carla_traffic_manager_shutdown(self.inner) };
            self.inner = ptr::null_mut();
        }
    }

    /// Check if a vehicle is registered with this traffic manager
    pub fn is_vehicle_registered<A: ActorBase>(&self, actor: &A) -> bool {
        unsafe { carla_traffic_manager_is_vehicle_registered(self.inner, actor.raw_ptr()) }
    }

    /// Get traffic manager statistics
    pub fn get_stats(&self) -> Result<carla_traffic_manager_stats_t> {
        let mut stats = unsafe { std::mem::zeroed() };
        let error = unsafe { carla_traffic_manager_get_stats(self.inner, &mut stats) };
        check_carla_error(error)?;
        Ok(stats)
    }

    /// Reset traffic manager statistics
    pub fn reset_stats(&mut self) -> Result<()> {
        let error = unsafe { carla_traffic_manager_reset_stats(self.inner) };
        check_carla_error(error)
    }

    pub fn next_action<A: ActorBase>(
        &mut self,
        actor: &A,
    ) -> Result<Option<carla_traffic_manager_action_t>> {
        let mut action = carla_traffic_manager_action_t {
            road_option: carla_road_option_t_CARLA_ROAD_OPTION_VOID,
            waypoint: ptr::null_mut(),
        };
        let error = unsafe {
            carla_traffic_manager_get_vehicle_next_action(self.inner, actor.raw_ptr(), &mut action)
        };
        match error {
            carla_error_t_CARLA_ERROR_NONE => Ok(Some(action)),
            carla_error_t_CARLA_ERROR_NO_DATA => Ok(None),
            _ => {
                check_carla_error(error)?;
                Ok(None)
            }
        }
    }

    pub fn action_buffer<A: ActorBase>(
        &mut self,
        actor: &A,
    ) -> Result<Option<carla_traffic_manager_action_buffer_t>> {
        let mut buffer = carla_traffic_manager_action_buffer_t {
            actions: ptr::null_mut(),
            action_count: 0,
            capacity: 0,
        };
        let error = unsafe {
            carla_traffic_manager_get_vehicle_action_buffer(
                self.inner,
                actor.raw_ptr(),
                &mut buffer,
            )
        };
        match error {
            carla_error_t_CARLA_ERROR_NONE => Ok(Some(buffer)),
            carla_error_t_CARLA_ERROR_NO_DATA => Ok(None),
            _ => {
                check_carla_error(error)?;
                Ok(None)
            }
        }
    }
}

impl Drop for TrafficManager {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_traffic_manager_shutdown(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: TrafficManager wraps a thread-safe C API
unsafe impl Send for TrafficManager {}
unsafe impl Sync for TrafficManager {}
