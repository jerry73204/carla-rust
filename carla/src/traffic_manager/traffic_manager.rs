use super::{Action, ActionBuffer};
use crate::{
    client::ActorBase,
    geom::Location,
    rpc::ActorId,
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
}

impl TrafficManager {
    /// Create a new TrafficManager instance.
    pub(crate) fn new(port: u16) -> Result<Self> {
        let tm_ptr = unsafe { carla_traffic_manager_create(port) };
        if tm_ptr.is_null() {
            return Err(anyhow!("Failed to create traffic manager on port {}", port));
        }
        Ok(Self { inner: tm_ptr })
    }

    /// Create a TrafficManager from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_traffic_manager_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null traffic manager pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn port(&self) -> u16 {
        unsafe { carla_traffic_manager_get_port(self.inner) }
    }

    pub fn is_valid_port(&self) -> bool {
        unsafe { carla_traffic_manager_is_valid_port(self.inner) }
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
        let c_path: Vec<carla_location_t> = path
            .iter()
            .map(|point| {
                let p = point.as_ref();
                carla_location_t {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                }
            })
            .collect();

        let error = unsafe {
            carla_traffic_manager_set_custom_path(
                self.inner,
                actor.raw_ptr(),
                c_path.as_ptr(),
                c_path.len(),
                empty_buffer,
            )
        };
        check_carla_error(error)
    }

    pub fn remove_upload_path(&mut self, actor_id: ActorId, remove_path: bool) -> Result<()> {
        let c_actor_id = actor_id as carla_actor_id_t;
        let error = unsafe { carla_traffic_manager_remove_upload_path(self.inner, c_actor_id, remove_path) };
        check_carla_error(error)
    }

    pub fn update_upload_path<P>(&mut self, actor_id: ActorId, path: &[P]) -> Result<()>
    where
        P: AsRef<Point3<f32>>,
    {
        let c_path: Vec<carla_location_t> = path
            .iter()
            .map(|point| {
                let p = point.as_ref();
                carla_location_t {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                }
            })
            .collect();

        let c_actor_id = actor_id as carla_actor_id_t;
        let error = unsafe {
            carla_traffic_manager_update_upload_path(
                self.inner,
                c_actor_id,
                c_path.as_ptr(),
                c_path.len(),
            )
        };
        check_carla_error(error)
    }

    pub fn set_import_route<A: ActorBase>(&mut self, actor: &A, route: &[u8], empty_buffer: bool) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_imported_route(
                self.inner,
                actor.raw_ptr(),
                route.as_ptr(),
                route.len(),
                empty_buffer,
            )
        };
        check_carla_error(error)
    }

    pub fn remove_imported_route(&mut self, actor_id: ActorId, remove_path: bool) -> Result<()> {
        let c_actor_id = actor_id as carla_actor_id_t;
        let error = unsafe { carla_traffic_manager_remove_imported_route(self.inner, c_actor_id, remove_path) };
        check_carla_error(error)
    }

    pub fn update_imported_route(&mut self, actor_id: ActorId, route: &[u8]) -> Result<()> {
        let c_actor_id = actor_id as carla_actor_id_t;
        let error = unsafe {
            carla_traffic_manager_update_imported_route(
                self.inner,
                c_actor_id,
                route.as_ptr(),
                route.len(),
            )
        };
        check_carla_error(error)
    }

    pub fn set_respawn_dormant_vehicles(&mut self, yes: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_respawn_dormant_vehicles(self.inner, yes) };
        check_carla_error(error)
    }

    pub fn set_boundaries_respawn_dormant_vehicles(&mut self, lower_bound: f32, upper_bound: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_boundaries_respawn_dormant_vehicles(self.inner, lower_bound, upper_bound)
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
        let actor_ptrs: Vec<*mut carla_actor_t> = actors.iter().map(|actor| actor.raw_ptr()).collect();
        let error = unsafe {
            carla_traffic_manager_register_vehicles(
                self.inner,
                actor_ptrs.as_ptr(),
                actor_ptrs.len(),
            )
        };
        check_carla_error(error)
    }

    pub fn unregister_vehicles<A>(&mut self, actors: &[A]) -> Result<()>
    where
        A: ActorBase,
    {
        let actor_ptrs: Vec<*mut carla_actor_t> = actors.iter().map(|actor| actor.raw_ptr()).collect();
        let error = unsafe {
            carla_traffic_manager_unregister_vehicles(
                self.inner,
                actor_ptrs.as_ptr(),
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
            carla_traffic_manager_set_percentage_speed_difference(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_lane_offset<A: ActorBase>(&mut self, actor: &A, offset: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_lane_offset(self.inner, actor.raw_ptr(), offset) };
        check_carla_error(error)
    }

    pub fn set_desired_speed<A: ActorBase>(&mut self, actor: &A, value: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_desired_speed(self.inner, actor.raw_ptr(), value) };
        check_carla_error(error)
    }

    pub fn set_global_percentage_speed_difference(&mut self, percentage: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_global_percentage_speed_difference(self.inner, percentage) };
        check_carla_error(error)
    }

    pub fn set_global_lane_offset(&mut self, offset: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_global_lane_offset(self.inner, offset) };
        check_carla_error(error)
    }

    pub fn set_update_vehicle_lights<A: ActorBase>(&mut self, actor: &A, do_update: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_update_vehicle_lights(self.inner, actor.raw_ptr(), do_update) };
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

    pub fn set_force_lane_change<A: ActorBase>(&mut self, actor: &A, direction: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_force_lane_change(self.inner, actor.raw_ptr(), direction) };
        check_carla_error(error)
    }

    pub fn set_auto_lane_change<A: ActorBase>(&mut self, actor: &A, enable: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_auto_lane_change(self.inner, actor.raw_ptr(), enable) };
        check_carla_error(error)
    }

    pub fn set_distance_to_leading_vehicle<A: ActorBase>(&mut self, actor: &A, distance: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_distance_to_leading_vehicle(self.inner, actor.raw_ptr(), distance)
        };
        check_carla_error(error)
    }

    pub fn set_percentage_ignore_walkers<A: ActorBase>(&mut self, actor: &A, percentage: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_percentage_ignore_walkers(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_percentage_ignore_vehicles<A: ActorBase>(&mut self, actor: &A, percentage: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_percentage_ignore_vehicles(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_percentage_running_light<A: ActorBase>(&mut self, actor: &A, percentage: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_percentage_running_light(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_synchronous_mode(&mut self, yes: bool) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_synchronous_mode(self.inner, yes) };
        check_carla_error(error)
    }

    pub fn set_synchronous_mode_time_out(&mut self, time: Duration) -> Result<()> {
        let timeout_ms = time.as_secs_f64() * 1000.0;
        let error = unsafe { carla_traffic_manager_set_synchronous_mode_timeout(self.inner, timeout_ms) };
        check_carla_error(error)
    }

    pub fn synchronous_tick(&mut self) -> bool {
        unsafe { carla_traffic_manager_synchronous_tick(self.inner) }
    }

    pub fn set_global_distance_to_leading_vehicle(&mut self, distance: f32) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_global_distance_to_leading_vehicle(self.inner, distance) };
        check_carla_error(error)
    }

    pub fn set_keep_right_percentage<A: ActorBase>(&mut self, actor: &A, percentage: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_keep_right_percentage(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_random_left_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_random_left_lane_change_percentage(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_random_right_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_traffic_manager_set_random_right_lane_change_percentage(self.inner, actor.raw_ptr(), percentage)
        };
        check_carla_error(error)
    }

    pub fn set_random_device_seed(&mut self, seed: u64) -> Result<()> {
        let error = unsafe { carla_traffic_manager_set_random_device_seed(self.inner, seed) };
        check_carla_error(error)
    }

    pub fn shutdown(&mut self) -> Result<()> {
        let error = unsafe { carla_traffic_manager_shutdown(self.inner) };
        self.inner = ptr::null_mut();
        check_carla_error(error)
    }

    pub fn next_action(&mut self, actor_id: ActorId) -> Option<Action> {
        let c_actor_id = actor_id as carla_actor_id_t;
        let action_ptr = unsafe { carla_traffic_manager_get_next_action(self.inner, c_actor_id) };
        if action_ptr.is_null() {
            None
        } else {
            // TODO: Implement proper Action conversion once Waypoint is updated to C FFI
            // For now, return None until Waypoint C FFI is implemented
            None
        }
    }

    pub fn action_buffer(&mut self, actor_id: ActorId) -> Option<ActionBuffer> {
        let c_actor_id = actor_id as carla_actor_id_t;
        let buffer_ptr = unsafe { carla_traffic_manager_get_action_buffer(self.inner, c_actor_id) };
        if buffer_ptr.is_null() {
            None
        } else {
            ActionBuffer::from_raw_ptr(buffer_ptr).ok()
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
