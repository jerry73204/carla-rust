// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.
//
// Additionally, actor.cxx_actor().as_ref().unwrap_unchecked() is safe because:
// - All ActorBase types (Actor, Vehicle, etc.) are constructed via from_cxx()
// - from_cxx() only returns Some when the SharedPtr is non-null
// - Therefore, cxx_actor() always returns a non-null SharedPtr
// - as_ref() on a non-null SharedPtr is guaranteed to succeed

use super::{Action, ActionBuffer, PrivateAction};
use crate::{
    client::ActorBase,
    geom::{Location, LocationExt},
    rpc::ActorId,
    utils::CxxVectorExt,
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::{client::FfiActor, traffic_manager::FfiTrafficManager};
use cxx::{CxxVector, SharedPtr, UniquePtr};
use derivative::Derivative;
use nalgebra::Point3;
use static_assertions::assert_impl_all;
use std::time::Duration;

/// Manages groups of autopilot vehicles with realistic urban traffic behavior.
///
/// Corresponds to `carla.TrafficManager` in Python API.
///
/// The traffic manager coordinates multiple vehicles in autopilot mode, providing
/// sophisticated control over their driving behavior. It enables:
/// - Customized speeds and lane behaviors per vehicle or globally
/// - Collision avoidance and safe following distances
/// - Realistic traffic rule violations (running lights, ignoring pedestrians)
/// - Custom routes and paths
/// - Performance optimizations for large fleets
///
/// # Usage Pattern
///
/// 1. Get a TrafficManager instance from [`Client::instance_tm()`](crate::client::Client::instance_tm)
/// 2. Spawn vehicles using [`World::spawn_actor()`](crate::client::World::spawn_actor)
/// 3. Register vehicles with [`register_vehicles()`](Self::register_vehicles)
/// 4. Configure behavior using setter methods
/// 5. Enable autopilot on vehicles with [`Vehicle::set_autopilot()`](crate::client::Vehicle::set_autopilot)
///
/// # Examples
///
/// See [module-level documentation](crate::traffic_manager) for examples.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct TrafficManager {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiTrafficManager>,
}

impl TrafficManager {
    pub fn port(&self) -> u16 {
        self.inner.Port()
    }

    pub fn is_valid_port(&self) -> bool {
        self.inner.IsValidPort()
    }

    pub fn set_osm_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetOSMMode(yes);
    }

    pub fn set_custom_path<A, P>(&mut self, actor: &A, path: &[P], empty_buffer: bool)
    where
        A: ActorBase,
        P: AsRef<Point3<f32>>,
    {
        let path = path.iter().fold(CxxVector::new_typed(), |mut vec, point| {
            let point = <Location as LocationExt>::from_na_point(point.as_ref());
            vec.pin_mut().push(point);
            vec
        });

        self.inner.pin_mut().SetCustomPath(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            &path,
            empty_buffer,
        );
    }

    pub fn remove_upload_path(&mut self, actor_id: ActorId, remove_path: bool) {
        self.inner
            .pin_mut()
            .RemoveUploadPath(&actor_id, remove_path);
    }

    pub fn update_upload_path<P>(&mut self, actor_id: ActorId, path: &[P])
    where
        P: AsRef<Point3<f32>>,
    {
        let path = path.iter().fold(CxxVector::new_typed(), |mut vec, point| {
            let point = <Location as LocationExt>::from_na_point(point.as_ref());
            vec.pin_mut().push(point);
            vec
        });

        self.inner.pin_mut().UpdateUploadPath(&actor_id, &path);
    }

    pub fn set_import_route<A: ActorBase>(&mut self, actor: &A, route: &[u8], empty_buffer: bool) {
        let route = route.iter().fold(CxxVector::new_typed(), |mut vec, &val| {
            vec.pin_mut().push(val);
            vec
        });

        self.inner.pin_mut().SetImportedRoute(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            route,
            empty_buffer,
        );
    }

    pub fn remove_imported_route(&mut self, actor_id: ActorId, remove_path: bool) {
        self.inner
            .pin_mut()
            .RemoveImportedRoute(&actor_id, remove_path);
    }

    pub fn update_imported_route(&mut self, actor_id: ActorId, route: &[u8]) {
        let route = route.iter().fold(CxxVector::new_typed(), |mut vec, &val| {
            vec.pin_mut().push(val);
            vec
        });

        self.inner.pin_mut().UpdateImportedRoute(&actor_id, route);
    }

    pub fn set_respawn_dormant_vehicles(&mut self, yes: bool) {
        self.inner.pin_mut().SetRespawnDormantVehicles(yes);
    }

    pub fn set_boundaries_respawn_dormant_vehicles(&mut self, lower_bound: f32, upper_bound: f32) {
        self.inner
            .pin_mut()
            .SetBoundariesRespawnDormantVehicles(lower_bound, upper_bound);
    }

    pub fn set_max_boundaries(&mut self, lower: f32, upper: f32) {
        self.inner.pin_mut().SetMaxBoundaries(lower, upper);
    }

    pub fn set_hybrid_physics_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetHybridPhysicsMode(yes);
    }

    pub fn set_hybrid_physics_radius(&mut self, radius: f32) {
        self.inner.pin_mut().SetHybridPhysicsRadius(radius);
    }

    pub fn register_vehicles<A>(&mut self, actors: &[A])
    where
        A: ActorBase,
    {
        let actors: Vec<SharedPtr<FfiActor>> =
            actors.iter().map(|actor| actor.cxx_actor()).collect();
        let ptr = actors.as_ptr();
        let len = actors.len();
        unsafe { self.inner.pin_mut().RegisterVehicles(ptr, len) };
    }

    pub fn unregister_vehicles<A>(&mut self, actors: &[A])
    where
        A: ActorBase,
    {
        let actors: Vec<SharedPtr<FfiActor>> =
            actors.iter().map(|actor| actor.cxx_actor()).collect();
        let ptr = actors.as_ptr();
        let len = actors.len();
        unsafe { self.inner.pin_mut().UnregisterVehicles(ptr, len) };
    }

    pub fn set_percentage_speed_difference<A>(&mut self, actor: &A, percentage: f32)
    where
        A: ActorBase,
    {
        self.inner.pin_mut().SetPercentageSpeedDifference(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_lane_offset<A: ActorBase>(&mut self, actor: &A, offset: f32) {
        self.inner.pin_mut().SetLaneOffset(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            offset,
        );
    }

    pub fn set_desired_speed<A: ActorBase>(&mut self, actor: &A, value: f32) {
        self.inner.pin_mut().SetDesiredSpeed(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            value,
        );
    }

    pub fn set_global_percentage_speed_difference(&mut self, percentage: f32) {
        self.inner
            .pin_mut()
            .SetGlobalPercentageSpeedDifference(percentage);
    }

    pub fn set_global_lane_offset(&mut self, offset: f32) {
        self.inner.pin_mut().SetGlobalLaneOffset(offset);
    }

    pub fn set_update_vehicle_lights<A: ActorBase>(&mut self, actor: &A, do_update: bool) {
        self.inner.pin_mut().SetUpdateVehicleLights(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            do_update,
        );
    }

    pub fn set_collision_detection<A1: ActorBase, A2: ActorBase>(
        &mut self,
        reference_actor: &A1,
        other_actor: &A2,
        detect_collision: bool,
    ) {
        self.inner.pin_mut().SetCollisionDetection(
            unsafe { reference_actor.cxx_actor().as_ref().unwrap_unchecked() },
            unsafe { other_actor.cxx_actor().as_ref().unwrap_unchecked() },
            detect_collision,
        );
    }

    pub fn set_force_lane_change<A: ActorBase>(&mut self, actor: &A, direction: bool) {
        self.inner.pin_mut().SetForceLaneChange(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            direction,
        );
    }

    pub fn set_auto_lane_change<A: ActorBase>(&mut self, actor: &A, enable: bool) {
        self.inner.pin_mut().SetAutoLaneChange(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            enable,
        );
    }

    pub fn set_distance_to_leading_vehicle<A: ActorBase>(&mut self, actor: &A, distance: f32) {
        self.inner.pin_mut().SetDistanceToLeadingVehicle(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            distance,
        );
    }

    pub fn set_percentage_ignore_walkers<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetPercentageIgnoreWalkers(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_percentage_ignore_vehicles<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetPercentageIgnoreVehicles(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_percentage_running_light<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetPercentageRunningLight(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_synchronous_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetSynchronousMode(yes);
    }

    pub fn set_synchronous_mode_time_out(&mut self, time: Duration) {
        // Use as_millis() to avoid floating point overflow and precision loss
        let millis = time.as_millis();

        // Check for overflow - f64 max is ~1.8e308, but practical limit for milliseconds
        // is much lower. We cap at f64::MAX to avoid overflow in the conversion.
        let millis_f64 = if millis > f64::MAX as u128 {
            f64::MAX
        } else {
            millis as f64
        };

        self.inner
            .pin_mut()
            .SetSynchronousModeTimeOutInMiliSecond(millis_f64);
    }

    pub fn synchronous_tick(&mut self) -> bool {
        self.inner.pin_mut().SynchronousTick()
    }

    pub fn set_global_distance_to_leading_vehicle(&mut self, distance: f32) {
        self.inner
            .pin_mut()
            .SetGlobalDistanceToLeadingVehicle(distance);
    }

    pub fn set_keep_right_percentage<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetKeepRightPercentage(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_random_left_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) {
        self.inner.pin_mut().SetRandomLeftLaneChangePercentage(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_random_right_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) {
        self.inner.pin_mut().SetRandomRightLaneChangePercentage(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    pub fn set_random_device_seed(&mut self, seed: u64) {
        self.inner.pin_mut().SetRandomDeviceSeed(seed);
    }

    pub fn shutdown(mut self) {
        self.inner.pin_mut().ShutDown();
        self.inner = UniquePtr::null();
    }

    pub fn next_action(&mut self, actor_id: ActorId) -> Action {
        let action = self
            .inner
            .pin_mut()
            .GetNextAction(&actor_id)
            .within_unique_ptr();
        unsafe { PrivateAction::from_cxx(action).unwrap_unchecked() }.to_pair()
    }

    pub fn action_buffer(&mut self, actor_id: ActorId) -> ActionBuffer {
        let ptr = self
            .inner
            .pin_mut()
            .GetActionBuffer(&actor_id)
            .within_unique_ptr();
        unsafe { ActionBuffer::from_cxx(ptr).unwrap_unchecked() }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiTrafficManager>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }

        Some(Self { inner: ptr })
    }
}

impl Drop for TrafficManager {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            self.inner.pin_mut().ShutDown();
        }
    }
}

assert_impl_all!(TrafficManager: Send);
