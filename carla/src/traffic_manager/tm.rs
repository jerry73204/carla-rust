// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.
//
// Additionally, actor.cxx_actor().as_ref().unwrap_unchecked() is safe because:
// - All ActorBase types (Actor, Vehicle, etc.) are constructed via from_cxx()
// - from_cxx() only returns Some when the SharedPtr is non-null
// - Therefore, cxx_actor() always returns a non-null SharedPtr
// - as_ref() on a non-null SharedPtr is guaranteed to succeed

use super::{Action, ActionBuffer, PrivateAction};
use crate::{client::ActorBase, geom::Location, rpc::ActorId, utils::CxxVectorExt};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::{client::FfiActor, traffic_manager::FfiTrafficManager};
use cxx::{CxxVector, SharedPtr, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::time::Duration;

/// Manages groups of autopilot vehicles with realistic urban traffic behavior.
///
/// The traffic manager coordinates multiple vehicles in autopilot mode, providing
/// sophisticated control over their driving behavior. It enables:
/// - Customized speeds and lane behaviors per vehicle or globally
/// - Collision avoidance and safe following distances
/// - Realistic traffic rule violations (running lights, ignoring pedestrians)
/// - Custom routes and paths
/// - Performance optimizations for large fleets
///
/// Corresponds to [`carla.TrafficManager`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager) in the Python API.
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
    /// Returns the port used by this traffic manager instance.
    ///
    /// See [carla.TrafficManager.get_port](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.get_port)
    /// in the Python API.
    pub fn port(&self) -> u16 {
        self.inner.Port()
    }

    /// Returns true if the traffic manager is using a valid port.
    pub fn is_valid_port(&self) -> bool {
        self.inner.IsValidPort()
    }

    /// Enables or disables OSM (OpenStreetMap) mode.
    ///
    /// See [carla.TrafficManager.set_osm_mode](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_osm_mode)
    /// in the Python API.
    pub fn set_osm_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetOSMMode(yes);
    }

    /// Sets a custom path for a vehicle to follow.
    ///
    /// See [carla.TrafficManager.set_path](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_path)
    /// in the Python API.
    pub fn set_custom_path<A, P>(&mut self, actor: &A, path: &[P], empty_buffer: bool)
    where
        A: ActorBase,
        P: AsRef<Location>,
    {
        let path = path.iter().fold(CxxVector::new_typed(), |mut vec, point| {
            let point = point.as_ref();
            vec.pin_mut().push(point.into_ffi());
            vec
        });

        self.inner.pin_mut().SetCustomPath(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            &path,
            empty_buffer,
        );
    }

    /// Removes an uploaded path for a vehicle.
    pub fn remove_upload_path(&mut self, actor_id: ActorId, remove_path: bool) {
        self.inner
            .pin_mut()
            .RemoveUploadPath(&actor_id, remove_path);
    }

    /// Updates an uploaded path for a vehicle.
    pub fn update_upload_path<P>(&mut self, actor_id: ActorId, path: &[P])
    where
        P: AsRef<Location>,
    {
        let path = path.iter().fold(CxxVector::new_typed(), |mut vec, point| {
            let point = point.as_ref();
            vec.pin_mut().push(point.into_ffi());
            vec
        });

        self.inner.pin_mut().UpdateUploadPath(&actor_id, &path);
    }

    /// Sets an imported route for a vehicle to follow.
    ///
    /// See [carla.TrafficManager.set_imported_route](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_imported_route)
    /// in the Python API.
    pub fn set_import_route<A: ActorBase>(&mut self, actor: &A, route: &[u8], empty_buffer: bool) {
        let route = route.iter().fold(CxxVector::new_typed(), |mut vec, &val| {
            vec.pin_mut().push(val);
            vec
        });

        self.inner.pin_mut().SetImportedRoute(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            route.as_ref().unwrap(),
            empty_buffer,
        );
    }

    /// Removes an imported route for a vehicle.
    pub fn remove_imported_route(&mut self, actor_id: ActorId, remove_path: bool) {
        self.inner
            .pin_mut()
            .RemoveImportedRoute(&actor_id, remove_path);
    }

    /// Updates an imported route for a vehicle.
    pub fn update_imported_route(&mut self, actor_id: ActorId, route: &[u8]) {
        let route = route.iter().fold(CxxVector::new_typed(), |mut vec, &val| {
            vec.pin_mut().push(val);
            vec
        });

        self.inner
            .pin_mut()
            .UpdateImportedRoute(&actor_id, route.as_ref().unwrap());
    }

    /// Enables or disables respawning of dormant vehicles.
    ///
    /// See [carla.TrafficManager.set_respawn_dormant_vehicles](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_respawn_dormant_vehicles)
    /// in the Python API.
    pub fn set_respawn_dormant_vehicles(&mut self, yes: bool) {
        self.inner.pin_mut().SetRespawnDormantVehicles(yes);
    }

    /// Sets the boundaries for respawning dormant vehicles.
    ///
    /// See [carla.TrafficManager.set_boundaries_respawn_dormant_vehicles](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_boundaries_respawn_dormant_vehicles)
    /// in the Python API.
    pub fn set_boundaries_respawn_dormant_vehicles(&mut self, lower_bound: f32, upper_bound: f32) {
        self.inner
            .pin_mut()
            .SetBoundariesRespawnDormantVehicles(lower_bound, upper_bound);
    }

    /// Sets the maximum boundaries for the simulation.
    pub fn set_max_boundaries(&mut self, lower: f32, upper: f32) {
        self.inner.pin_mut().SetMaxBoundaries(lower, upper);
    }

    /// Enables or disables hybrid physics mode for performance optimization.
    ///
    /// In hybrid physics mode, only vehicles near the ego vehicle use full physics simulation.
    ///
    /// See [carla.TrafficManager.set_hybrid_physics_mode](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_hybrid_physics_mode)
    /// in the Python API.
    pub fn set_hybrid_physics_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetHybridPhysicsMode(yes);
    }

    /// Sets the radius for hybrid physics mode.
    ///
    /// Vehicles within this radius of the ego vehicle will use full physics simulation.
    ///
    /// See [carla.TrafficManager.set_hybrid_physics_radius](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_hybrid_physics_radius)
    /// in the Python API.
    pub fn set_hybrid_physics_radius(&mut self, radius: f32) {
        self.inner.pin_mut().SetHybridPhysicsRadius(radius);
    }

    /// Registers vehicles with the traffic manager for autopilot control.
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

    /// Unregisters vehicles from the traffic manager.
    ///
    /// Removes vehicles from autopilot control.
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

    /// Sets the percentage difference from the speed limit for a vehicle.
    ///
    /// A positive percentage makes the vehicle drive slower, negative makes it faster.
    /// For example, 20 means the vehicle will drive at 80% of the speed limit.
    ///
    /// See [carla.TrafficManager.vehicle_percentage_speed_difference](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.vehicle_percentage_speed_difference)
    /// in the Python API.
    pub fn set_percentage_speed_difference<A>(&mut self, actor: &A, percentage: f32)
    where
        A: ActorBase,
    {
        self.inner.pin_mut().SetPercentageSpeedDifference(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    /// Sets the lane offset for a vehicle.
    ///
    /// A positive offset makes the vehicle drive to the right of the lane center,
    /// negative makes it drive to the left.
    ///
    /// See [carla.TrafficManager.update_vehicle_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.update_vehicle_lights)
    /// in the Python API.
    pub fn set_lane_offset<A: ActorBase>(&mut self, actor: &A, offset: f32) {
        self.inner.pin_mut().SetLaneOffset(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            offset,
        );
    }

    /// Sets a specific desired speed for a vehicle in m/s.
    ///
    /// This overrides the speed limit-based speed calculation.
    pub fn set_desired_speed<A: ActorBase>(&mut self, actor: &A, value: f32) {
        self.inner.pin_mut().SetDesiredSpeed(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            value,
        );
    }

    /// Sets the percentage difference from the speed limit for all vehicles.
    ///
    /// A positive percentage makes vehicles drive slower, negative makes them faster.
    ///
    /// See [carla.TrafficManager.global_percentage_speed_difference](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.global_percentage_speed_difference)
    /// in the Python API.
    pub fn set_global_percentage_speed_difference(&mut self, percentage: f32) {
        self.inner
            .pin_mut()
            .SetGlobalPercentageSpeedDifference(percentage);
    }

    /// Sets the lane offset for all vehicles.
    ///
    /// A positive offset makes vehicles drive to the right of the lane center,
    /// negative makes them drive to the left.
    pub fn set_global_lane_offset(&mut self, offset: f32) {
        self.inner.pin_mut().SetGlobalLaneOffset(offset);
    }

    /// Enables or disables automatic vehicle light updates for a vehicle.
    ///
    /// When enabled, the traffic manager automatically updates the vehicle's lights
    /// based on the situation (brake lights, turn signals, etc.).
    ///
    /// See [carla.TrafficManager.update_vehicle_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.update_vehicle_lights)
    /// in the Python API.
    pub fn set_update_vehicle_lights<A: ActorBase>(&mut self, actor: &A, do_update: bool) {
        self.inner.pin_mut().SetUpdateVehicleLights(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            do_update,
        );
    }

    /// Enables or disables collision detection between two vehicles.
    ///
    /// When disabled, the reference vehicle will ignore collisions with the other vehicle.
    ///
    /// See [carla.TrafficManager.collision_detection](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.collision_detection)
    /// in the Python API.
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

    /// Forces a lane change for a vehicle.
    ///
    /// The direction parameter indicates which direction to change: true for left, false for right.
    ///
    /// See [carla.TrafficManager.force_lane_change](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.force_lane_change)
    /// in the Python API.
    pub fn set_force_lane_change<A: ActorBase>(&mut self, actor: &A, direction: bool) {
        self.inner.pin_mut().SetForceLaneChange(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            direction,
        );
    }

    /// Enables or disables automatic lane changes for a vehicle.
    ///
    /// When enabled, the vehicle can autonomously change lanes when appropriate.
    ///
    /// See [carla.TrafficManager.auto_lane_change](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.auto_lane_change)
    /// in the Python API.
    pub fn set_auto_lane_change<A: ActorBase>(&mut self, actor: &A, enable: bool) {
        self.inner.pin_mut().SetAutoLaneChange(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            enable,
        );
    }

    /// Sets the minimum distance to maintain from the leading vehicle in meters.
    ///
    /// This controls the safe following distance for the vehicle.
    ///
    /// See [carla.TrafficManager.distance_to_leading_vehicle](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.distance_to_leading_vehicle)
    /// in the Python API.
    pub fn set_distance_to_leading_vehicle<A: ActorBase>(&mut self, actor: &A, distance: f32) {
        self.inner.pin_mut().SetDistanceToLeadingVehicle(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            distance,
        );
    }

    /// Sets the percentage chance for a vehicle to ignore pedestrians.
    ///
    /// A value of 100 means the vehicle will always ignore pedestrians, 0 means it will always respect them.
    ///
    /// See [carla.TrafficManager.ignore_walkers_percentage](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.ignore_walkers_percentage)
    /// in the Python API.
    pub fn set_percentage_ignore_walkers<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetPercentageIgnoreWalkers(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    /// Sets the percentage chance for a vehicle to ignore other vehicles.
    ///
    /// A value of 100 means the vehicle will always ignore other vehicles, 0 means it will always respect them.
    ///
    /// See [carla.TrafficManager.ignore_vehicles_percentage](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.ignore_vehicles_percentage)
    /// in the Python API.
    pub fn set_percentage_ignore_vehicles<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetPercentageIgnoreVehicles(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    /// Sets the percentage chance for a vehicle to run red lights.
    ///
    /// A value of 100 means the vehicle will always run red lights, 0 means it will always stop.
    ///
    /// See [carla.TrafficManager.ignore_lights_percentage](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.ignore_lights_percentage)
    /// in the Python API.
    pub fn set_percentage_running_light<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetPercentageRunningLight(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    /// Enables or disables synchronous mode for the traffic manager.
    ///
    /// In synchronous mode, the traffic manager waits for a tick before updating.
    ///
    /// See [carla.TrafficManager.set_synchronous_mode](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_synchronous_mode)
    /// in the Python API.
    pub fn set_synchronous_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetSynchronousMode(yes);
    }

    /// Sets the timeout for synchronous mode.
    ///
    /// This defines how long the traffic manager will wait for a world tick in synchronous mode.
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

    /// Executes one tick in synchronous mode.
    ///
    /// Returns true if the tick was successful.
    ///
    /// See [carla.TrafficManager.tick](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.tick)
    /// in the Python API.
    pub fn synchronous_tick(&mut self) -> bool {
        self.inner.pin_mut().SynchronousTick()
    }

    /// Sets the minimum distance to maintain from the leading vehicle for all vehicles.
    ///
    /// This controls the safe following distance globally.
    ///
    /// See [carla.TrafficManager.set_global_distance_to_leading_vehicle](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_global_distance_to_leading_vehicle)
    /// in the Python API.
    pub fn set_global_distance_to_leading_vehicle(&mut self, distance: f32) {
        self.inner
            .pin_mut()
            .SetGlobalDistanceToLeadingVehicle(distance);
    }

    /// Sets the percentage tendency for a vehicle to keep to the right lane.
    ///
    /// Higher values make the vehicle more likely to stay in the right lane.
    ///
    /// Note: This method is only available in CARLA versions before 0.9.16.
    #[cfg(not(carla_0916))]
    pub fn set_keep_right_percentage<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner.pin_mut().SetKeepRightPercentage(
            unsafe { actor.cxx_actor().as_ref().unwrap_unchecked() },
            percentage,
        );
    }

    /// Sets the percentage chance for a vehicle to randomly change to the left lane.
    ///
    /// Higher values increase the likelihood of random left lane changes.
    ///
    /// See [carla.TrafficManager.random_left_lanechange_percentage](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.random_left_lanechange_percentage)
    /// in the Python API.
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

    /// Sets the percentage chance for a vehicle to randomly change to the right lane.
    ///
    /// Higher values increase the likelihood of random right lane changes.
    ///
    /// See [carla.TrafficManager.random_right_lanechange_percentage](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.random_right_lanechange_percentage)
    /// in the Python API.
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

    /// Sets the random seed for the traffic manager's random number generator.
    ///
    /// This allows for reproducible traffic behavior across runs.
    ///
    /// See [carla.TrafficManager.set_random_device_seed](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.set_random_device_seed)
    /// in the Python API.
    pub fn set_random_device_seed(&mut self, seed: u64) {
        self.inner.pin_mut().SetRandomDeviceSeed(seed);
    }

    /// Shuts down the traffic manager and releases all resources.
    ///
    /// See [carla.TrafficManager.shut_down](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager.shut_down)
    /// in the Python API.
    pub fn shutdown(mut self) {
        self.inner.pin_mut().ShutDown();
        self.inner = UniquePtr::null();
    }

    /// Returns the next action planned for a vehicle.
    ///
    /// This provides access to the traffic manager's decision for the vehicle's next move.
    pub fn next_action(&mut self, actor_id: ActorId) -> Action {
        let action = self
            .inner
            .pin_mut()
            .GetNextAction(&actor_id)
            .within_unique_ptr();
        unsafe { PrivateAction::from_cxx(action).unwrap_unchecked() }.to_pair()
    }

    /// Returns the action buffer for a vehicle.
    ///
    /// The action buffer contains the sequence of planned actions for the vehicle.
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
