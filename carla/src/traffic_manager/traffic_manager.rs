use crate::{
    client::{Actor, ActorBase},
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

use super::{Action, ActionBuffer, PrivateAction};

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

    pub fn set_custom_path<P>(&mut self, actor: &Actor, path: &[P], empty_buffer: bool)
    where
        P: AsRef<Point3<f32>>,
    {
        let path = path.iter().fold(CxxVector::new_typed(), |mut vec, point| {
            let point = <Location as LocationExt>::from_na_point(point.as_ref());
            vec.pin_mut().push(point);
            vec
        });

        self.inner
            .pin_mut()
            .SetCustomPath(&actor.inner, &path, empty_buffer);
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
            actor.cxx_actor().as_ref().unwrap(),
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
        self.inner
            .pin_mut()
            .SetPercentageSpeedDifference(actor.cxx_actor().as_ref().unwrap(), percentage);
    }

    pub fn set_lane_offset<A: ActorBase>(&mut self, actor: &A, offset: f32) {
        self.inner
            .pin_mut()
            .SetLaneOffset(actor.cxx_actor().as_ref().unwrap(), offset);
    }

    pub fn set_desired_speed<A: ActorBase>(&mut self, actor: &A, value: f32) {
        self.inner
            .pin_mut()
            .SetDesiredSpeed(actor.cxx_actor().as_ref().unwrap(), value);
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
        self.inner
            .pin_mut()
            .SetUpdateVehicleLights(actor.cxx_actor().as_ref().unwrap(), do_update);
    }

    pub fn set_collision_detection<A1: ActorBase, A2: ActorBase>(
        &mut self,
        reference_actor: &A1,
        other_actor: &A2,
        detect_collision: bool,
    ) {
        self.inner.pin_mut().SetCollisionDetection(
            reference_actor.cxx_actor().as_ref().unwrap(),
            other_actor.cxx_actor().as_ref().unwrap(),
            detect_collision,
        );
    }

    pub fn set_force_lane_change<A: ActorBase>(&mut self, actor: &A, direction: bool) {
        self.inner
            .pin_mut()
            .SetForceLaneChange(actor.cxx_actor().as_ref().unwrap(), direction);
    }

    pub fn set_auto_lane_change<A: ActorBase>(&mut self, actor: &A, enable: bool) {
        self.inner
            .pin_mut()
            .SetAutoLaneChange(actor.cxx_actor().as_ref().unwrap(), enable);
    }

    pub fn set_distance_to_leading_vehicle<A: ActorBase>(&mut self, actor: &A, distance: f32) {
        self.inner
            .pin_mut()
            .SetDistanceToLeadingVehicle(actor.cxx_actor().as_ref().unwrap(), distance);
    }

    pub fn set_percentage_ignore_walkers<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner
            .pin_mut()
            .SetPercentageIgnoreWalkers(actor.cxx_actor().as_ref().unwrap(), percentage);
    }

    pub fn set_percentage_ignore_vehicles<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner
            .pin_mut()
            .SetPercentageIgnoreVehicles(actor.cxx_actor().as_ref().unwrap(), percentage);
    }

    pub fn set_percentage_running_light<A: ActorBase>(&mut self, actor: &A, percentage: f32) {
        self.inner
            .pin_mut()
            .SetPercentageRunningLight(actor.cxx_actor().as_ref().unwrap(), percentage);
    }

    pub fn set_synchronous_mode(&mut self, yes: bool) {
        self.inner.pin_mut().SetSynchronousMode(yes);
    }

    pub fn set_synchronous_mode_time_out(&mut self, time: Duration) {
        self.inner
            .pin_mut()
            .SetSynchronousModeTimeOutInMiliSecond(time.as_secs_f64() * 1000.0);
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
        self.inner
            .pin_mut()
            .SetKeepRightPercentage(actor.cxx_actor().as_ref().unwrap(), percentage);
    }

    pub fn set_random_left_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) {
        self.inner
            .pin_mut()
            .SetRandomLeftLaneChangePercentage(actor.cxx_actor().as_ref().unwrap(), percentage);
    }

    pub fn set_random_right_lane_change_percentage<A: ActorBase>(
        &mut self,
        actor: &A,
        percentage: f32,
    ) {
        self.inner
            .pin_mut()
            .SetRandomRightLaneChangePercentage(actor.cxx_actor().as_ref().unwrap(), percentage);
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
        PrivateAction::from_cxx(action).unwrap().to_pair()
    }

    pub fn action_buffer(&mut self, actor_id: ActorId) -> ActionBuffer {
        let ptr = self
            .inner
            .pin_mut()
            .GetActionBuffer(&actor_id)
            .within_unique_ptr();
        ActionBuffer::from_cxx(ptr).unwrap()
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
