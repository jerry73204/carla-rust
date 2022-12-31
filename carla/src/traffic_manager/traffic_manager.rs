use std::time::Duration;

use carla_sys::carla_rust::traffic_manager::FfiTrafficManager;
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

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

    pub fn set_global_percentage_speed_difference(&mut self, percentage: f32) {
        self.inner
            .pin_mut()
            .SetGlobalPercentageSpeedDifference(percentage);
    }

    pub fn set_global_lane_offset(&mut self, offset: f32) {
        self.inner.pin_mut().SetGlobalLaneOffset(offset);
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

    pub fn set_random_device_seed(&mut self, seed: u64) {
        self.inner.pin_mut().SetRandomDeviceSeed(seed);
    }

    pub fn shutdown(mut self) {
        self.inner.pin_mut().ShutDown();
        self.inner = UniquePtr::null();
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
