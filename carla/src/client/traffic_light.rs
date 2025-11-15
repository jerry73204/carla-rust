// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{Actor, ActorBase, BoundingBoxList, TrafficLightList, WaypointList};
use crate::{geom::BoundingBox, road::SignId, rpc::TrafficLightState};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::{FfiActor, FfiTrafficLight};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Represents a traffic light in the simulation.
///
/// Traffic lights control vehicle flow at intersections and can be queried
/// for their current state, timing, and affected lanes. They can also be
/// controlled programmatically to change states and timing.
///
/// Corresponds to [`carla.TrafficLight`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, rpc::TrafficLightState};
///
/// let client = Client::default();
/// let world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let vehicle = world.spawn_actor(&vehicle_bp, spawn_points.get(0).unwrap()).unwrap();
/// # let vehicle: carla::client::Vehicle = vehicle.try_into().unwrap();
/// # let waypoint = world.map().waypoint_at(&vehicle.location()).unwrap();
/// // Get traffic lights affecting a waypoint
/// let traffic_lights = world.traffic_lights_from_waypoint(&waypoint, 50.0);
///
/// if let Some(light) = traffic_lights.get(0) {
///     println!("Current state: {:?}", light.state());
///     println!("Green time: {}s", light.green_time());
///
///     // Change the traffic light state
///     light.set_state(TrafficLightState::Green);
///     light.freeze(true); // Freeze in this state
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub struct TrafficLight {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiTrafficLight>,
}

impl TrafficLight {
    /// Returns the OpenDRIVE sign ID.
    ///
    /// See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_opendrive_id)
    /// in the Python API.
    pub fn sign_id(&self) -> SignId {
        self.inner.GetSignId().to_string()
    }

    /// Returns the trigger volume bounding box.
    ///
    /// See [carla.TrafficLight.get_trigger_volume](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_trigger_volume)
    /// in the Python API.
    ///
    /// The trigger volume is the area where vehicles are affected by this traffic light.
    pub fn trigger_volume(&self) -> BoundingBox {
        BoundingBox::from_native(self.inner.GetTriggerVolume())
    }

    /// Returns the current state of the traffic light.
    ///
    /// See [carla.TrafficLight.get_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_state)
    /// in the Python API.
    pub fn state(&self) -> TrafficLightState {
        self.inner.GetState()
    }

    /// Sets the state of the traffic light.
    ///
    /// See [carla.TrafficLight.set_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_state)
    /// in the Python API.
    pub fn set_state(&self, state: TrafficLightState) {
        self.inner.SetState(state);
    }

    /// Returns the duration of the green light phase in seconds.
    ///
    /// See [carla.TrafficLight.get_green_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_green_time)
    /// in the Python API.
    pub fn green_time(&self) -> f32 {
        self.inner.GetGreenTime()
    }

    /// Returns the duration of the yellow light phase in seconds.
    ///
    /// See [carla.TrafficLight.get_yellow_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_yellow_time)
    /// in the Python API.
    pub fn yellow_time(&self) -> f32 {
        self.inner.GetYellowTime()
    }

    /// Returns the duration of the red light phase in seconds.
    ///
    /// See [carla.TrafficLight.get_red_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_red_time)
    /// in the Python API.
    pub fn red_time(&self) -> f32 {
        self.inner.GetRedTime()
    }

    /// Sets the duration of the green light phase.
    ///
    /// See [carla.TrafficLight.set_green_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_green_time)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `time` - Duration in seconds
    pub fn set_green_time(&self, time: f32) {
        self.inner.SetGreenTime(time)
    }

    /// Sets the duration of the yellow light phase.
    ///
    /// See [carla.TrafficLight.set_yellow_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_yellow_time)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `time` - Duration in seconds
    pub fn set_yellow_time(&self, time: f32) {
        self.inner.SetYellowTime(time)
    }

    /// Sets the duration of the red light phase.
    ///
    /// See [carla.TrafficLight.set_red_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_red_time)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `time` - Duration in seconds
    pub fn set_red_time(&self, time: f32) {
        self.inner.SetRedTime(time)
    }

    /// Returns the time elapsed since the current phase started.
    ///
    /// See [carla.TrafficLight.get_elapsed_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_elapsed_time)
    /// in the Python API.
    pub fn elapsed_time(&self) -> f32 {
        self.inner.GetElapsedTime()
    }

    /// Freezes or unfreezes the traffic light.
    ///
    /// See [carla.TrafficLight.freeze](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.freeze)
    /// in the Python API.
    ///
    /// When frozen, the traffic light will not change state automatically.
    ///
    /// # Arguments
    /// * `freeze` - `true` to freeze, `false` to unfreeze
    pub fn freeze(&self, freeze: bool) {
        self.inner.Freeze(freeze)
    }

    /// Returns whether the traffic light is frozen.
    ///
    /// See [carla.TrafficLight.is_frozen](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.is_frozen)
    /// in the Python API.
    pub fn is_frozen(&self) -> bool {
        self.inner.IsFrozen()
    }

    /// Returns the index of the pole in the traffic light group.
    ///
    /// See [carla.TrafficLight.get_pole_index](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_pole_index)
    /// in the Python API.
    pub fn pole_index(&self) -> u32 {
        self.inner.GetPoleIndex()
    }

    /// Returns all traffic lights in the same synchronization group.
    ///
    /// See [carla.TrafficLight.get_group_traffic_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_group_traffic_lights)
    /// in the Python API.
    pub fn group_traffic_lights(&self) -> TrafficLightList {
        let ptr = self.inner.GetGroupTrafficLights().within_unique_ptr();
        unsafe { TrafficLightList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Resets the state of the entire traffic light group.
    ///
    /// See [carla.TrafficLight.reset_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.reset_group)
    /// in the Python API.
    ///
    /// This affects all traffic lights in the synchronized group,
    /// resetting their timing and state.
    pub fn reset_group(&self) {
        self.inner.ResetGroup();
    }

    /// Returns waypoints affected by this traffic light.
    ///
    /// See [carla.TrafficLight.get_affected_lane_waypoints](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_affected_lane_waypoints)
    /// in the Python API.
    ///
    /// Returns waypoints on lanes controlled by this traffic light.
    pub fn affected_lane_waypoints(&self) -> WaypointList {
        let ptr = self.inner.GetAffectedLaneWaypoints().within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the bounding boxes of the light bulbs.
    ///
    /// See [carla.TrafficLight.get_light_boxes](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_light_boxes)
    /// in the Python API.
    pub fn light_boxes(&self) -> BoundingBoxList {
        let ptr = self.inner.GetLightBoxes().within_unique_ptr();
        unsafe { BoundingBoxList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the OpenDRIVE ID of this traffic light.
    ///
    /// See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_opendrive_id)
    /// in the Python API.
    pub fn opendrive_id(&self) -> SignId {
        self.inner.GetOpenDRIVEID().to_string()
    }

    /// Returns waypoints where vehicles should stop for this traffic light.
    ///
    /// See [carla.TrafficLight.get_stop_waypoints](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_stop_waypoints)
    /// in the Python API.
    pub fn stop_waypoints(&self) -> WaypointList {
        let ptr = self.inner.GetStopWaypoints().within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiTrafficLight>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for TrafficLight {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

impl TryFrom<Actor> for TrafficLight {
    type Error = Actor;

    fn try_from(value: Actor) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_traffic_light();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(TrafficLight: Send, Sync);
