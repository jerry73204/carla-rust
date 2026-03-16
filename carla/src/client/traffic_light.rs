use super::{Actor, ActorBase, BoundingBoxList, TrafficLightList, WaypointList};
use crate::{error::ffi::with_ffi_error, geom::BoundingBox, road::SignId, rpc::TrafficLightState};
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
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use carla::{
///     client::{ActorBase, Client},
///     rpc::TrafficLightState,
/// };
///
/// let client = Client::connect("localhost", 2000, None)?;
/// let world = client.world()?;
///
/// # let bp_lib = world.blueprint_library()?;
/// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// # let spawn_points = world.map()?.recommended_spawn_points();
/// # let vehicle = world.spawn_actor(&vehicle_bp, spawn_points.get(0).unwrap())?;
/// # let vehicle: carla::client::Vehicle = vehicle.try_into().unwrap();
/// # let waypoint = world.map()?.waypoint_at(&vehicle.location()?)?.unwrap();
/// // Get traffic lights affecting a waypoint
/// let traffic_lights = world.traffic_lights_from_waypoint(&waypoint, 50.0)?;
///
/// if let Some(light) = traffic_lights.get(0) {
///     println!("Current state: {:?}", light.state()?);
///     println!("Green time: {}s", light.green_time()?);
///
///     // Change the traffic light state
///     light.set_state(TrafficLightState::Green)?;
///     light.freeze(true)?; // Freeze in this state
/// }
/// # Ok(())
/// # }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub struct TrafficLight {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiTrafficLight>,
}

impl TrafficLight {
    /// Returns the OpenDRIVE sign ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_opendrive_id)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_opendrive_id)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_opendrive_id)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn sign_id(&self) -> crate::Result<SignId> {
        let id = with_ffi_error("sign_id", |e| self.inner.GetSignId(e))?;
        Ok(id.to_string())
    }

    /// Returns the trigger volume bounding box.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_trigger_volume](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_trigger_volume)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_trigger_volume](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_trigger_volume)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_trigger_volume](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_trigger_volume)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// The trigger volume is the area where vehicles are affected by this traffic light.
    pub fn trigger_volume(&self) -> crate::Result<BoundingBox> {
        let bbox = with_ffi_error("trigger_volume", |e| self.inner.GetTriggerVolume(e))?;
        Ok(BoundingBox::from_native(&bbox))
    }

    /// Returns the current state of the traffic light.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_state)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_state](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_state)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_state](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_state)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn state(&self) -> crate::Result<TrafficLightState> {
        with_ffi_error("state", |e| self.inner.GetState(e))
    }

    /// Sets the state of the traffic light.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.set_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_state)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.set_state](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.set_state)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.set_state](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.set_state)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_state(&self, state: TrafficLightState) -> crate::Result<()> {
        with_ffi_error("set_state", |e| self.inner.SetState(state, e))
    }

    /// Returns the duration of the green light phase in seconds.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_green_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_green_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_green_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_green_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_green_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_green_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn green_time(&self) -> crate::Result<f32> {
        with_ffi_error("green_time", |e| self.inner.GetGreenTime(e))
    }

    /// Returns the duration of the yellow light phase in seconds.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_yellow_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_yellow_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_yellow_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_yellow_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_yellow_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_yellow_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn yellow_time(&self) -> crate::Result<f32> {
        with_ffi_error("yellow_time", |e| self.inner.GetYellowTime(e))
    }

    /// Returns the duration of the red light phase in seconds.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_red_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_red_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_red_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_red_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_red_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_red_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn red_time(&self) -> crate::Result<f32> {
        with_ffi_error("red_time", |e| self.inner.GetRedTime(e))
    }

    /// Sets the duration of the green light phase.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.set_green_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_green_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.set_green_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.set_green_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.set_green_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.set_green_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Arguments
    /// * `time` - Duration in seconds
    pub fn set_green_time(&self, time: f32) -> crate::Result<()> {
        with_ffi_error("set_green_time", |e| self.inner.SetGreenTime(time, e))
    }

    /// Sets the duration of the yellow light phase.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.set_yellow_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_yellow_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.set_yellow_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.set_yellow_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.set_yellow_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.set_yellow_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Arguments
    /// * `time` - Duration in seconds
    pub fn set_yellow_time(&self, time: f32) -> crate::Result<()> {
        with_ffi_error("set_yellow_time", |e| self.inner.SetYellowTime(time, e))
    }

    /// Sets the duration of the red light phase.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.set_red_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.set_red_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.set_red_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.set_red_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.set_red_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.set_red_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Arguments
    /// * `time` - Duration in seconds
    pub fn set_red_time(&self, time: f32) -> crate::Result<()> {
        with_ffi_error("set_red_time", |e| self.inner.SetRedTime(time, e))
    }

    /// Returns the time elapsed since the current phase started.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_elapsed_time](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_elapsed_time)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_elapsed_time](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_elapsed_time)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_elapsed_time](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_elapsed_time)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn elapsed_time(&self) -> crate::Result<f32> {
        with_ffi_error("elapsed_time", |e| self.inner.GetElapsedTime(e))
    }

    /// Freezes or unfreezes the traffic light.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.freeze](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.freeze)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.freeze](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.freeze)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.freeze](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.freeze)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// When frozen, the traffic light will not change state automatically.
    ///
    /// # Arguments
    /// * `freeze` - `true` to freeze, `false` to unfreeze
    pub fn freeze(&self, freeze: bool) -> crate::Result<()> {
        with_ffi_error("freeze", |e| self.inner.Freeze(freeze, e))
    }

    /// Returns whether the traffic light is frozen.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.is_frozen](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.is_frozen)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.is_frozen](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.is_frozen)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.is_frozen](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.is_frozen)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn is_frozen(&self) -> crate::Result<bool> {
        with_ffi_error("is_frozen", |e| self.inner.IsFrozen(e))
    }

    /// Returns the index of the pole in the traffic light group.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_pole_index](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_pole_index)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_pole_index](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_pole_index)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_pole_index](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_pole_index)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn pole_index(&self) -> crate::Result<u32> {
        with_ffi_error("pole_index", |e| self.inner.GetPoleIndex(e))
    }

    /// Returns all traffic lights in the same synchronization group.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_group_traffic_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_group_traffic_lights)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_group_traffic_lights](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_group_traffic_lights)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_group_traffic_lights](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_group_traffic_lights)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn group_traffic_lights(&self) -> crate::Result<TrafficLightList> {
        let ptr = with_ffi_error("group_traffic_lights", |e| {
            self.inner.GetGroupTrafficLights(e)
        })?;
        Ok(unsafe { TrafficLightList::from_cxx(ptr).unwrap_unchecked() })
    }

    /// Resets the state of the entire traffic light group.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.reset_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.reset_group)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.reset_group](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.reset_group)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.reset_group](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.reset_group)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// This affects all traffic lights in the synchronized group,
    /// resetting their timing and state.
    pub fn reset_group(&self) -> crate::Result<()> {
        with_ffi_error("reset_group", |e| self.inner.ResetGroup(e))
    }

    /// Returns waypoints affected by this traffic light.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_affected_lane_waypoints](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_affected_lane_waypoints)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_affected_lane_waypoints](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_affected_lane_waypoints)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_affected_lane_waypoints](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_affected_lane_waypoints)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// Returns waypoints on lanes controlled by this traffic light.
    pub fn affected_lane_waypoints(&self) -> crate::Result<WaypointList> {
        let ptr = with_ffi_error("affected_lane_waypoints", |e| {
            self.inner.GetAffectedLaneWaypoints(e)
        })?;
        Ok(unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() })
    }

    /// Returns the bounding boxes of the light bulbs.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_light_boxes](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_light_boxes)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_light_boxes](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_light_boxes)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_light_boxes](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_light_boxes)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn light_boxes(&self) -> crate::Result<BoundingBoxList> {
        let ptr = with_ffi_error("light_boxes", |e| self.inner.GetLightBoxes(e))?;
        Ok(unsafe { BoundingBoxList::from_cxx(ptr).unwrap_unchecked() })
    }

    /// Returns the OpenDRIVE ID of this traffic light.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_opendrive_id)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_opendrive_id)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_opendrive_id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_opendrive_id)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn opendrive_id(&self) -> crate::Result<SignId> {
        let id = with_ffi_error("opendrive_id", |e| self.inner.GetOpenDRIVEID(e))?;
        Ok(id.to_string())
    }

    /// Returns waypoints where vehicles should stop for this traffic light.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.TrafficLight.get_stop_waypoints](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight.get_stop_waypoints)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.TrafficLight.get_stop_waypoints](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.TrafficLight.get_stop_waypoints)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.TrafficLight.get_stop_waypoints](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.TrafficLight.get_stop_waypoints)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn stop_waypoints(&self) -> crate::Result<WaypointList> {
        let ptr = with_ffi_error("stop_waypoints", |e| self.inner.GetStopWaypoints(e))?;
        Ok(unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() })
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
