//! Regression tests for two defects that made generated types hard to use.

/// `TrafficLightState` derived Clone/Hash/PartialEq/Eq but not Debug, so it could not be
/// used with `assert_eq!` or `{:?}` at all.
#[test]
fn traffic_light_state_is_debug() {
    use carla::rpc::TrafficLightState;
    assert_eq!(
        format!("{:?}", TrafficLightState::Green),
        "TrafficLightState::Green"
    );
    assert_eq!(TrafficLightState::Red, TrafficLightState::Red);
}

/// `FfiLocation` has a destructor, so it is not `Copy` and cannot be moved out of a struct
/// behind a shared reference -- which made the results of `ground_projection`, `cast_ray`
/// and `project_point` unreadable from safe Rust.
#[test]
fn labelled_point_location_is_readable_behind_a_reference() {
    fn read(p: &carla::rpc::LabelledPoint) -> carla::geom::Location {
        carla::geom::Location::from_ffi_ref(&p.location)
    }
    let _ = read;
}
