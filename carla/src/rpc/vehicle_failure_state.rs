/// Vehicle failure state enum indicating what kind of failure (if any) the vehicle is experiencing.
///
/// Available since CARLA 0.9.14+
///
/// Possible states:
/// - `None`: No failure detected
/// - `Rollover`: Vehicle has rolled over
/// - `Engine`: Engine failure
/// - `TirePuncture`: Tire puncture
///
/// This is a re-export of the C++ enum from CARLA.
/// See [`carla.VehicleFailureState`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.VehicleFailureState) in the Python API.
pub use carla_sys::carla::rpc::VehicleFailureState;
