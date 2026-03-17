use crate::{
    geom::{FfiVector2D, Location, Vector2D, Vector3D},
    utils::IteratorExt,
};
use autocxx::prelude::*;
use carla_sys::carla_rust::rpc::FfiWheelPhysicsControl;

/// Wheel physics control parameters for CARLA 0.10.0.
///
/// This type contains all the physical parameters that define how a wheel behaves,
/// including tire physics, suspension, braking, and steering characteristics.
#[derive(Debug, Clone)]
pub struct WheelPhysicsControl {
    pub axle_type: u8,
    pub offset: Vector3D,
    pub wheel_radius: f32,
    pub wheel_width: f32,
    pub wheel_mass: f32,
    pub cornering_stiffness: f32,
    pub friction_force_multiplier: f32,
    pub side_slip_modifier: f32,
    pub slip_threshold: f32,
    pub skid_threshold: f32,
    pub max_steer_angle: f32,
    pub affected_by_steering: bool,
    pub affected_by_brake: bool,
    pub affected_by_handbrake: bool,
    pub affected_by_engine: bool,
    pub abs_enabled: bool,
    pub traction_control_enabled: bool,
    pub max_wheelspin_rotation: f32,
    pub external_torque_combine_method: u8,
    pub lateral_slip_graph: Vec<Vector2D>,
    pub suspension_axis: Vector3D,
    pub suspension_force_offset: Vector3D,
    pub suspension_max_raise: f32,
    pub suspension_max_drop: f32,
    pub suspension_damping_ratio: f32,
    pub wheel_load_ratio: f32,
    pub spring_rate: f32,
    pub spring_preload: f32,
    pub suspension_smoothing: i32,
    pub rollbar_scaling: f32,
    pub sweep_shape: u8,
    pub sweep_type: u8,
    pub max_brake_torque: f32,
    pub max_hand_brake_torque: f32,
    pub wheel_index: i32,
    pub location: Location,
    pub old_location: Location,
    pub velocity: Location,
}

impl WheelPhysicsControl {
    pub fn from_cxx(from: &FfiWheelPhysicsControl) -> Self {
        Self {
            axle_type: from.get_axle_type(),
            offset: Vector3D::from_ffi(from.get_offset()),
            wheel_radius: from.get_wheel_radius(),
            wheel_width: from.get_wheel_width(),
            wheel_mass: from.get_wheel_mass(),
            cornering_stiffness: from.get_cornering_stiffness(),
            friction_force_multiplier: from.get_friction_force_multiplier(),
            side_slip_modifier: from.get_side_slip_modifier(),
            slip_threshold: from.get_slip_threshold(),
            skid_threshold: from.get_skid_threshold(),
            max_steer_angle: from.get_max_steer_angle(),
            affected_by_steering: from.get_affected_by_steering(),
            affected_by_brake: from.get_affected_by_brake(),
            affected_by_handbrake: from.get_affected_by_handbrake(),
            affected_by_engine: from.get_affected_by_engine(),
            abs_enabled: from.get_abs_enabled(),
            traction_control_enabled: from.get_traction_control_enabled(),
            max_wheelspin_rotation: from.get_max_wheelspin_rotation(),
            external_torque_combine_method: from.get_external_torque_combine_method(),
            lateral_slip_graph: from
                .get_lateral_slip_graph()
                .iter()
                .map(|v| unsafe {
                    Vector2D::from_ffi(std::mem::transmute::<
                        carla_sys::carla::geom::Vector2D,
                        FfiVector2D,
                    >(v.clone()))
                })
                .collect(),
            suspension_axis: Vector3D::from_ffi(from.get_suspension_axis()),
            suspension_force_offset: Vector3D::from_ffi(from.get_suspension_force_offset()),
            suspension_max_raise: from.get_suspension_max_raise(),
            suspension_max_drop: from.get_suspension_max_drop(),
            suspension_damping_ratio: from.get_suspension_damping_ratio(),
            wheel_load_ratio: from.get_wheel_load_ratio(),
            spring_rate: from.get_spring_rate(),
            spring_preload: from.get_spring_preload(),
            suspension_smoothing: from.get_suspension_smoothing(),
            rollbar_scaling: from.get_rollbar_scaling(),
            sweep_shape: from.get_sweep_shape(),
            sweep_type: from.get_sweep_type(),
            max_brake_torque: from.get_max_brake_torque(),
            max_hand_brake_torque: from.get_max_hand_brake_torque(),
            wheel_index: from.get_wheel_index(),
            location: Location::from_ffi(from.get_location()),
            old_location: Location::from_ffi(from.get_old_location()),
            velocity: Location::from_ffi(from.get_velocity()),
        }
    }

    pub fn to_cxx(&self) -> UniquePtr<FfiWheelPhysicsControl> {
        let mut wheel = FfiWheelPhysicsControl::new().within_unique_ptr();
        wheel.pin_mut().set_axle_type(self.axle_type);
        wheel.pin_mut().set_offset(&self.offset.into_ffi());
        wheel.pin_mut().set_wheel_radius(self.wheel_radius);
        wheel.pin_mut().set_wheel_width(self.wheel_width);
        wheel.pin_mut().set_wheel_mass(self.wheel_mass);
        wheel
            .pin_mut()
            .set_cornering_stiffness(self.cornering_stiffness);
        wheel
            .pin_mut()
            .set_friction_force_multiplier(self.friction_force_multiplier);
        wheel
            .pin_mut()
            .set_side_slip_modifier(self.side_slip_modifier);
        wheel.pin_mut().set_slip_threshold(self.slip_threshold);
        wheel.pin_mut().set_skid_threshold(self.skid_threshold);
        wheel.pin_mut().set_max_steer_angle(self.max_steer_angle);
        wheel
            .pin_mut()
            .set_affected_by_steering(self.affected_by_steering);
        wheel
            .pin_mut()
            .set_affected_by_brake(self.affected_by_brake);
        wheel
            .pin_mut()
            .set_affected_by_handbrake(self.affected_by_handbrake);
        wheel
            .pin_mut()
            .set_affected_by_engine(self.affected_by_engine);
        wheel.pin_mut().set_abs_enabled(self.abs_enabled);
        wheel
            .pin_mut()
            .set_traction_control_enabled(self.traction_control_enabled);
        wheel
            .pin_mut()
            .set_max_wheelspin_rotation(self.max_wheelspin_rotation);
        wheel
            .pin_mut()
            .set_external_torque_combine_method(self.external_torque_combine_method);

        // Convert lateral_slip_graph
        let lateral_slip = self
            .lateral_slip_graph
            .iter()
            .map(|v| unsafe {
                std::mem::transmute::<FfiVector2D, carla_sys::carla::geom::Vector2D>(v.into_ffi())
            })
            .collect_cxx_vector();
        wheel.pin_mut().set_lateral_slip_graph(&lateral_slip);

        wheel
            .pin_mut()
            .set_suspension_axis(&self.suspension_axis.into_ffi());
        wheel
            .pin_mut()
            .set_suspension_force_offset(&self.suspension_force_offset.into_ffi());
        wheel
            .pin_mut()
            .set_suspension_max_raise(self.suspension_max_raise);
        wheel
            .pin_mut()
            .set_suspension_max_drop(self.suspension_max_drop);
        wheel
            .pin_mut()
            .set_suspension_damping_ratio(self.suspension_damping_ratio);
        wheel.pin_mut().set_wheel_load_ratio(self.wheel_load_ratio);
        wheel.pin_mut().set_spring_rate(self.spring_rate);
        wheel.pin_mut().set_spring_preload(self.spring_preload);
        wheel
            .pin_mut()
            .set_suspension_smoothing(self.suspension_smoothing);
        wheel.pin_mut().set_rollbar_scaling(self.rollbar_scaling);
        wheel.pin_mut().set_sweep_shape(self.sweep_shape);
        wheel.pin_mut().set_sweep_type(self.sweep_type);
        wheel.pin_mut().set_max_brake_torque(self.max_brake_torque);
        wheel
            .pin_mut()
            .set_max_hand_brake_torque(self.max_hand_brake_torque);
        wheel.pin_mut().set_wheel_index(self.wheel_index);
        wheel.pin_mut().set_location(&self.location.into_ffi());
        wheel
            .pin_mut()
            .set_old_location(&self.old_location.into_ffi());
        wheel.pin_mut().set_velocity(&self.velocity.into_ffi());
        wheel
    }
}
