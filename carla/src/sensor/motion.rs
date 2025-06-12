use crate::sensor::data::{GnssMeasurement, ImuMeasurement, LidarMeasurement};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, UnitQuaternion,
    Vector3,
};
use std::{collections::VecDeque, ptr, time::Duration};

/// Advanced motion analysis and sensor fusion capabilities.
/// Provides multi-sensor fusion, motion estimation, and coordinate transformations.
pub struct MotionProcessor {
    inner: *mut carla_motion_processor_t,
}

impl MotionProcessor {
    /// Create a new motion processor.
    pub fn new() -> Result<Self> {
        let processor_ptr = unsafe { carla_motion_processor_create() };
        if processor_ptr.is_null() {
            return Err(anyhow!("Failed to create motion processor"));
        }
        Ok(Self {
            inner: processor_ptr,
        })
    }

    /// Process IMU data for motion estimation.
    ///
    /// # Arguments
    /// * `imu_data` - IMU measurement data
    /// * `timestamp` - Timestamp of the measurement
    ///
    /// # Returns
    /// Updated motion state
    pub fn process_imu(
        &mut self,
        imu_data: &ImuMeasurement,
        timestamp: f64,
    ) -> Result<MotionState> {
        let accel = imu_data.accelerometer();
        let gyro = imu_data.gyroscope();
        let compass = imu_data.compass();

        let c_imu = carla_imu_data_t {
            accelerometer: carla_vector3d_t {
                x: accel.x,
                y: accel.y,
                z: accel.z,
            },
            gyroscope: carla_vector3d_t {
                x: gyro.x,
                y: gyro.y,
                z: gyro.z,
            },
            compass: compass,
        };

        let mut c_state = carla_motion_state_t {
            position: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: carla_quaternion_t {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            timestamp: 0.0,
        };

        let error =
            unsafe { carla_motion_process_imu(self.inner, &c_imu, timestamp, &mut c_state) };

        if error != 0 {
            return Err(anyhow!("Failed to process IMU data"));
        }

        Ok(MotionState::from_c_state(c_state))
    }

    /// Process GNSS data for position correction.
    ///
    /// # Arguments
    /// * `gnss_data` - GNSS measurement data
    /// * `timestamp` - Timestamp of the measurement
    ///
    /// # Returns
    /// Updated motion state with position correction
    pub fn process_gnss(
        &mut self,
        gnss_data: &GnssMeasurement,
        timestamp: f64,
    ) -> Result<MotionState> {
        let geo_location = gnss_data.geo_location();

        let c_gnss = carla_gnss_data_t {
            latitude: gnss_data.latitude(),
            longitude: gnss_data.longitude(),
            altitude: gnss_data.altitude(),
        };

        let mut c_state = carla_motion_state_t {
            position: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: carla_quaternion_t {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            timestamp: 0.0,
        };

        let error =
            unsafe { carla_motion_process_gnss(self.inner, &c_gnss, timestamp, &mut c_state) };

        if error != 0 {
            return Err(anyhow!("Failed to process GNSS data"));
        }

        Ok(MotionState::from_c_state(c_state))
    }

    /// Fuse multiple sensor measurements for improved motion estimation.
    ///
    /// # Arguments
    /// * `sensor_data` - Collection of sensor measurements
    /// * `timestamp` - Timestamp of the fusion
    ///
    /// # Returns
    /// Fused motion state
    pub fn fuse_sensors(
        &mut self,
        sensor_data: &SensorFusionData,
        timestamp: f64,
    ) -> Result<MotionState> {
        let c_fusion_data = sensor_data.to_c_fusion_data();
        let mut c_state = carla_motion_state_t {
            position: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: carla_quaternion_t {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            timestamp: 0.0,
        };

        let error = unsafe {
            carla_motion_fuse_sensors(self.inner, &c_fusion_data, timestamp, &mut c_state)
        };

        if error != 0 {
            return Err(anyhow!("Failed to fuse sensor data"));
        }

        Ok(MotionState::from_c_state(c_state))
    }

    /// Transform coordinates between different reference frames.
    ///
    /// # Arguments
    /// * `point` - Point to transform
    /// * `from_frame` - Source coordinate frame
    /// * `to_frame` - Target coordinate frame
    ///
    /// # Returns
    /// Transformed point
    pub fn transform_coordinates(
        &self,
        point: &Point3<f32>,
        from_frame: &CoordinateFrame,
        to_frame: &CoordinateFrame,
    ) -> Result<Point3<f32>> {
        let c_point = carla_vector3d_t {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let c_from_frame = from_frame.to_c_frame();
        let c_to_frame = to_frame.to_c_frame();
        let mut c_result = carla_vector3d_t {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };

        let error = unsafe {
            carla_motion_transform_coordinates(
                self.inner,
                &c_point,
                &c_from_frame,
                &c_to_frame,
                &mut c_result,
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to transform coordinates"));
        }

        Ok(Point3::new(c_result.x, c_result.y, c_result.z))
    }

    /// Calculate motion between two poses.
    ///
    /// # Arguments
    /// * `pose1` - Initial pose
    /// * `pose2` - Final pose
    /// * `time_delta` - Time difference between poses
    ///
    /// # Returns
    /// Motion estimation between poses
    pub fn calculate_motion(
        &self,
        pose1: &Isometry3<f32>,
        pose2: &Isometry3<f32>,
        time_delta: Duration,
    ) -> Result<MotionEstimation> {
        let c_pose1 = pose_to_c_transform(pose1);
        let c_pose2 = pose_to_c_transform(pose2);
        let dt = time_delta.as_secs_f64();

        let mut c_motion = carla_motion_estimation_t {
            linear_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            linear_acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            displacement: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: carla_quaternion_t {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };

        let error = unsafe {
            carla_motion_calculate_motion(self.inner, &c_pose1, &c_pose2, dt, &mut c_motion)
        };

        if error != 0 {
            return Err(anyhow!("Failed to calculate motion"));
        }

        Ok(MotionEstimation::from_c_motion(c_motion))
    }

    /// Track object motion over time.
    ///
    /// # Arguments
    /// * `object_id` - Unique identifier for the object
    /// * `pose` - Current pose of the object
    /// * `timestamp` - Timestamp of the observation
    ///
    /// # Returns
    /// Motion tracking result
    pub fn track_object_motion(
        &mut self,
        object_id: u32,
        pose: &Isometry3<f32>,
        timestamp: f64,
    ) -> Result<ObjectMotionTrack> {
        let c_transform = pose_to_c_transform(pose);
        let mut c_track = carla_object_motion_track_t {
            object_id: 0,
            current_pose: carla_transform_t {
                location: carla_vector3d_t {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: carla_quaternion_t {
                    w: 1.0,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
            velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            confidence: 0.0,
            track_length: 0,
            last_seen: 0.0,
        };

        let error = unsafe {
            carla_motion_track_object(self.inner, object_id, &c_transform, timestamp, &mut c_track)
        };

        if error != 0 {
            return Err(anyhow!("Failed to track object motion"));
        }

        Ok(ObjectMotionTrack::from_c_track(c_track))
    }

    /// Predict future motion state.
    ///
    /// # Arguments
    /// * `current_state` - Current motion state
    /// * `prediction_time` - Time to predict forward
    ///
    /// # Returns
    /// Predicted future motion state
    pub fn predict_motion(
        &self,
        current_state: &MotionState,
        prediction_time: Duration,
    ) -> Result<MotionState> {
        let c_current = current_state.to_c_state();
        let dt = prediction_time.as_secs_f64();
        let mut c_predicted = carla_motion_state_t {
            position: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: carla_quaternion_t {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_velocity: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            timestamp: 0.0,
        };

        let error = unsafe { carla_motion_predict(self.inner, &c_current, dt, &mut c_predicted) };

        if error != 0 {
            return Err(anyhow!("Failed to predict motion"));
        }

        Ok(MotionState::from_c_state(c_predicted))
    }

    /// Set motion processing parameters.
    ///
    /// # Arguments
    /// * `params` - Motion processing parameters
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_parameters(&mut self, params: &MotionProcessingParams) -> Result<()> {
        let c_params = params.to_c_params();
        let error = unsafe { carla_motion_set_parameters(self.inner, &c_params) };

        if error != 0 {
            return Err(anyhow!("Failed to set motion processing parameters"));
        }

        Ok(())
    }
}

impl Drop for MotionProcessor {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_motion_processor_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

/// Complete motion state of an object.
#[derive(Clone, Debug)]
pub struct MotionState {
    /// Position in 3D space
    pub position: Vector3<f32>,
    /// Linear velocity
    pub velocity: Vector3<f32>,
    /// Linear acceleration
    pub acceleration: Vector3<f32>,
    /// Orientation as quaternion
    pub orientation: UnitQuaternion<f32>,
    /// Angular velocity
    pub angular_velocity: Vector3<f32>,
    /// Timestamp of the state
    pub timestamp: f64,
}

impl MotionState {
    pub(crate) fn from_c_state(state: carla_motion_state_t) -> Self {
        Self {
            position: Vector3::new(state.position.x, state.position.y, state.position.z),
            velocity: Vector3::new(state.velocity.x, state.velocity.y, state.velocity.z),
            acceleration: Vector3::new(
                state.acceleration.x,
                state.acceleration.y,
                state.acceleration.z,
            ),
            orientation: UnitQuaternion::from_quaternion(Quaternion::new(
                state.orientation.w,
                state.orientation.x,
                state.orientation.y,
                state.orientation.z,
            )),
            angular_velocity: Vector3::new(
                state.angular_velocity.x,
                state.angular_velocity.y,
                state.angular_velocity.z,
            ),
            timestamp: state.timestamp,
        }
    }

    pub(crate) fn to_c_state(&self) -> carla_motion_state_t {
        let quat = self.orientation.quaternion();
        carla_motion_state_t {
            position: carla_vector3d_t {
                x: self.position.x,
                y: self.position.y,
                z: self.position.z,
            },
            velocity: carla_vector3d_t {
                x: self.velocity.x,
                y: self.velocity.y,
                z: self.velocity.z,
            },
            acceleration: carla_vector3d_t {
                x: self.acceleration.x,
                y: self.acceleration.y,
                z: self.acceleration.z,
            },
            orientation: carla_quaternion_t {
                w: quat.w,
                x: quat.i,
                y: quat.j,
                z: quat.k,
            },
            angular_velocity: carla_vector3d_t {
                x: self.angular_velocity.x,
                y: self.angular_velocity.y,
                z: self.angular_velocity.z,
            },
            timestamp: self.timestamp,
        }
    }

    /// Get the pose (position and orientation) as an Isometry3.
    pub fn pose(&self) -> Isometry3<f32> {
        Isometry3::from_parts(Translation3::from(self.position), self.orientation)
    }

    /// Get the speed (magnitude of velocity).
    pub fn speed(&self) -> f32 {
        self.velocity.magnitude()
    }
}

/// Data for multi-sensor fusion.
#[derive(Clone, Debug)]
pub struct SensorFusionData {
    /// IMU measurements
    pub imu_data: Option<ImuMeasurement>,
    /// GNSS measurements
    pub gnss_data: Option<GnssMeasurement>,
    /// LiDAR measurements (for SLAM)
    pub lidar_data: Option<LidarMeasurement>,
    /// Visual odometry data
    pub visual_odometry: Option<VisualOdometry>,
    /// Wheel odometry data
    pub wheel_odometry: Option<WheelOdometry>,
}

impl SensorFusionData {
    pub(crate) fn to_c_fusion_data(&self) -> carla_sensor_fusion_data_t {
        // This is a simplified implementation
        // Real implementation would convert all sensor data
        carla_sensor_fusion_data_t {
            has_imu: self.imu_data.is_some(),
            has_gnss: self.gnss_data.is_some(),
            has_lidar: self.lidar_data.is_some(),
            has_visual_odometry: self.visual_odometry.is_some(),
            has_wheel_odometry: self.wheel_odometry.is_some(),
            // TODO: Add actual sensor data conversion
        }
    }
}

/// Motion estimation between two poses.
#[derive(Clone, Debug)]
pub struct MotionEstimation {
    /// Linear velocity
    pub linear_velocity: Vector3<f32>,
    /// Angular velocity
    pub angular_velocity: Vector3<f32>,
    /// Linear acceleration
    pub linear_acceleration: Vector3<f32>,
    /// Angular acceleration
    pub angular_acceleration: Vector3<f32>,
    /// Displacement vector
    pub displacement: Vector3<f32>,
    /// Rotation quaternion
    pub rotation: UnitQuaternion<f32>,
}

impl MotionEstimation {
    pub(crate) fn from_c_motion(motion: carla_motion_estimation_t) -> Self {
        Self {
            linear_velocity: Vector3::new(
                motion.linear_velocity.x,
                motion.linear_velocity.y,
                motion.linear_velocity.z,
            ),
            angular_velocity: Vector3::new(
                motion.angular_velocity.x,
                motion.angular_velocity.y,
                motion.angular_velocity.z,
            ),
            linear_acceleration: Vector3::new(
                motion.linear_acceleration.x,
                motion.linear_acceleration.y,
                motion.linear_acceleration.z,
            ),
            angular_acceleration: Vector3::new(
                motion.angular_acceleration.x,
                motion.angular_acceleration.y,
                motion.angular_acceleration.z,
            ),
            displacement: Vector3::new(
                motion.displacement.x,
                motion.displacement.y,
                motion.displacement.z,
            ),
            rotation: UnitQuaternion::from_quaternion(Quaternion::new(
                motion.rotation.w,
                motion.rotation.x,
                motion.rotation.y,
                motion.rotation.z,
            )),
        }
    }
}

/// Object motion tracking result.
#[derive(Clone, Debug)]
pub struct ObjectMotionTrack {
    /// Object identifier
    pub object_id: u32,
    /// Current pose
    pub current_pose: Isometry3<f32>,
    /// Current velocity
    pub velocity: Vector3<f32>,
    /// Current acceleration
    pub acceleration: Vector3<f32>,
    /// Angular velocity
    pub angular_velocity: Vector3<f32>,
    /// Tracking confidence (0.0 to 1.0)
    pub confidence: f32,
    /// Number of observations in track
    pub track_length: u32,
    /// Last seen timestamp
    pub last_seen: f64,
}

impl ObjectMotionTrack {
    pub(crate) fn from_c_track(track: carla_object_motion_track_t) -> Self {
        let translation = Translation3::new(
            track.current_pose.location.x,
            track.current_pose.location.y,
            track.current_pose.location.z,
        );
        let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
            track.current_pose.rotation.w,
            track.current_pose.rotation.x,
            track.current_pose.rotation.y,
            track.current_pose.rotation.z,
        ));

        Self {
            object_id: track.object_id,
            current_pose: Isometry3::from_parts(translation, rotation),
            velocity: Vector3::new(track.velocity.x, track.velocity.y, track.velocity.z),
            acceleration: Vector3::new(
                track.acceleration.x,
                track.acceleration.y,
                track.acceleration.z,
            ),
            angular_velocity: Vector3::new(
                track.angular_velocity.x,
                track.angular_velocity.y,
                track.angular_velocity.z,
            ),
            confidence: track.confidence,
            track_length: track.track_length,
            last_seen: track.last_seen,
        }
    }
}

/// Coordinate reference frame definition.
#[derive(Clone, Debug)]
pub struct CoordinateFrame {
    /// Frame origin
    pub origin: Point3<f32>,
    /// Frame orientation
    pub orientation: UnitQuaternion<f32>,
    /// Frame name/identifier
    pub name: String,
}

impl CoordinateFrame {
    pub(crate) fn to_c_frame(&self) -> carla_coordinate_frame_t {
        let quat = self.orientation.quaternion();
        carla_coordinate_frame_t {
            origin: carla_vector3d_t {
                x: self.origin.x,
                y: self.origin.y,
                z: self.origin.z,
            },
            orientation: carla_quaternion_t {
                w: quat.w,
                x: quat.i,
                y: quat.j,
                z: quat.k,
            },
        }
    }
}

/// Visual odometry data.
#[derive(Clone, Debug)]
pub struct VisualOdometry {
    /// Estimated pose change
    pub pose_delta: Isometry3<f32>,
    /// Confidence of the estimate
    pub confidence: f32,
    /// Number of features tracked
    pub feature_count: u32,
}

/// Wheel odometry data.
#[derive(Clone, Debug)]
pub struct WheelOdometry {
    /// Distance traveled
    pub distance: f32,
    /// Heading change
    pub heading_delta: f32,
    /// Left wheel distance
    pub left_wheel_distance: f32,
    /// Right wheel distance
    pub right_wheel_distance: f32,
}

/// Motion processing parameters.
#[derive(Clone, Debug)]
pub struct MotionProcessingParams {
    /// IMU noise standard deviation
    pub imu_noise_std: f32,
    /// GNSS noise standard deviation
    pub gnss_noise_std: f32,
    /// Process noise standard deviation
    pub process_noise_std: f32,
    /// Update rate in Hz
    pub update_rate: f32,
    /// Enable prediction
    pub enable_prediction: bool,
}

impl MotionProcessingParams {
    pub(crate) fn to_c_params(&self) -> carla_motion_processing_params_t {
        carla_motion_processing_params_t {
            imu_noise_std: self.imu_noise_std,
            gnss_noise_std: self.gnss_noise_std,
            process_noise_std: self.process_noise_std,
            update_rate: self.update_rate,
            enable_prediction: self.enable_prediction,
        }
    }
}

impl Default for MotionProcessingParams {
    fn default() -> Self {
        Self {
            imu_noise_std: 0.1,
            gnss_noise_std: 1.0,
            process_noise_std: 0.01,
            update_rate: 100.0,
            enable_prediction: true,
        }
    }
}

/// Utility function to convert Isometry3 to C transform.
fn pose_to_c_transform(pose: &Isometry3<f32>) -> carla_transform_t {
    let translation = pose.translation.vector;
    let quat = pose.rotation.quaternion();

    carla_transform_t {
        location: carla_vector3d_t {
            x: translation.x,
            y: translation.y,
            z: translation.z,
        },
        rotation: carla_quaternion_t {
            w: quat.w,
            x: quat.i,
            y: quat.j,
            z: quat.k,
        },
    }
}

// SAFETY: MotionProcessor wraps a thread-safe C API
unsafe impl Send for MotionProcessor {}
unsafe impl Sync for MotionProcessor {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motion_state_conversion() {
        let state = MotionState {
            position: Vector3::new(1.0, 2.0, 3.0),
            velocity: Vector3::new(0.1, 0.2, 0.3),
            acceleration: Vector3::new(0.01, 0.02, 0.03),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::new(0.001, 0.002, 0.003),
            timestamp: 123.456,
        };

        let c_state = state.to_c_state();
        let converted_back = MotionState::from_c_state(c_state);

        assert!((state.position - converted_back.position).magnitude() < 1e-6);
        assert!((state.velocity - converted_back.velocity).magnitude() < 1e-6);
        assert!((state.timestamp - converted_back.timestamp).abs() < 1e-6);
    }

    #[test]
    fn test_coordinate_frame() {
        let frame = CoordinateFrame {
            origin: Point3::new(10.0, 20.0, 30.0),
            orientation: UnitQuaternion::identity(),
            name: "test_frame".to_string(),
        };

        let c_frame = frame.to_c_frame();
        assert_eq!(c_frame.origin.x, 10.0);
        assert_eq!(c_frame.origin.y, 20.0);
        assert_eq!(c_frame.origin.z, 30.0);
    }

    #[test]
    fn test_motion_processing_params() {
        let params = MotionProcessingParams::default();
        assert!(params.imu_noise_std > 0.0);
        assert!(params.update_rate > 0.0);
        assert!(params.enable_prediction);
    }
}
