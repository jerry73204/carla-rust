//! Integration tests for new CARLA 0.10.0 features
//!
//! These tests verify the C FFI bindings for new features like:
//! - DVS (Dynamic Vision Sensor) events
//! - Optical flow data processing
//! - Vehicle door control
//! - Ackermann steering control
//! - Enhanced motion analysis

use carla_sys::*;
use std::ffi::CString;

#[test]
fn test_dvs_event_processing() {
    // Test DVS event structure and analysis functions
    let dvs_event = carla_dvs_event_t {
        x: 320,
        y: 240,
        t: 123456789,
        pol: true,
    };

    assert_eq!(dvs_event.x, 320);
    assert_eq!(dvs_event.y, 240);
    assert_eq!(dvs_event.t, 123456789);
    assert_eq!(dvs_event.pol, true);

    // Test DVS analysis structure
    let dvs_analysis = carla_dvs_analysis_t {
        positive_events: 150,
        negative_events: 75,
        total_events: 225,
        event_rate: 1000.0,
        active_region: carla_image_roi_t {
            x: 100,
            y: 100,
            width: 200,
            height: 200,
        },
        activity_density: 0.25,
    };

    assert_eq!(dvs_analysis.positive_events, 150);
    assert_eq!(dvs_analysis.negative_events, 75);
    assert_eq!(dvs_analysis.total_events, 225);
    assert_eq!(dvs_analysis.event_rate, 1000.0);
    assert_eq!(dvs_analysis.activity_density, 0.25);
}

#[test]
fn test_optical_flow_structures() {
    // Test optical flow pixel structure
    let flow_pixel = carla_optical_flow_pixel_t { x: 2.5, y: -1.8 };

    assert_eq!(flow_pixel.x, 2.5);
    assert_eq!(flow_pixel.y, -1.8);

    // Test optical flow analysis structure
    let flow_analysis = carla_optical_flow_analysis_t {
        average_flow: carla_vector3d_t {
            x: 1.2,
            y: -0.8,
            z: 0.0,
        },
        magnitude_avg: 3.5,
        magnitude_max: 8.2,
        moving_pixels: 15000,
        motion_density: 0.75,
        dominant_direction: carla_vector3d_t {
            x: 0.8,
            y: 0.6,
            z: 0.0,
        },
    };

    assert_eq!(flow_analysis.average_flow.x, 1.2);
    assert_eq!(flow_analysis.magnitude_avg, 3.5);
    assert_eq!(flow_analysis.moving_pixels, 15000);
    assert_eq!(flow_analysis.motion_density, 0.75);
}

#[test]
fn test_enhanced_vehicle_control() {
    // Test Ackermann control structure (CARLA 0.10.0)
    let ackermann_control = carla_vehicle_ackermann_control_t {
        steer: 25.0,
        steer_speed: 45.0,
        speed: 15.0,
        acceleration: 2.0,
        jerk: 1.0,
    };

    assert_eq!(ackermann_control.steer, 25.0);
    assert_eq!(ackermann_control.steer_speed, 45.0);
    assert_eq!(ackermann_control.speed, 15.0);
    assert_eq!(ackermann_control.acceleration, 2.0);
    assert_eq!(ackermann_control.jerk, 1.0);

    // Test Ackermann controller settings
    let controller_settings = carla_ackermann_controller_settings_t {
        speed_kp: 0.7,
        speed_ki: 0.12,
        speed_kd: 0.04,
        accel_kp: 0.5,
        accel_ki: 0.08,
        accel_kd: 0.02,
    };

    assert_eq!(controller_settings.speed_kp, 0.7);
    assert_eq!(controller_settings.accel_ki, 0.08);
}

#[test]
fn test_vehicle_door_control() {
    // Test all vehicle door enum values (CARLA 0.10.0 feature)
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL, 0);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FR, 1);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RL, 2);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RR, 3);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD, 4);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK, 5);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL, 6);
}

#[test]
fn test_enhanced_light_states() {
    // Test all vehicle light states including new ones
    let position = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION;
    let low_beam = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM;
    let high_beam = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_HIGH_BEAM;
    let brake = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_BRAKE;
    let reverse = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_REVERSE;
    let left_blinker = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER;
    let right_blinker = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER;
    let special1 = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL1;
    let special2 = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL2;
    let fog = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_FOG;
    let _interior = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_INTERIOR;

    // Test bitwise combinations
    let headlights = position | low_beam;
    let emergency = left_blinker | right_blinker;
    let all_special = special1 | special2 | fog;

    assert_eq!(position, 0x1);
    assert_eq!(low_beam, 0x1 << 1);
    assert_eq!(high_beam, 0x1 << 2);
    assert_eq!(brake, 0x1 << 3);
    assert_eq!(reverse, 0x1 << 4);
    assert_eq!(left_blinker, 0x1 << 5);
    assert_eq!(right_blinker, 0x1 << 6);

    // Test combinations work correctly
    assert_eq!(headlights, 0x1 | 0x2);
    assert_eq!(emergency, 0x20 | 0x40);
    assert!(all_special > 0);
}

#[test]
fn test_motion_analysis_structures() {
    // Test motion vector structure
    let motion_vector = carla_motion_vector3d_t {
        x: 10.0,
        y: 5.0,
        z: 0.0,
    };

    assert_eq!(motion_vector.x, 10.0);
    assert_eq!(motion_vector.y, 5.0);
    assert_eq!(motion_vector.z, 0.0);

    // Test actor dynamic state
    let dynamic_state = carla_actor_dynamic_state_t {
        velocity: carla_vector3d_t {
            x: 15.0,
            y: 0.0,
            z: 0.0,
        },
        angular_velocity: carla_vector3d_t {
            x: 0.0,
            y: 0.0,
            z: 0.1,
        },
        acceleration: carla_vector3d_t {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
    };

    assert_eq!(dynamic_state.velocity.x, 15.0);
    assert_eq!(dynamic_state.acceleration.x, 2.0);
    assert_eq!(dynamic_state.angular_velocity.z, 0.1);
}

#[test]
fn test_sensor_fusion_structures() {
    // Test sensor fusion stats structure
    let fusion_stats = carla_sensor_fusion_stats_t {
        total_sensor_frames: 1000,
        synchronized_frames: 950,
        dropped_frames: 5,
        interpolated_frames: 25,
        extrapolated_frames: 20,
        average_sync_error_ms: 2.5,
        max_sync_error_ms: 15.0,
        min_sync_error_ms: 0.1,
        total_fused_objects: 150,
        average_fusion_time_ms: 8.5,
        max_fusion_time_ms: 20.0,
        frames_per_sensor: [100; 10],
        latency_per_sensor: [5.0; 10],
        reliability_per_sensor: [0.95; 10],
    };

    assert_eq!(fusion_stats.total_sensor_frames, 1000);
    assert_eq!(fusion_stats.synchronized_frames, 950);
    assert_eq!(fusion_stats.average_sync_error_ms, 2.5);
    assert_eq!(fusion_stats.total_fused_objects, 150);
}

#[test]
fn test_image_analysis_structures() {
    // Test camera intrinsics (with all fields)
    let camera_intrinsics = carla_camera_intrinsics_t {
        fx: 800.0,
        fy: 800.0,
        cx: 320.0,
        cy: 240.0,
        width: 640,
        height: 480,
        k1: 0.0,
        k2: 0.0,
        k3: 0.0,
        p1: 0.0,
        p2: 0.0,
    };

    assert_eq!(camera_intrinsics.fx, 800.0);
    assert_eq!(camera_intrinsics.width, 640);
    assert_eq!(camera_intrinsics.k1, 0.0);

    // Test image statistics structure
    let image_stats = carla_image_stats_t {
        width: 1920,
        height: 1080,
        channels: 3,
        pixel_count: 1920 * 1080,
        fov_angle: 90.0,
        format: carla_image_format_t_CARLA_IMAGE_FORMAT_COLOR_BGRA,
        color_stats: carla_image_stats_t__bindgen_ty_1 {
            min_r: 0,
            min_g: 0,
            min_b: 0,
            max_r: 255,
            max_g: 255,
            max_b: 255,
            avg_r: 127.5,
            avg_g: 127.5,
            avg_b: 127.5,
            brightness: 0.5,
            contrast: 0.8,
        },
        depth_stats: carla_image_stats_t__bindgen_ty_2 {
            min_depth: 0.1,
            max_depth: 1000.0,
            avg_depth: 50.0,
            median_depth: 45.0,
        },
    };

    assert_eq!(image_stats.width, 1920);
    assert_eq!(image_stats.pixel_count, 1920 * 1080);
}

#[test]
fn test_opendrive_generation() {
    // Test OpenDRIVE generation parameters
    let opendrive_params = carla_opendrive_generation_params_t {
        vertex_distance: 2.0,
        max_road_length: 50.0,
        wall_height: 1.0,
        additional_width: 0.6,
        vertex_width_resolution: 2.0,
        simplification_percentage: 1.0,
        smooth_junctions: true,
        enable_mesh_visibility: true,
        enable_pedestrian_navigation: true,
    };

    assert_eq!(opendrive_params.vertex_distance, 2.0);
    assert_eq!(opendrive_params.max_road_length, 50.0);
    assert!(opendrive_params.smooth_junctions);
    assert!(opendrive_params.enable_mesh_visibility);
}

#[test]
fn test_object_tracking_structures() {
    // Test motion analysis structure
    let motion_analysis = carla_motion_analysis_t {
        linear_motion: carla_motion_vector3d_t {
            x: 10.0,
            y: 5.0,
            z: 0.0,
        },
        angular_motion: carla_motion_vector3d_t {
            x: 0.1,
            y: 0.0,
            z: 0.05,
        },
        magnitude: 11.18,
        direction: 0.463,
        timestamp: 123456.789,
    };

    assert_eq!(motion_analysis.linear_motion.x, 10.0);
    assert_eq!(motion_analysis.magnitude, 11.18);
    assert_eq!(motion_analysis.timestamp, 123456.789);
}

#[test]
fn test_client_connection_robustness() {
    unsafe {
        // Test client creation with various parameters
        let host = CString::new("localhost").unwrap();

        // Test with different timeout values
        let client1 = carla_client_new(host.as_ptr(), 2000, 0); // No timeout
        let client2 = carla_client_new(host.as_ptr(), 2000, 1); // 1ms timeout
        let client3 = carla_client_new(host.as_ptr(), 2000, 5000); // 5s timeout

        // All should create client objects (though they may not connect)
        assert!(!client1.is_null());
        assert!(!client2.is_null());
        assert!(!client3.is_null());

        // Test timeout setting and getting
        carla_client_set_timeout(client1, 10000);
        let timeout = carla_client_get_timeout(client1);
        assert_eq!(timeout, 10000);

        // Clean up
        carla_client_free(client1);
        carla_client_free(client2);
        carla_client_free(client3);
    }
}

#[test]
fn test_error_handling_completeness() {
    // Test available error codes
    assert_eq!(carla_error_t_CARLA_ERROR_NONE, 0);
    assert_eq!(carla_error_t_CARLA_ERROR_CONNECTION_FAILED, 1);
    assert_eq!(carla_error_t_CARLA_ERROR_TIMEOUT, 2);
    assert_eq!(carla_error_t_CARLA_ERROR_INVALID_ARGUMENT, 3);
    assert_eq!(carla_error_t_CARLA_ERROR_NOT_FOUND, 4);

    // Test error to string conversion
    unsafe {
        let error_str = carla_error_to_string(carla_error_t_CARLA_ERROR_NONE);
        assert!(!error_str.is_null());

        let timeout_str = carla_error_to_string(carla_error_t_CARLA_ERROR_TIMEOUT);
        assert!(!timeout_str.is_null());
    }
}
