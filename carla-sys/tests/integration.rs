use carla_sys::*;
use std::ptr;

#[test]
fn test_error_string_conversion() {
    unsafe {
        // Test error string conversion functions
        let error_none = carla_error_to_string(carla_error_t_CARLA_ERROR_NONE);
        assert!(!error_none.is_null());

        let error_conn = carla_error_to_string(carla_error_t_CARLA_ERROR_CONNECTION_FAILED);
        assert!(!error_conn.is_null());
    }
}

#[test]
fn test_vehicle_control_structure() {
    // Test that vehicle control structure has expected layout
    let control = carla_vehicle_control_t {
        throttle: 0.8,
        steer: -0.5,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 1,
    };

    assert_eq!(control.throttle, 0.8);
    assert_eq!(control.steer, -0.5);
    assert_eq!(control.brake, 0.0);
    assert_eq!(control.gear, 1);
}

#[test]
fn test_vehicle_ackermann_control_structure() {
    // Test Ackermann control structure
    let control = carla_vehicle_ackermann_control_t {
        steer: 30.0,
        steer_speed: 10.0,
        speed: 20.0,
        acceleration: 2.0,
        jerk: 0.5,
    };

    assert_eq!(control.steer, 30.0);
    assert_eq!(control.speed, 20.0);
    assert_eq!(control.acceleration, 2.0);
}

#[test]
fn test_vehicle_light_state_enums() {
    // Test vehicle light state enum values
    assert_eq!(carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_NONE, 0);
    assert_eq!(
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION,
        0x1
    );
    assert_eq!(
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM,
        0x1 << 1
    );
    assert_eq!(
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_HIGH_BEAM,
        0x1 << 2
    );
    assert_eq!(
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_BRAKE,
        0x1 << 3
    );
    assert_eq!(
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER,
        0x1 << 4
    );
    assert_eq!(
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER,
        0x1 << 5
    );

    // Test combination of lights
    let headlights = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
        | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM;
    assert_eq!(headlights, 0x1 | 0x2);
}

#[test]
fn test_vehicle_door_enums() {
    // Test vehicle door enum values (CARLA 0.10.0 feature)
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL, 0);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FR, 1);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RL, 2);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RR, 3);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD, 4);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK, 5);
    assert_eq!(carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL, 6);
}

#[test]
fn test_vehicle_wheel_location_enums() {
    // Test vehicle wheel location enum values
    assert_eq!(carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_FL, 0);
    assert_eq!(carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_FR, 1);
    assert_eq!(carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_BL, 2);
    assert_eq!(carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_BR, 3);

    // Test bike/bicycle wheel locations
    assert_eq!(carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_FRONT, 0);
    assert_eq!(carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_BACK, 1);
}

#[test]
fn test_traffic_light_state_enums() {
    // Test traffic light state enum values
    assert_eq!(carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_RED, 0);
    assert_eq!(carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_YELLOW, 1);
    assert_eq!(carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_GREEN, 2);
    assert_eq!(carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF, 3);
    assert_eq!(carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_UNKNOWN, 4);
}

#[test]
fn test_vehicle_failure_state_enums() {
    // Test vehicle failure state enum values
    assert_eq!(carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_NONE, 0);
    assert_eq!(
        carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_ROLLOVER,
        1
    );
    assert_eq!(
        carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_ENGINE,
        2
    );
    assert_eq!(
        carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_TIRE_PUNCTURE,
        3
    );
}

#[test]
fn test_ackermann_controller_settings_structure() {
    // Test Ackermann controller settings structure
    let settings = carla_ackermann_controller_settings_t {
        speed_kp: 0.5,
        speed_ki: 0.1,
        speed_kd: 0.01,
        accel_kp: 0.3,
        accel_ki: 0.05,
        accel_kd: 0.02,
    };

    assert_eq!(settings.speed_kp, 0.5);
    assert_eq!(settings.speed_ki, 0.1);
    assert_eq!(settings.accel_kp, 0.3);
}

#[test]
fn test_client_creation_and_cleanup() {
    unsafe {
        // Test client creation without actual connection
        let host = b"localhost\0".as_ptr() as *const i8;
        let client = carla_client_new(host, 2000, 1);

        // Client should be created (but might not connect)
        assert!(!client.is_null());

        // Test timeout settings
        carla_client_set_timeout(client, 5000);
        let timeout = carla_client_get_timeout(client);
        assert_eq!(timeout, 5000);

        // Cleanup
        carla_client_free(client);
    }
}

#[test]
fn test_vector_and_transform_types() {
    // Test that the geometry types have expected sizes and alignment
    assert_eq!(std::mem::size_of::<carla_vector3d_t>(), 12); // 3 * f32
    assert_eq!(std::mem::size_of::<carla_rotation_t>(), 12); // 3 * f32
    assert_eq!(std::mem::size_of::<carla_transform_t>(), 24); // vector3d + rotation

    // Test that we can create and manipulate these types
    let location = carla_vector3d_t {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    let rotation = carla_rotation_t {
        pitch: 0.0,
        yaw: 90.0,
        roll: 0.0,
    };
    let transform = carla_transform_t { location, rotation };

    assert_eq!(transform.location.x, 1.0);
    assert_eq!(transform.rotation.yaw, 90.0);
}

#[test]
fn test_weather_parameters() {
    // Test weather parameters structure
    let weather = carla_weather_parameters_t {
        cloudiness: 0.5,
        precipitation: 0.0,
        precipitation_deposits: 0.0,
        wind_intensity: 10.0,
        sun_azimuth_angle: 180.0,
        sun_altitude_angle: 45.0,
        fog_density: 0.1,
        fog_distance: 100.0,
        wetness: 0.0,
        fog_falloff: 0.2,
        scattering_intensity: 1.0,
        mie_scattering_scale: 0.03,
        rayleigh_scattering_scale: 0.0331,
    };

    assert_eq!(weather.cloudiness, 0.5);
    assert_eq!(weather.sun_azimuth_angle, 180.0);
}

#[test]
fn test_spawn_result_structure() {
    // Test that spawn result has expected layout
    let spawn_result = carla_spawn_result_t {
        actor: ptr::null_mut(),
        error: carla_error_t_CARLA_ERROR_NONE,
    };

    assert!(spawn_result.actor.is_null());
    assert_eq!(spawn_result.error, carla_error_t_CARLA_ERROR_NONE);
}

#[test]
fn test_sensor_data_types() {
    // Test sensor data type enum values
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_UNKNOWN, 0);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMAGE, 1);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_LIDAR, 2);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_SEMANTIC_LIDAR, 3);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_RADAR, 4);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMU, 5);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_GNSS, 6);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_COLLISION, 7);
    assert_eq!(carla_sensor_data_type_t_CARLA_SENSOR_DATA_LANE_INVASION, 8);
    assert_eq!(
        carla_sensor_data_type_t_CARLA_SENSOR_DATA_OBSTACLE_DETECTION,
        9
    );
    assert_eq!(
        carla_sensor_data_type_t_CARLA_SENSOR_DATA_DVS_EVENT_ARRAY,
        10
    );
}

#[test]
fn test_sensor_data_structures() {
    // Test that sensor data structures have expected layout
    let image_data = carla_image_data_t {
        width: 1920,
        height: 1080,
        fov: 90,
        raw_data: ptr::null(),
        raw_data_size: 0,
    };

    assert_eq!(image_data.width, 1920);
    assert_eq!(image_data.height, 1080);
    assert_eq!(image_data.fov, 90);

    let lidar_detection = carla_lidar_detection_t {
        x: 1.0,
        y: 2.0,
        z: 3.0,
        intensity: 0.8,
    };

    assert_eq!(lidar_detection.x, 1.0);
    assert_eq!(lidar_detection.intensity, 0.8);

    let imu_data = carla_imu_data_t {
        accelerometer: carla_vector3d_t {
            x: 0.0,
            y: 0.0,
            z: -9.81,
        },
        gyroscope: carla_vector3d_t {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        compass: 0.0,
    };

    assert_eq!(imu_data.accelerometer.z, -9.81);
    assert_eq!(imu_data.compass, 0.0);

    let gnss_data = carla_gnss_data_t {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 100.0,
    };

    assert_eq!(gnss_data.latitude, 37.7749);
    assert_eq!(gnss_data.longitude, -122.4194);
}

#[test]
fn test_dvs_and_optical_flow_data_types() {
    // Test DVS event structure
    let dvs_event = carla_dvs_event_t {
        x: 320,
        y: 240,
        t: 12345678,
        pol: true,
    };

    assert_eq!(dvs_event.x, 320);
    assert_eq!(dvs_event.y, 240);
    assert_eq!(dvs_event.t, 12345678);
    assert_eq!(dvs_event.pol, true);

    // Test optical flow pixel structure
    let optical_flow_pixel = carla_optical_flow_pixel_t { x: 1.5, y: -2.3 };

    assert_eq!(optical_flow_pixel.x, 1.5);
    assert_eq!(optical_flow_pixel.y, -2.3);

    // Test DVS analysis structure initialization
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
    assert_eq!(dvs_analysis.event_rate, 1000.0);
    assert_eq!(dvs_analysis.activity_density, 0.25);

    // Test optical flow analysis structure initialization
    let optical_flow_analysis = carla_optical_flow_analysis_t {
        average_flow: carla_vector3d_t {
            x: 0.5,
            y: -0.2,
            z: 0.0,
        },
        magnitude_avg: 2.1,
        magnitude_max: 5.8,
        moving_pixels: 12500,
        motion_density: 0.65,
        dominant_direction: carla_vector3d_t {
            x: 0.8,
            y: 0.6,
            z: 0.0,
        },
    };

    assert_eq!(optical_flow_analysis.average_flow.x, 0.5);
    assert_eq!(optical_flow_analysis.magnitude_avg, 2.1);
    assert_eq!(optical_flow_analysis.moving_pixels, 12500);
    assert_eq!(optical_flow_analysis.motion_density, 0.65);
}

#[test]
fn test_sensor_data_type_dvs_optical_flow() {
    // Test that DVS and optical flow sensor data types are defined
    assert_eq!(
        carla_sensor_data_type_t_CARLA_SENSOR_DATA_DVS_EVENT_ARRAY,
        10
    );
    assert_eq!(
        carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE,
        11
    );
}

#[test]
fn test_camera_analysis_structures() {
    // Test image statistics structure (using actual fields)
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
    assert_eq!(image_stats.height, 1080);
    assert_eq!(image_stats.pixel_count, 1920 * 1080);
    assert_eq!(image_stats.fov_angle, 90.0);

    // Test image ROI structure
    let roi = carla_image_roi_t {
        x: 100,
        y: 50,
        width: 640,
        height: 480,
    };

    assert_eq!(roi.x, 100);
    assert_eq!(roi.y, 50);
    assert_eq!(roi.width, 640);
    assert_eq!(roi.height, 480);

    // Test semantic analysis structure (using actual fields)
    let semantic_analysis = carla_semantic_analysis_t {
        classes: ptr::null_mut(),
        class_count: 15,
        total_pixels: 307200,
        class_histogram: ptr::null_mut(),
        histogram_size: 256,
    };

    assert_eq!(semantic_analysis.class_count, 15);
    assert_eq!(semantic_analysis.total_pixels, 307200);
    assert_eq!(semantic_analysis.histogram_size, 256);
}

#[test]
fn test_motion_analysis_structures() {
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

    // Test actor dynamic state structure
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
