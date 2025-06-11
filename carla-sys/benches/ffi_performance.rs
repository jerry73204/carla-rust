//! Performance benchmarks comparing C FFI vs original autocxx implementation
//!
//! These benchmarks measure the performance characteristics of:
//! - Basic client operations (connect, timeout, version)
//! - Vehicle control application
//! - Sensor data processing
//! - World and map operations
//! - Error handling overhead

use carla_sys::*;
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::{ffi::CString, ptr};

/// Benchmark client creation and basic operations
fn bench_client_operations(c: &mut Criterion) {
    c.bench_function("client_creation", |b| {
        b.iter(|| unsafe {
            let host = CString::new("localhost").unwrap();
            let client = carla_client_new(host.as_ptr(), 2000, 1);
            black_box(client);
            if !client.is_null() {
                carla_client_free(client);
            }
        })
    });

    c.bench_function("client_timeout_operations", |b| unsafe {
        let host = CString::new("localhost").unwrap();
        let client = carla_client_new(host.as_ptr(), 2000, 1);

        b.iter(|| {
            carla_client_set_timeout(client, black_box(5000));
            let timeout = carla_client_get_timeout(client);
            black_box(timeout);
        });

        if !client.is_null() {
            carla_client_free(client);
        }
    });

    c.bench_function("client_version_check", |b| unsafe {
        let host = CString::new("localhost").unwrap();
        let client = carla_client_new(host.as_ptr(), 2000, 1);

        b.iter(|| {
            let version = carla_client_get_client_version(client);
            black_box(version);
        });

        if !client.is_null() {
            carla_client_free(client);
        }
    });
}

/// Benchmark vehicle control structures and operations
fn bench_vehicle_control(c: &mut Criterion) {
    c.bench_function("vehicle_control_creation", |b| {
        b.iter(|| {
            let control = carla_vehicle_control_t {
                throttle: black_box(0.8),
                steer: black_box(-0.3),
                brake: black_box(0.0),
                hand_brake: black_box(false),
                reverse: black_box(false),
                manual_gear_shift: black_box(false),
                gear: black_box(1),
            };
            black_box(control);
        })
    });

    c.bench_function("ackermann_control_creation", |b| {
        b.iter(|| {
            let ackermann_control = carla_vehicle_ackermann_control_t {
                steer: black_box(15.0),
                steer_speed: black_box(45.0),
                speed: black_box(10.0),
                acceleration: black_box(1.5),
                jerk: black_box(0.8),
            };
            black_box(ackermann_control);
        })
    });

    c.bench_function("light_state_combinations", |b| {
        b.iter(|| {
            let position = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION;
            let low_beam = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM;
            let brake = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_BRAKE;
            let left_blinker = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER;

            let headlights = position | low_beam;
            let night_driving = headlights | brake;
            let left_turn = night_driving | left_blinker;

            black_box(left_turn);
        })
    });
}

/// Benchmark geometry and transform operations
fn bench_geometry_operations(c: &mut Criterion) {
    c.bench_function("vector3d_creation", |b| {
        b.iter(|| {
            let vector = carla_vector3d_t {
                x: black_box(1.0),
                y: black_box(2.0),
                z: black_box(3.0),
            };
            black_box(vector);
        })
    });

    c.bench_function("transform_creation", |b| {
        b.iter(|| {
            let transform = carla_transform_t {
                location: carla_vector3d_t {
                    x: black_box(10.0),
                    y: black_box(20.0),
                    z: black_box(1.5),
                },
                rotation: carla_rotation_t {
                    pitch: black_box(0.0),
                    yaw: black_box(90.0),
                    roll: black_box(0.0),
                },
            };
            black_box(transform);
        })
    });

    c.bench_function("rotation_creation", |b| {
        b.iter(|| {
            let rotation = carla_rotation_t {
                pitch: black_box(0.0),
                yaw: black_box(90.0),
                roll: black_box(0.0),
            };
            black_box(rotation);
        })
    });
}

/// Benchmark sensor data structures
fn bench_sensor_data(c: &mut Criterion) {
    c.bench_function("image_data_creation", |b| {
        b.iter(|| {
            let image_data = carla_image_data_t {
                width: black_box(1920),
                height: black_box(1080),
                fov: black_box(90),
                raw_data: ptr::null(),
                raw_data_size: black_box(1920 * 1080 * 4),
            };
            black_box(image_data);
        })
    });

    c.bench_function("lidar_detection_creation", |b| {
        b.iter(|| {
            let detection = carla_lidar_detection_t {
                x: black_box(1.0),
                y: black_box(2.0),
                z: black_box(3.0),
                intensity: black_box(0.8),
            };
            black_box(detection);
        })
    });

    c.bench_function("imu_data_creation", |b| {
        b.iter(|| {
            let imu_data = carla_imu_data_t {
                accelerometer: carla_vector3d_t {
                    x: black_box(0.0),
                    y: black_box(0.0),
                    z: black_box(-9.81),
                },
                gyroscope: carla_vector3d_t {
                    x: black_box(0.0),
                    y: black_box(0.0),
                    z: black_box(0.0),
                },
                compass: black_box(0.0),
            };
            black_box(imu_data);
        })
    });

    c.bench_function("gnss_data_creation", |b| {
        b.iter(|| {
            let gnss_data = carla_gnss_data_t {
                latitude: black_box(37.7749),
                longitude: black_box(-122.4194),
                altitude: black_box(100.0),
            };
            black_box(gnss_data);
        })
    });
}

/// Benchmark new CARLA 0.10.0 sensor types
fn bench_new_sensor_types(c: &mut Criterion) {
    c.bench_function("dvs_event_creation", |b| {
        b.iter(|| {
            let dvs_event = carla_dvs_event_t {
                x: black_box(320),
                y: black_box(240),
                t: black_box(12345678),
                pol: black_box(true),
            };
            black_box(dvs_event);
        })
    });

    c.bench_function("optical_flow_pixel_creation", |b| {
        b.iter(|| {
            let flow_pixel = carla_optical_flow_pixel_t {
                x: black_box(1.5),
                y: black_box(-2.3),
            };
            black_box(flow_pixel);
        })
    });

    c.bench_function("dvs_analysis_creation", |b| {
        b.iter(|| {
            let dvs_analysis = carla_dvs_analysis_t {
                positive_events: black_box(150),
                negative_events: black_box(75),
                total_events: black_box(225),
                event_rate: black_box(1000.0),
                active_region: carla_image_roi_t {
                    x: black_box(100),
                    y: black_box(100),
                    width: black_box(200),
                    height: black_box(200),
                },
                activity_density: black_box(0.25),
            };
            black_box(dvs_analysis);
        })
    });

    c.bench_function("optical_flow_analysis_creation", |b| {
        b.iter(|| {
            let flow_analysis = carla_optical_flow_analysis_t {
                average_flow: carla_vector3d_t {
                    x: black_box(0.5),
                    y: black_box(-0.2),
                    z: black_box(0.0),
                },
                magnitude_avg: black_box(2.1),
                magnitude_max: black_box(5.8),
                moving_pixels: black_box(12500),
                motion_density: black_box(0.65),
                dominant_direction: carla_vector3d_t {
                    x: black_box(0.8),
                    y: black_box(0.6),
                    z: black_box(0.0),
                },
            };
            black_box(flow_analysis);
        })
    });
}

/// Benchmark motion analysis and sensor fusion
fn bench_motion_analysis(c: &mut Criterion) {
    c.bench_function("actor_dynamic_state_creation", |b| {
        b.iter(|| {
            let dynamic_state = carla_actor_dynamic_state_t {
                velocity: carla_vector3d_t {
                    x: black_box(15.0),
                    y: black_box(0.0),
                    z: black_box(0.0),
                },
                angular_velocity: carla_vector3d_t {
                    x: black_box(0.0),
                    y: black_box(0.0),
                    z: black_box(0.1),
                },
                acceleration: carla_vector3d_t {
                    x: black_box(2.0),
                    y: black_box(0.0),
                    z: black_box(0.0),
                },
            };
            black_box(dynamic_state);
        })
    });

    c.bench_function("motion_analysis_creation", |b| {
        b.iter(|| {
            let motion_analysis = carla_motion_analysis_t {
                linear_motion: carla_motion_vector3d_t {
                    x: black_box(10.0),
                    y: black_box(5.0),
                    z: black_box(0.0),
                },
                angular_motion: carla_motion_vector3d_t {
                    x: black_box(0.1),
                    y: black_box(0.0),
                    z: black_box(0.05),
                },
                magnitude: black_box(11.18),
                direction: black_box(0.463),
                timestamp: black_box(123456.789),
            };
            black_box(motion_analysis);
        })
    });

    c.bench_function("sensor_fusion_stats_creation", |b| {
        b.iter(|| {
            let fusion_stats = carla_sensor_fusion_stats_t {
                total_sensor_frames: black_box(1000),
                synchronized_frames: black_box(950),
                dropped_frames: black_box(5),
                interpolated_frames: black_box(25),
                extrapolated_frames: black_box(20),
                average_sync_error_ms: black_box(2.5),
                max_sync_error_ms: black_box(15.0),
                min_sync_error_ms: black_box(0.1),
                total_fused_objects: black_box(150),
                average_fusion_time_ms: black_box(8.5),
                max_fusion_time_ms: black_box(20.0),
                frames_per_sensor: [black_box(100); 10],
                latency_per_sensor: [black_box(5.0); 10],
                reliability_per_sensor: [black_box(0.95); 10],
            };
            black_box(fusion_stats);
        })
    });
}

/// Benchmark error handling
fn bench_error_handling(c: &mut Criterion) {
    c.bench_function("error_code_comparison", |b| {
        b.iter(|| {
            let error = black_box(carla_error_t_CARLA_ERROR_TIMEOUT);
            let is_ok = error == carla_error_t_CARLA_ERROR_NONE;
            black_box(is_ok);
        })
    });

    c.bench_function("error_to_string_conversion", |b| {
        b.iter(|| unsafe {
            let error = black_box(carla_error_t_CARLA_ERROR_CONNECTION_FAILED);
            let error_str = carla_error_to_string(error);
            black_box(error_str);
        })
    });
}

/// Benchmark string operations
fn bench_string_operations(c: &mut Criterion) {
    c.bench_function("cstring_creation", |b| {
        b.iter(|| {
            let host = CString::new("localhost").unwrap();
            black_box(host);
        })
    });

    c.bench_function("cstring_from_vec", |b| {
        b.iter(|| {
            let data = vec![b'l', b'o', b'c', b'a', b'l', b'h', b'o', b's', b't', 0];
            let cstring = CString::from_vec_with_nul(data).unwrap();
            black_box(cstring);
        })
    });
}

/// Benchmark memory operations and allocation patterns
fn bench_memory_operations(c: &mut Criterion) {
    c.bench_function("large_struct_creation", |b| {
        b.iter(|| {
            // Simulate a large structure with many fields
            let large_struct = carla_image_stats_t {
                width: black_box(1920),
                height: black_box(1080),
                channels: black_box(3),
                pixel_count: black_box(1920 * 1080),
                fov_angle: black_box(90.0),
                format: black_box(carla_image_format_t_CARLA_IMAGE_FORMAT_COLOR_BGRA),
                color_stats: carla_image_stats_t__bindgen_ty_1 {
                    min_r: black_box(0),
                    min_g: black_box(0),
                    min_b: black_box(0),
                    max_r: black_box(255),
                    max_g: black_box(255),
                    max_b: black_box(255),
                    avg_r: black_box(127.5),
                    avg_g: black_box(127.5),
                    avg_b: black_box(127.5),
                    brightness: black_box(0.5),
                    contrast: black_box(0.8),
                },
                depth_stats: carla_image_stats_t__bindgen_ty_2 {
                    min_depth: black_box(0.1),
                    max_depth: black_box(1000.0),
                    avg_depth: black_box(50.0),
                    median_depth: black_box(45.0),
                },
            };
            black_box(large_struct);
        })
    });

    c.bench_function("pointer_null_checks", |b| {
        b.iter(|| {
            let ptr: *mut carla_client_t = ptr::null_mut();
            let is_null = ptr.is_null();
            black_box(is_null);
        })
    });
}

criterion_group!(
    benches,
    bench_client_operations,
    bench_vehicle_control,
    bench_geometry_operations,
    bench_sensor_data,
    bench_new_sensor_types,
    bench_motion_analysis,
    bench_error_handling,
    bench_string_operations,
    bench_memory_operations
);

criterion_main!(benches);
