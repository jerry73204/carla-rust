//! LiDAR-to-Camera Projection Example
//!
//! Demonstrates how to project 3D LiDAR points onto a 2D camera image.
//! This example shows sensor fusion techniques by overlaying LiDAR data
//! on camera images, visualizing depth information from the LiDAR sensor.
//!
//! # What it does
//!
//! 1. Spawns a vehicle with autopilot enabled
//! 2. Attaches RGB camera and LiDAR sensor to the vehicle
//! 3. Enables synchronous mode for perfect sensor synchronization
//! 4. For each frame:
//!    - Receives synchronized camera image and LiDAR point cloud
//!    - Transforms LiDAR points from sensor space → world space → camera space
//!    - Projects 3D points to 2D image coordinates
//!    - Colors points by distance (viridis colormap)
//!    - Saves annotated images showing LiDAR overlay
//!
//! # Output
//!
//! Creates `_out/` directory with annotated images showing LiDAR points
//! overlaid on camera images, colored by distance from the sensor.
//!
//! # Usage
//!
//! ```bash
//! cargo run --example lidar_to_camera
//! ```

use carla::{
    client::{ActorBase, Client, Sensor},
    geom::{Location, LocationExt},
    rpc::{AttachmentType, EpisodeSettings},
    sensor::{
        camera::{build_projection_matrix, project_to_2d, world_to_camera},
        data::{Image, LidarMeasurement},
        SensorDataBase,
    },
};
use nalgebra::{Point3, Translation3, UnitQuaternion};
use std::{
    fs,
    sync::{Arc, Mutex},
    time::Duration,
};

const IMAGE_WIDTH: u32 = 800;
const IMAGE_HEIGHT: u32 = 600;
const FOV: f32 = 90.0;
const NUM_FRAMES: usize = 50;

type CameraData = Arc<Mutex<Option<(Image, nalgebra::Isometry3<f32>, usize)>>>;
type LidarData = Arc<Mutex<Option<(LidarMeasurement, nalgebra::Isometry3<f32>, usize)>>>;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("LiDAR-to-Camera Projection Example");
    println!("===================================\n");

    // Create output directory
    fs::create_dir_all("_out")?;

    // Connect to CARLA server
    let client = Client::default();
    let mut world = client.world();

    // Enable synchronous mode for perfect sensor sync
    let original_settings = world.settings();
    let settings = EpisodeSettings {
        synchronous_mode: true,
        fixed_delta_seconds: Some(0.05), // 20 FPS
        ..Default::default()
    };
    world.apply_settings(&settings, Duration::from_secs(5));

    println!("Enabled synchronous mode (20 FPS)");

    // Get blueprints
    let bp_lib = world.blueprint_library();
    let vehicle_bp = bp_lib
        .find("vehicle.lincoln.mkz_2017")
        .ok_or("Failed to find vehicle blueprint")?;

    let mut camera_bp = bp_lib
        .find("sensor.camera.rgb")
        .ok_or("Failed to find RGB camera blueprint")?;

    let mut lidar_bp = bp_lib
        .find("sensor.lidar.ray_cast")
        .ok_or("Failed to find LiDAR blueprint")?;

    // Configure camera
    if !camera_bp.set_attribute("image_size_x", &IMAGE_WIDTH.to_string()) {
        return Err("Failed to set camera image_size_x".into());
    }
    if !camera_bp.set_attribute("image_size_y", &IMAGE_HEIGHT.to_string()) {
        return Err("Failed to set camera image_size_y".into());
    }
    if !camera_bp.set_attribute("fov", &FOV.to_string()) {
        return Err("Failed to set camera fov".into());
    }

    // Configure LiDAR
    if !lidar_bp.set_attribute("range", "50.0") {
        return Err("Failed to set lidar range".into());
    }
    if !lidar_bp.set_attribute("rotation_frequency", "20.0") {
        return Err("Failed to set lidar rotation_frequency".into());
    }
    if !lidar_bp.set_attribute("channels", "32") {
        return Err("Failed to set lidar channels".into());
    }
    if !lidar_bp.set_attribute("points_per_second", "100000") {
        return Err("Failed to set lidar points_per_second".into());
    }
    if !lidar_bp.set_attribute("upper_fov", "15.0") {
        return Err("Failed to set lidar upper_fov".into());
    }
    if !lidar_bp.set_attribute("lower_fov", "-25.0") {
        return Err("Failed to set lidar lower_fov".into());
    }

    // Spawn vehicle
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;
    let vehicle = world
        .spawn_actor(&vehicle_bp, &spawn_point)
        .map_err(|e| format!("Failed to spawn vehicle: {:?}", e))?;

    println!("Spawned vehicle at spawn point 0");

    // Enable autopilot
    let vehicle_actor =
        carla::client::Vehicle::try_from(vehicle.clone()).expect("Failed to convert to Vehicle");
    vehicle_actor.set_autopilot(true);
    println!("Enabled autopilot");

    // Attach camera (front center, at windshield height)
    let camera_transform = nalgebra::Isometry3::from_parts(
        Translation3::new(1.6, 0.0, 1.6),
        UnitQuaternion::identity(),
    );
    let camera_actor = world
        .spawn_actor_opt(
            &camera_bp,
            &camera_transform,
            Some(&vehicle),
            AttachmentType::Rigid,
        )
        .map_err(|e| format!("Failed to spawn camera: {:?}", e))?;
    let camera = Sensor::try_from(camera_actor).map_err(|_| "Failed to convert to Sensor")?;
    println!("Attached RGB camera");

    // Attach LiDAR (slightly above camera)
    let lidar_transform = nalgebra::Isometry3::from_parts(
        Translation3::new(1.0, 0.0, 1.8),
        UnitQuaternion::identity(),
    );
    let lidar_actor = world
        .spawn_actor_opt(
            &lidar_bp,
            &lidar_transform,
            Some(&vehicle),
            AttachmentType::Rigid,
        )
        .map_err(|e| format!("Failed to spawn LiDAR: {:?}", e))?;
    let lidar = Sensor::try_from(lidar_actor).map_err(|_| "Failed to convert to Sensor")?;
    println!("Attached LiDAR sensor\n");

    // Build projection matrix
    let k_matrix = build_projection_matrix(IMAGE_WIDTH, IMAGE_HEIGHT, FOV);
    println!("Built camera projection matrix:");
    println!("{}", k_matrix);
    println!();

    // Shared data between sensors
    let camera_data: CameraData = Arc::new(Mutex::new(None));
    let lidar_data: LidarData = Arc::new(Mutex::new(None));

    // Camera listener
    let camera_data_clone = camera_data.clone();
    camera.listen(move |sensor_data| {
        let frame = sensor_data.frame();
        let transform = sensor_data.sensor_transform();
        if let Ok(image) = Image::try_from(sensor_data) {
            *camera_data_clone.lock().unwrap() = Some((image, transform, frame));
        }
    });

    // LiDAR listener
    let lidar_data_clone = lidar_data.clone();
    lidar.listen(move |sensor_data| {
        let frame = sensor_data.frame();
        let transform = sensor_data.sensor_transform();
        if let Ok(measurement) = LidarMeasurement::try_from(sensor_data) {
            *lidar_data_clone.lock().unwrap() = Some((measurement, transform, frame));
        }
    });

    println!("Capturing and processing {} frames...\n", NUM_FRAMES);

    // Process frames
    for frame_idx in 0..NUM_FRAMES {
        // Tick the world
        world.tick();

        // Small delay to ensure data arrives
        std::thread::sleep(Duration::from_millis(50));

        // Get synchronized data
        let camera_guard = camera_data.lock().unwrap();
        let lidar_guard = lidar_data.lock().unwrap();

        if let (
            Some((image, camera_transform, cam_frame)),
            Some((measurement, lidar_transform, lidar_frame)),
        ) = (camera_guard.as_ref(), lidar_guard.as_ref())
        {
            // Verify frame synchronization
            if cam_frame != lidar_frame {
                println!(
                    "Frame {}: Skipping (camera frame {} != lidar frame {})",
                    frame_idx, cam_frame, lidar_frame
                );
                continue;
            }

            println!(
                "Frame {}: Processing (frame {}, {} lidar points)",
                frame_idx,
                cam_frame,
                measurement.len()
            );

            // Get image data
            let mut image_data = image
                .as_slice()
                .iter()
                .map(|c| [c.r, c.g, c.b])
                .collect::<Vec<_>>();

            // Project LiDAR points onto camera
            let mut points_in_view = 0;
            for detection in measurement.as_slice() {
                // Get 3D point in lidar coordinate space
                let point_lidar =
                    Point3::new(detection.point.x, detection.point.y, detection.point.z);

                // Transform to world space
                let point_world = lidar_transform.transform_point(&point_lidar);

                // Convert Point3 to Location
                let world_location = Location::from_na_point(&point_world);

                // Transform to camera space and convert to standard camera coordinates
                let point_camera = world_to_camera(&world_location, camera_transform);

                // Check if point is in front of camera
                if point_camera.z <= 0.0 {
                    continue;
                }

                // Project to 2D
                let (u, v) = project_to_2d(&point_camera, &k_matrix);

                // Check if point is within image bounds
                if u >= 0.0 && u < IMAGE_WIDTH as f32 && v >= 0.0 && v < IMAGE_HEIGHT as f32 {
                    let u_int = u as usize;
                    let v_int = v as usize;
                    let idx = v_int * IMAGE_WIDTH as usize + u_int;

                    if idx < image_data.len() {
                        // Color by distance (simple intensity-based coloring)
                        let distance = point_camera.z;
                        let normalized = (distance / 50.0).min(1.0);
                        let color = viridis_color(normalized);

                        image_data[idx] = color;
                        points_in_view += 1;
                    }
                }
            }

            println!("  → Projected {} points onto camera image", points_in_view);

            // Save annotated image
            save_rgb_image(
                &format!("_out/lidar_camera_{:04}.png", frame_idx),
                &image_data,
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
            )?;
        } else {
            println!("Frame {}: Waiting for sensor data...", frame_idx);
        }
    }

    println!("\n✓ Saved {} annotated images to _out/", NUM_FRAMES);
    println!("✓ Cleaning up...");

    // Cleanup
    camera.destroy();
    lidar.destroy();
    vehicle.destroy();

    // Restore original settings
    world.apply_settings(&original_settings, Duration::from_secs(5));

    Ok(())
}

/// Simple viridis colormap approximation
fn viridis_color(t: f32) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);

    // Simplified viridis colormap (purple → blue → green → yellow)
    let r = ((0.267 + t * (0.004 + t * (0.435 - t * 0.435))) * 255.0) as u8;
    let g = ((0.005 + t * (0.723 + t * (0.272 - t * 0.723))) * 255.0) as u8;
    let b = ((0.329 + t * (0.528 + t * (-0.857 + t * 0.528))) * 255.0) as u8;

    [r, g, b]
}

/// Save RGB image to disk
fn save_rgb_image(
    path: &str,
    data: &[[u8; 3]],
    width: u32,
    height: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    use image::{ImageBuffer, Rgb};

    let flat_data: Vec<u8> = data.iter().flat_map(|&[r, g, b]| [r, g, b]).collect();

    let img_buffer = ImageBuffer::<Rgb<u8>, Vec<u8>>::from_raw(width, height, flat_data)
        .ok_or("Failed to create image buffer")?;

    img_buffer.save(path)?;
    Ok(())
}
