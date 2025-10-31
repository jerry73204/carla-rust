//! Hud (Heads-Up Display) module
//!
//! The Hud struct is responsible for:
//! - Real-time telemetry display (FPS, speed, location, compass)
//! - IMU data (accelerometer, gyroscope)
//! - Vehicle control state (throttle, steer, brake, gear)
//! - Collision history graph (200 frame rolling window)
//! - Nearby vehicles list (sorted by distance)
//! - Notification integration
//!
//! ## Phase 13 Implementation
//!
//! - Phase 4: Basic telemetry (FPS, speed, location, vehicle/map name)
//! - Phase 5: Sensor data (IMU compass, accelerometer, gyroscope, GNSS, collision graph)
//! - Phase 6: Control state (throttle/brake/steer bars, gear, nearby vehicles)
//! - Phase 7: Ackermann controller info

use carla::prelude::*;
use eyre::Result;
use macroquad::prelude::*;

/// Hud display manager
///
/// Renders all on-screen telemetry and information
pub struct Hud {
    #[allow(dead_code)]
    pub width: u32,
    pub height: u32,
    pub show_info: bool,

    // ✅ Subphase 12.4.1: FPS tracking
    pub server_fps: f32,
    pub frame_count: u32,
    pub fps_timer: f32,

    // ✅ Subphase 12.4.2: Telemetry data
    pub vehicle_name: String,
    pub map_name: String,
    pub speed_kmh: f32,
    pub location: (f32, f32, f32),

    // ✅ Subphase 12.5: Sensor data
    pub imu_sensor: Option<crate::sensors::IMUSensor>,
    pub gnss_sensor: Option<crate::sensors::GnssSensor>,
    pub collision_sensor: Option<crate::sensors::CollisionSensor>,
    pub lane_invasion_sensor: Option<crate::sensors::LaneInvasionSensor>,
    pub radar_sensor: Option<crate::sensors::RadarSensor>,

    // ✅ Subphase 12.6: Control state
    pub throttle: f32,
    pub steer: f32,
    pub brake: f32,
    pub hand_brake: bool,
    pub reverse: bool,
    pub gear: i32,
    pub manual_gear_shift: bool,
    pub nearby_vehicles: Vec<(String, f32)>,
    // TODO Phase 7.3: Add Ackermann display state
    // show_ackermann_info: bool,
}

impl Hud {
    /// Create a new Hud
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            show_info: true,
            server_fps: 0.0,
            frame_count: 0,
            fps_timer: 0.0,
            vehicle_name: String::new(),
            map_name: String::new(),
            speed_kmh: 0.0,
            location: (0.0, 0.0, 0.0),
            imu_sensor: None,
            gnss_sensor: None,
            collision_sensor: None,
            lane_invasion_sensor: None,
            radar_sensor: None,
            throttle: 0.0,
            steer: 0.0,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            gear: 0,
            manual_gear_shift: false,
            nearby_vehicles: Vec::new(),
        }
    }

    /// Spawn all sensors
    ///
    /// ✅ Subphase 12.5: Spawn all four sensors
    pub fn spawn_sensors(&mut self, world: &mut crate::world::World) -> Result<()> {
        // Spawn IMU sensor
        let mut imu = crate::sensors::IMUSensor::new();
        imu.spawn(world)?;
        self.imu_sensor = Some(imu);

        // Spawn GNSS sensor
        let mut gnss = crate::sensors::GnssSensor::new();
        gnss.spawn(world)?;
        self.gnss_sensor = Some(gnss);

        // Spawn Collision sensor
        let mut collision = crate::sensors::CollisionSensor::new();
        collision.spawn(world)?;
        self.collision_sensor = Some(collision);

        // Spawn Lane Invasion sensor
        let mut lane_invasion = crate::sensors::LaneInvasionSensor::new();
        lane_invasion.spawn(world)?;
        self.lane_invasion_sensor = Some(lane_invasion);

        // Spawn Radar sensor
        let mut radar = crate::sensors::RadarSensor::new();
        radar.spawn(world)?;
        self.radar_sensor = Some(radar);

        Ok(())
    }

    /// Update Hud state
    ///
    /// ✅ Subphase 12.4: Gather telemetry from world
    pub fn update(&mut self, world: &crate::world::World, delta_time: f32) {
        // ✅ Subphase 12.4.1: Update FPS counter
        self.frame_count += 1;
        self.fps_timer += delta_time;
        if self.fps_timer >= 1.0 {
            self.server_fps = self.frame_count as f32 / self.fps_timer;
            self.frame_count = 0;
            self.fps_timer = 0.0;
        }

        // ✅ Subphase 12.4.2: Gather vehicle telemetry
        if let Some(ref player) = world.player {
            // Get vehicle transform and velocity
            let transform = player.transform();
            let velocity = player.velocity();

            // Calculate speed in km/h
            let velocity_ms =
                (velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)
                    .sqrt();
            self.speed_kmh = velocity_ms * 3.6; // m/s to km/h

            // Get location from transform translation
            let translation = transform.translation;
            self.location = (translation.x, translation.y, translation.z);

            // Get vehicle blueprint ID as name
            if self.vehicle_name.is_empty() {
                let type_id = player.type_id();
                self.vehicle_name = type_id.clone();
            }
        }

        // Get map name
        if self.map_name.is_empty() {
            let map = world.world.map();
            self.map_name = map.name();
        }

        // ✅ Subphase 12.5.3: Update collision sensor frame count
        if let Some(ref collision) = self.collision_sensor {
            collision.update_frame();
        }

        // ✅ Subphase 12.6.1: Get vehicle control state
        if let Some(ref player) = world.player {
            let control = player.control();
            self.throttle = control.throttle;
            self.steer = control.steer;
            self.brake = control.brake;
            self.hand_brake = control.hand_brake;
            self.reverse = control.reverse;
            self.gear = control.gear;
            self.manual_gear_shift = control.manual_gear_shift;
        }

        // ✅ Subphase 12.6.2: Calculate nearby vehicles
        self.update_nearby_vehicles(world);
    }

    /// Update nearby vehicles list
    ///
    /// ✅ Subphase 12.6.2: Find all vehicles within 200m, sorted by distance
    fn update_nearby_vehicles(&mut self, world: &crate::world::World) {
        self.nearby_vehicles.clear();

        let player = match world.player.as_ref() {
            Some(p) => p,
            None => return,
        };

        let player_location = player.transform().translation;

        // Get all actors and filter for vehicles
        let actors = world.world.actors();
        let vehicles: Vec<_> = actors
            .iter()
            .filter(|a| a.type_id().starts_with("vehicle."))
            .filter(|a| a.id() != player.id()) // Exclude player vehicle
            .collect();

        // Calculate distances and filter within 200m
        let mut nearby: Vec<(String, f32)> = vehicles
            .iter()
            .map(|v| {
                let v_location = v.transform().translation;
                let dx = v_location.x - player_location.x;
                let dy = v_location.y - player_location.y;
                let distance = (dx * dx + dy * dy).sqrt();

                // Get vehicle name (last part of type_id)
                let name = v
                    .type_id()
                    .rsplit('.')
                    .next()
                    .unwrap_or("unknown")
                    .to_string();

                (name, distance)
            })
            .filter(|(_, dist)| *dist <= 200.0)
            .collect();

        // Sort by distance
        nearby.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Keep top 5
        nearby.truncate(5);

        self.nearby_vehicles = nearby;
    }

    /// Render Hud to screen
    ///
    /// ✅ Subphase 12.4: Render FPS and telemetry
    pub fn render(&self) -> Result<()> {
        if !self.show_info {
            return Ok(());
        }

        // ✅ Subphase 12.4.1: Render semi-transparent overlay (220x720, alpha 100)
        let overlay_width = 220.0;
        let overlay_color = Color::new(0.0, 0.0, 0.0, 100.0 / 255.0);
        draw_rectangle(0.0, 0.0, overlay_width, self.height as f32, overlay_color);

        let mut y_offset = 14.0;
        let x_pos = 8.0;
        let line_height = 18.0;
        let font_size = 14.0;

        // ✅ Subphase 12.4.1: Display FPS
        self.draw_text(
            &format!("FPS (server): {:.0}", self.server_fps),
            x_pos,
            y_offset,
            font_size,
        );
        y_offset += line_height;

        // Client FPS from macroquad
        let client_fps = get_fps();
        self.draw_text(
            &format!("FPS (client): {}", client_fps),
            x_pos,
            y_offset,
            font_size,
        );
        y_offset += line_height;

        y_offset += line_height / 2.0; // Spacing

        // ✅ Subphase 12.4.2: Display vehicle telemetry
        self.draw_text(
            &format!("Speed: {:.0} km/h", self.speed_kmh),
            x_pos,
            y_offset,
            font_size,
        );
        y_offset += line_height;

        self.draw_text(
            &format!("Location: ({:.1}, {:.1})", self.location.0, self.location.1),
            x_pos,
            y_offset,
            font_size,
        );
        y_offset += line_height;

        // Vehicle name (simplified - show last part after '.')
        let vehicle_display = if let Some(last_part) = self.vehicle_name.rsplit('.').next() {
            last_part
        } else {
            &self.vehicle_name
        };
        self.draw_text(
            &format!("Vehicle: {}", vehicle_display),
            x_pos,
            y_offset,
            font_size,
        );
        y_offset += line_height;

        self.draw_text(
            &format!("Map: {}", self.map_name),
            x_pos,
            y_offset,
            font_size,
        );
        y_offset += line_height;

        y_offset += line_height / 2.0; // Spacing

        // ✅ Subphase 12.5.1: Render IMU data
        if let Some(ref imu) = self.imu_sensor {
            let heading = imu.get_heading();
            self.draw_text(
                &format!("Compass: {} ({:.0}°)", heading, imu.compass()),
                x_pos,
                y_offset,
                font_size,
            );
            y_offset += line_height;

            let accel = imu.accelerometer();
            self.draw_text(
                &format!("Accelero: ({:.1}, {:.1}, {:.1})", accel.0, accel.1, accel.2),
                x_pos,
                y_offset,
                font_size,
            );
            y_offset += line_height;

            let gyro = imu.gyroscope();
            self.draw_text(
                &format!("Gyroscope: ({:.1}, {:.1}, {:.1})", gyro.0, gyro.1, gyro.2),
                x_pos,
                y_offset,
                font_size,
            );
            y_offset += line_height;
        }

        y_offset += line_height / 2.0; // Spacing

        // ✅ Subphase 12.5.2: Render GNSS data
        if let Some(ref gnss) = self.gnss_sensor {
            self.draw_text(
                &format!("Latitude: {:.6}", gnss.latitude()),
                x_pos,
                y_offset,
                font_size,
            );
            y_offset += line_height;

            self.draw_text(
                &format!("Longitude: {:.6}", gnss.longitude()),
                x_pos,
                y_offset,
                font_size,
            );
            y_offset += line_height;
        }

        y_offset += line_height / 2.0; // Spacing

        // ✅ Subphase 12.5.3: Render collision history graph
        if let Some(ref collision) = self.collision_sensor {
            let frame = collision.frame_count();
            let history = collision.get_collision_history(frame);

            if history.iter().any(|&v| v > 0.0) {
                self.draw_text("Collision:", x_pos, y_offset, font_size);
                y_offset += line_height;

                // Draw simple bar graph (200 frames, 200px wide)
                let graph_x = x_pos;
                let graph_y = y_offset;
                let graph_width = 200.0;
                let graph_height = 30.0;

                // Draw background
                draw_rectangle(
                    graph_x,
                    graph_y,
                    graph_width,
                    graph_height,
                    Color::new(0.2, 0.2, 0.2, 0.8),
                );

                // Draw bars
                for (i, &intensity) in history.iter().enumerate() {
                    if intensity > 0.0 {
                        let x = graph_x + (i as f32);
                        let h = intensity * graph_height;
                        draw_rectangle(x, graph_y + graph_height - h, 1.0, h, RED);
                    }
                }

                // y_offset prepared for future use
            }
        }

        y_offset += line_height / 2.0; // Spacing

        // ✅ Subphase 12.6.1: Render control state bars
        self.draw_text("Controls:", x_pos, y_offset, font_size);
        y_offset += line_height;

        // Throttle bar (green)
        self.draw_bar(
            "Throttle",
            self.throttle,
            x_pos,
            y_offset,
            100.0,
            10.0,
            GREEN,
        );
        y_offset += 14.0;

        // Brake bar (red)
        self.draw_bar("Brake", self.brake, x_pos, y_offset, 100.0, 10.0, RED);
        y_offset += 14.0;

        // Steer bar (blue, centered at 0)
        self.draw_steer_bar(x_pos, y_offset, 100.0, 10.0);
        y_offset += 14.0;

        // Gear and flags
        let mut status_parts = vec![format!("Gear: {}", self.gear)];
        if self.manual_gear_shift {
            status_parts.push("M".to_string());
        }
        if self.hand_brake {
            status_parts.push("H".to_string());
        }
        if self.reverse {
            status_parts.push("R".to_string());
        }
        self.draw_text(&status_parts.join(" | "), x_pos, y_offset, font_size);
        y_offset += line_height;

        y_offset += line_height / 2.0; // Spacing

        // ✅ Subphase 12.6.2: Render nearby vehicles list
        if !self.nearby_vehicles.is_empty() {
            self.draw_text("Nearby Vehicles:", x_pos, y_offset, font_size);
            y_offset += line_height;

            for (name, distance) in &self.nearby_vehicles {
                self.draw_text(
                    &format!("  {} ({:.0}m)", name, distance),
                    x_pos,
                    y_offset,
                    font_size,
                );
                y_offset += line_height;
            }
        }

        // TODO Phase 7.3: Render Ackermann info if enabled

        Ok(())
    }

    /// Draw a horizontal bar for control values
    #[allow(clippy::too_many_arguments)]
    fn draw_bar(
        &self,
        label: &str,
        value: f32,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        color: Color,
    ) {
        // Draw label
        draw_text(label, x, y - 2.0, 12.0, WHITE);

        // Draw background
        draw_rectangle(
            x + 55.0,
            y - height,
            width,
            height,
            Color::new(0.2, 0.2, 0.2, 0.8),
        );

        // Draw filled portion
        let fill_width = value.clamp(0.0, 1.0) * width;
        if fill_width > 0.0 {
            draw_rectangle(x + 55.0, y - height, fill_width, height, color);
        }
    }

    /// Draw steering bar (centered at 0)
    fn draw_steer_bar(&self, x: f32, y: f32, width: f32, height: f32) {
        draw_text("Steer", x, y - 2.0, 12.0, WHITE);

        let bar_x = x + 55.0;

        // Draw background
        draw_rectangle(
            bar_x,
            y - height,
            width,
            height,
            Color::new(0.2, 0.2, 0.2, 0.8),
        );

        // Draw center line
        draw_rectangle(bar_x + width / 2.0, y - height, 1.0, height, WHITE);

        // Draw steer indicator
        let steer_offset = self.steer * (width / 2.0);
        let indicator_x = bar_x + width / 2.0 + steer_offset;
        let indicator_width = 3.0;
        draw_rectangle(
            indicator_x - indicator_width / 2.0,
            y - height,
            indicator_width,
            height,
            BLUE,
        );
    }

    /// Helper method to draw text with consistent styling
    fn draw_text(&self, text: &str, x: f32, y: f32, font_size: f32) {
        draw_text(text, x, y, font_size, WHITE);
    }

    /// Toggle Hud info display
    ///
    /// TODO Phase 12.3: F1 key functionality
    #[allow(dead_code)]
    pub fn toggle_info(&mut self) {
        self.show_info = !self.show_info;
    }

    /// Show/hide Ackermann controller info
    ///
    /// TODO Phase 7.3: Called when F key pressed
    #[allow(dead_code)]
    pub fn show_ackermann_info(&mut self, _enabled: bool) {
        // TODO: self.show_ackermann_info = enabled;
    }
}

impl Default for Hud {
    fn default() -> Self {
        Self::new(1280, 720)
    }
}
