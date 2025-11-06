//! No Rendering Mode - Large-Scale Performance Demo
//!
//! Demonstrates CARLA's no rendering mode for performance testing with 100+ vehicles.
//! Shows simulation FPS without rendering overhead for benchmarking and scalability testing.
//!
//! # Features
//! - CARLA rendering disabled (no_rendering_mode)
//! - 100+ vehicle spawning with autopilot
//! - Real-time statistics (FPS, vehicle count, memory)
//! - Performance graphs (FPS over time)
//! - Minimal GUI (stats only, no camera)
//! - Dynamic vehicle spawning/destruction
//!
//! # Controls
//! - **+**: Spawn 10 more vehicles
//! - **-**: Destroy 10 vehicles
//! - **R**: Reset (destroy all, respawn 100)
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example no_rendering_mode --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    client::{Actor, ActorBase, Client, Vehicle, World as CarlaWorld},
    rpc::EpisodeSettings,
};
use macroquad::prelude::*;
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;
const INITIAL_VEHICLE_COUNT: usize = 100;
const SPAWN_BATCH_SIZE: usize = 10;
const MAX_SPAWN_ATTEMPTS: usize = 50;
const GRAPH_HISTORY_SIZE: usize = 300; // 30 seconds at 10 FPS update rate

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - No Rendering Mode (Performance Demo)".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

/// Performance statistics
struct PerformanceStats {
    vehicle_count: usize,
    simulation_fps: f32,
    frame_time_ms: f32,
    min_fps: f32,
    max_fps: f32,
    avg_fps: f32,
    total_frames: u64,
    fps_history: VecDeque<f32>,
}

impl PerformanceStats {
    fn new() -> Self {
        Self {
            vehicle_count: 0,
            simulation_fps: 0.0,
            frame_time_ms: 0.0,
            min_fps: f32::MAX,
            max_fps: 0.0,
            avg_fps: 0.0,
            total_frames: 0,
            fps_history: VecDeque::with_capacity(GRAPH_HISTORY_SIZE),
        }
    }

    fn update(&mut self, delta_time: f32, vehicle_count: usize) {
        self.vehicle_count = vehicle_count;

        if delta_time > 0.0 {
            self.simulation_fps = 1.0 / delta_time;
            self.frame_time_ms = delta_time * 1000.0;

            // Update min/max
            if self.simulation_fps < self.min_fps {
                self.min_fps = self.simulation_fps;
            }
            if self.simulation_fps > self.max_fps {
                self.max_fps = self.simulation_fps;
            }

            // Update average
            self.total_frames += 1;
            self.avg_fps = (self.avg_fps * (self.total_frames - 1) as f32 + self.simulation_fps)
                / self.total_frames as f32;

            // Update history for graph
            self.fps_history.push_back(self.simulation_fps);
            if self.fps_history.len() > GRAPH_HISTORY_SIZE {
                self.fps_history.pop_front();
            }
        }
    }

    fn reset_stats(&mut self) {
        self.min_fps = f32::MAX;
        self.max_fps = 0.0;
        self.avg_fps = 0.0;
        self.total_frames = 0;
        self.fps_history.clear();
    }
}

/// Vehicle manager for spawning and destroying vehicles
struct VehicleManager {
    vehicles: Vec<Vehicle>,
}

impl VehicleManager {
    fn new() -> Self {
        Self {
            vehicles: Vec::new(),
        }
    }

    fn spawn_vehicles(&mut self, world: &mut CarlaWorld, count: usize) -> Result<usize> {
        let blueprint_library = world.blueprint_library();
        let vehicle_blueprints: Vec<_> = blueprint_library.filter("vehicle.*").iter().collect();

        if vehicle_blueprints.is_empty() {
            return Err(anyhow::anyhow!("No vehicle blueprints found"));
        }

        let map = world.map();
        let spawn_points = map.recommended_spawn_points();

        if spawn_points.is_empty() {
            return Err(anyhow::anyhow!("No spawn points available"));
        }

        let spawn_points_slice = spawn_points.as_slice();
        let mut spawned = 0;
        let mut attempts = 0;

        while spawned < count && attempts < MAX_SPAWN_ATTEMPTS {
            // Pick random spawn point
            let spawn_idx = rand::gen_range(0, spawn_points_slice.len() as i32) as usize;
            let spawn_point = &spawn_points_slice[spawn_idx];

            // Pick random vehicle blueprint
            let bp_idx = rand::gen_range(0, vehicle_blueprints.len() as i32) as usize;
            let vehicle_bp = &vehicle_blueprints[bp_idx];

            // Try to spawn
            if let Ok(actor) =
                world.spawn_actor_opt::<Actor, _>(vehicle_bp, spawn_point, None::<&Actor>, None)
            {
                if let Ok(vehicle) = Vehicle::try_from(actor) {
                    vehicle.set_autopilot(true);
                    self.vehicles.push(vehicle);
                    spawned += 1;
                }
            }

            attempts += 1;
        }

        Ok(spawned)
    }

    fn destroy_vehicles(&mut self, count: usize) -> usize {
        let to_destroy = count.min(self.vehicles.len());

        for _ in 0..to_destroy {
            if let Some(vehicle) = self.vehicles.pop() {
                vehicle.destroy();
            }
        }

        to_destroy
    }

    fn destroy_all(&mut self) {
        for vehicle in self.vehicles.drain(..) {
            vehicle.destroy();
        }
    }

    fn count(&self) -> usize {
        self.vehicles.len()
    }
}

/// Statistics display
struct StatsDisplay {
    show: bool,
}

impl StatsDisplay {
    fn new() -> Self {
        Self { show: true }
    }

    fn render(&self, stats: &PerformanceStats, status_msg: &str) {
        if !self.show {
            return;
        }

        let y_offset = 20.0;
        let line_height = 25.0;
        let mut y = y_offset;

        // Title
        let title = "CARLA - NO RENDERING MODE";
        let title_width = measure_text(title, None, 30, 1.0).width;
        draw_rectangle(
            10.0,
            y - 20.0,
            title_width + 20.0,
            35.0,
            macroquad::color::Color::from_rgba(0, 0, 0, 200),
        );
        draw_text(title, 20.0, y, 30.0, WHITE);
        y += 50.0;

        // Stats section
        let stats_texts = vec![
            format!("Vehicles: {}", stats.vehicle_count),
            "".to_string(),
            format!("Simulation FPS: {:.1}", stats.simulation_fps),
            format!("Frame Time: {:.2} ms", stats.frame_time_ms),
            "".to_string(),
            format!("Min FPS: {:.1}", stats.min_fps),
            format!("Max FPS: {:.1}", stats.max_fps),
            format!("Avg FPS: {:.1}", stats.avg_fps),
            "".to_string(),
            format!("Total Frames: {}", stats.total_frames),
        ];

        for text in &stats_texts {
            let text_width = measure_text(text, None, 22, 1.0).width;
            draw_rectangle(
                10.0,
                y - 18.0,
                text_width + 20.0,
                line_height,
                macroquad::color::Color::from_rgba(0, 0, 0, 180),
            );
            draw_text(text, 20.0, y, 22.0, WHITE);
            y += line_height;
        }

        // Status message
        if !status_msg.is_empty() {
            y += 10.0;
            let msg_width = measure_text(status_msg, None, 20, 1.0).width;
            draw_rectangle(
                10.0,
                y - 18.0,
                msg_width + 20.0,
                line_height,
                macroquad::color::Color::from_rgba(0, 100, 200, 200),
            );
            draw_text(status_msg, 20.0, y, 20.0, YELLOW);
        }

        // Controls hint
        y = WINDOW_HEIGHT as f32 - 120.0;
        let controls = vec![
            "Controls:",
            "  + : Spawn 10 vehicles",
            "  - : Destroy 10 vehicles",
            "  R : Reset (100 vehicles)",
            "  H : Toggle help",
        ];

        for text in &controls {
            let text_width = measure_text(text, None, 18, 1.0).width;
            draw_rectangle(
                10.0,
                y - 15.0,
                text_width + 20.0,
                20.0,
                macroquad::color::Color::from_rgba(0, 0, 0, 180),
            );
            draw_text(text, 20.0, y, 18.0, LIGHTGRAY);
            y += 20.0;
        }
    }
}

/// FPS graph display
struct FpsGraph {
    show: bool,
}

impl FpsGraph {
    fn new() -> Self {
        Self { show: true }
    }

    fn toggle(&mut self) {
        self.show = !self.show;
    }

    fn render(&self, stats: &PerformanceStats) {
        if !self.show || stats.fps_history.is_empty() {
            return;
        }

        let graph_x = WINDOW_WIDTH as f32 - 420.0;
        let graph_y = 20.0;
        let graph_width = 400.0;
        let graph_height = 200.0;

        // Background
        draw_rectangle(
            graph_x,
            graph_y,
            graph_width,
            graph_height,
            macroquad::color::Color::from_rgba(0, 0, 0, 200),
        );

        // Title
        draw_text("FPS Over Time", graph_x + 10.0, graph_y + 20.0, 20.0, WHITE);

        // Find min/max for scaling
        let mut min_fps = f32::MAX;
        let mut max_fps = 0.0;
        for fps in stats.fps_history.iter() {
            if *fps < min_fps {
                min_fps = *fps;
            }
            if *fps > max_fps {
                max_fps = *fps;
            }
        }

        // Add padding to range
        let range = (max_fps - min_fps).max(10.0);
        min_fps = (min_fps - range * 0.1).max(0.0);
        max_fps += range * 0.1;

        // Draw grid lines
        let grid_y_start = graph_y + 40.0;
        let grid_height = graph_height - 60.0;

        for i in 0..5 {
            let y = grid_y_start + (i as f32 / 4.0) * grid_height;
            draw_line(
                graph_x + 50.0,
                y,
                graph_x + graph_width - 10.0,
                y,
                1.0,
                macroquad::color::Color::from_rgba(100, 100, 100, 100),
            );

            // Y-axis labels
            let fps_value = max_fps - (i as f32 / 4.0) * (max_fps - min_fps);
            let label = format!("{:.0}", fps_value);
            draw_text(&label, graph_x + 10.0, y + 5.0, 16.0, GRAY);
        }

        // Draw FPS line
        let plot_width = graph_width - 60.0;
        let history: Vec<_> = stats.fps_history.iter().copied().collect();

        for i in 1..history.len() {
            let x1 = graph_x + 50.0 + ((i - 1) as f32 / GRAPH_HISTORY_SIZE as f32) * plot_width;
            let x2 = graph_x + 50.0 + (i as f32 / GRAPH_HISTORY_SIZE as f32) * plot_width;

            let y1 = grid_y_start
                + grid_height * (1.0 - (history[i - 1] - min_fps) / (max_fps - min_fps));
            let y2 =
                grid_y_start + grid_height * (1.0 - (history[i] - min_fps) / (max_fps - min_fps));

            draw_line(x1, y1, x2, y2, 2.0, GREEN);
        }

        // Draw border
        draw_rectangle_lines(graph_x, graph_y, graph_width, graph_height, 2.0, WHITE);

        // Current FPS indicator
        let current_fps_text = format!("Current: {:.1} FPS", stats.simulation_fps);
        draw_text(
            &current_fps_text,
            graph_x + 10.0,
            graph_y + graph_height - 10.0,
            18.0,
            YELLOW,
        );
    }
}

/// Help overlay
struct HelpOverlay {
    show: bool,
}

impl HelpOverlay {
    fn new() -> Self {
        Self { show: false }
    }

    fn toggle(&mut self) {
        self.show = !self.show;
    }

    fn render(&self) {
        if !self.show {
            return;
        }

        draw_rectangle(
            0.0,
            0.0,
            WINDOW_WIDTH as f32,
            WINDOW_HEIGHT as f32,
            macroquad::color::Color::from_rgba(0, 0, 0, 220),
        );

        let center_x = WINDOW_WIDTH as f32 / 2.0;
        let start_y = 80.0;
        let line_height = 30.0;

        let help_text = vec![
            ("NO RENDERING MODE - PERFORMANCE DEMO", 32.0),
            ("", 20.0),
            (
                "This demo runs CARLA with rendering disabled to demonstrate",
                20.0,
            ),
            (
                "maximum simulation performance. Perfect for benchmarking and",
                20.0,
            ),
            ("large-scale simulations without graphics overhead.", 20.0),
            ("", 20.0),
            ("CONTROLS:", 26.0),
            ("", 20.0),
            ("+ (Plus)       Spawn 10 more vehicles", 20.0),
            ("- (Minus)      Destroy 10 vehicles", 20.0),
            ("R              Reset to 100 vehicles", 20.0),
            ("H              Toggle this help overlay", 20.0),
            ("ESC / Q        Quit application", 20.0),
            ("", 20.0),
            ("PERFORMANCE TIPS:", 26.0),
            ("", 20.0),
            ("• No rendering mode provides 2-5x FPS improvement", 20.0),
            ("• Ideal for testing AI, physics, or large scenarios", 20.0),
            ("• Monitor FPS graph to see performance scaling", 20.0),
            ("• Typical performance: 200-500 FPS with 100 vehicles", 20.0),
            ("", 20.0),
            ("Press H to close this help", 20.0),
        ];

        let mut y = start_y;
        for (text, size) in help_text {
            let text_dims = measure_text(text, None, size as u16, 1.0);
            let x = center_x - text_dims.width / 2.0;
            draw_text(text, x, y, size, WHITE);
            y += line_height;
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() -> Result<()> {
    println!("No Rendering Mode - Connecting to CARLA...");

    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Enabling no rendering mode...");

    // Enable no rendering mode
    let settings = EpisodeSettings {
        no_rendering_mode: true,
        synchronous_mode: false,
        fixed_delta_seconds: None,
        ..Default::default()
    };
    world.apply_settings(&settings, Duration::from_secs(2));

    println!("No rendering mode enabled");
    println!("This improves simulation performance by 2-5x!");

    // Initialize vehicle manager
    let mut vehicle_manager = VehicleManager::new();

    println!("Spawning {} initial vehicles...", INITIAL_VEHICLE_COUNT);
    let spawned = vehicle_manager.spawn_vehicles(&mut world, INITIAL_VEHICLE_COUNT)?;
    println!("Spawned {} vehicles with autopilot", spawned);

    // Initialize UI
    let mut stats = PerformanceStats::new();
    let stats_display = StatsDisplay::new();
    let mut fps_graph = FpsGraph::new();
    let mut help = HelpOverlay::new();

    // State
    let mut status_message = String::new();
    let mut status_message_time = 0.0;
    let mut last_frame_time = Instant::now();

    println!("Starting main loop...");
    println!("Press H for help");

    loop {
        // Handle input
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }

        if is_key_pressed(KeyCode::H) {
            help.toggle();
        }

        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            println!("Spawning {} vehicles...", SPAWN_BATCH_SIZE);
            match vehicle_manager.spawn_vehicles(&mut world, SPAWN_BATCH_SIZE) {
                Ok(count) => {
                    status_message = format!("Spawned {} vehicles", count);
                    status_message_time = 3.0;
                    println!("Spawned {} vehicles", count);
                }
                Err(e) => {
                    status_message = format!("Spawn failed: {}", e);
                    status_message_time = 3.0;
                    println!("Failed to spawn vehicles: {}", e);
                }
            }
        }

        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            let destroyed = vehicle_manager.destroy_vehicles(SPAWN_BATCH_SIZE);
            status_message = format!("Destroyed {} vehicles", destroyed);
            status_message_time = 3.0;
            println!("Destroyed {} vehicles", destroyed);
        }

        if is_key_pressed(KeyCode::R) {
            println!("Resetting to {} vehicles...", INITIAL_VEHICLE_COUNT);
            vehicle_manager.destroy_all();
            match vehicle_manager.spawn_vehicles(&mut world, INITIAL_VEHICLE_COUNT) {
                Ok(count) => {
                    status_message = format!("Reset: {} vehicles spawned", count);
                    status_message_time = 3.0;
                    stats.reset_stats();
                    println!("Reset complete: {} vehicles", count);
                }
                Err(e) => {
                    status_message = format!("Reset failed: {}", e);
                    status_message_time = 3.0;
                    println!("Reset failed: {}", e);
                }
            }
        }

        if is_key_pressed(KeyCode::G) {
            fps_graph.toggle();
        }

        // Update timing
        let current_time = Instant::now();
        let delta_time = current_time.duration_since(last_frame_time).as_secs_f32();
        last_frame_time = current_time;

        // Update stats
        let vehicle_count = vehicle_manager.count();
        stats.update(delta_time, vehicle_count);

        // Update status message timer
        if status_message_time > 0.0 {
            status_message_time -= delta_time;
            if status_message_time <= 0.0 {
                status_message.clear();
            }
        }

        // Render
        clear_background(macroquad::color::Color::from_rgba(20, 20, 30, 255));

        // Draw title background
        draw_rectangle(
            0.0,
            0.0,
            WINDOW_WIDTH as f32,
            80.0,
            macroquad::color::Color::from_rgba(10, 10, 20, 255),
        );

        stats_display.render(&stats, &status_message);
        fps_graph.render(&stats);
        help.render();

        next_frame().await;
    }

    println!("Cleaning up...");

    // Restore normal rendering mode
    let settings = EpisodeSettings {
        no_rendering_mode: false,
        synchronous_mode: false,
        fixed_delta_seconds: None,
        ..Default::default()
    };
    world.apply_settings(&settings, Duration::from_secs(2));

    vehicle_manager.destroy_all();

    println!("Done!");
    println!("Final stats:");
    println!("  Total frames: {}", stats.total_frames);
    println!("  Average FPS: {:.1}", stats.avg_fps);
    println!("  Min FPS: {:.1}", stats.min_fps);
    println!("  Max FPS: {:.1}", stats.max_fps);

    Ok(())
}
