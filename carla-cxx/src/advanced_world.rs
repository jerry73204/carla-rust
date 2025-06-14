//! Advanced world features for CARLA simulator.
//!
//! This module provides access to advanced world functionality including:
//! - Map layer management for dynamic level loading
//! - Environment object queries and manipulation
//! - Texture and material application
//! - Advanced traffic light management

use crate::{
    ffi::{
        bridge::*, MAP_LAYER_ALL, MAP_LAYER_BUILDINGS, MAP_LAYER_DECALS, MAP_LAYER_FOLIAGE,
        MAP_LAYER_GROUND, MAP_LAYER_NONE, MAP_LAYER_PARKED_VEHICLES, MAP_LAYER_PARTICLES,
        MAP_LAYER_PROPS, MAP_LAYER_STREET_LIGHTS, MAP_LAYER_WALLS,
    },
    world_interaction::CityObjectLabel,
    WorldWrapper,
};
use std::collections::HashMap;

/// Map layer flags for selective loading/unloading
pub struct MapLayers {
    bits: u8,
}

impl MapLayers {
    pub const NONE: Self = Self {
        bits: MAP_LAYER_NONE,
    };
    pub const BUILDINGS: Self = Self {
        bits: MAP_LAYER_BUILDINGS,
    };
    pub const DECALS: Self = Self {
        bits: MAP_LAYER_DECALS,
    };
    pub const FOLIAGE: Self = Self {
        bits: MAP_LAYER_FOLIAGE,
    };
    pub const GROUND: Self = Self {
        bits: MAP_LAYER_GROUND,
    };
    pub const PARKED_VEHICLES: Self = Self {
        bits: MAP_LAYER_PARKED_VEHICLES,
    };
    pub const PARTICLES: Self = Self {
        bits: MAP_LAYER_PARTICLES,
    };
    pub const PROPS: Self = Self {
        bits: MAP_LAYER_PROPS,
    };
    pub const STREET_LIGHTS: Self = Self {
        bits: MAP_LAYER_STREET_LIGHTS,
    };
    pub const WALLS: Self = Self {
        bits: MAP_LAYER_WALLS,
    };
    pub const ALL: Self = Self {
        bits: MAP_LAYER_ALL,
    };

    // Aliases for different naming conventions
    pub const ROADS: Self = Self {
        bits: MAP_LAYER_GROUND,
    };
    pub const SIDEWALKS: Self = Self {
        bits: MAP_LAYER_GROUND,
    };
    pub const TRAFFIC_SIGNS: Self = Self {
        bits: MAP_LAYER_PROPS,
    };

    /// Create from raw bits
    pub fn from_bits(bits: u8) -> Self {
        Self { bits }
    }

    /// Combine multiple layers
    pub fn union(self, other: Self) -> Self {
        Self {
            bits: self.bits | other.bits,
        }
    }

    /// Check if contains specific layers
    pub fn contains(self, other: Self) -> bool {
        (self.bits & other.bits) == other.bits
    }
}

/// Material parameter types for texture application
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MaterialParameter {
    Diffuse = 0,
    Emissive = 1,
    Normal = 2,
    AORoughnessMetallicEmissive = 3,
}

/// Texture builder for creating color textures
pub struct TextureColorBuilder {
    width: u32,
    height: u32,
    data: Vec<u8>,
}

impl TextureColorBuilder {
    /// Create a new texture with specified dimensions
    pub fn new(width: u32, height: u32) -> Self {
        let size = (width * height * 4) as usize; // RGBA
        Self {
            width,
            height,
            data: vec![0; size],
        }
    }

    /// Set pixel color at coordinates
    pub fn set_pixel(&mut self, x: u32, y: u32, r: u8, g: u8, b: u8, a: u8) {
        if x >= self.width || y >= self.height {
            return;
        }
        let idx = ((y * self.width + x) * 4) as usize;
        self.data[idx] = r;
        self.data[idx + 1] = g;
        self.data[idx + 2] = b;
        self.data[idx + 3] = a;
    }

    /// Fill entire texture with a color
    pub fn fill(&mut self, r: u8, g: u8, b: u8, a: u8) {
        for y in 0..self.height {
            for x in 0..self.width {
                self.set_pixel(x, y, r, g, b, a);
            }
        }
    }

    /// Build the texture
    pub fn build(self) -> SimpleTextureColor {
        SimpleTextureColor {
            width: self.width,
            height: self.height,
            data: self.data,
        }
    }
}

/// Float texture builder for HDR textures
pub struct TextureFloatColorBuilder {
    width: u32,
    height: u32,
    data: Vec<f32>,
}

impl TextureFloatColorBuilder {
    /// Create a new float texture with specified dimensions
    pub fn new(width: u32, height: u32) -> Self {
        let size = (width * height * 4) as usize; // RGBA
        Self {
            width,
            height,
            data: vec![0.0; size],
        }
    }

    /// Set pixel color at coordinates with HDR values
    pub fn set_pixel(&mut self, x: u32, y: u32, r: f32, g: f32, b: f32, a: f32) {
        if x >= self.width || y >= self.height {
            return;
        }
        let idx = ((y * self.width + x) * 4) as usize;
        self.data[idx] = r;
        self.data[idx + 1] = g;
        self.data[idx + 2] = b;
        self.data[idx + 3] = a;
    }

    /// Build the texture
    pub fn build(self) -> SimpleTextureFloatColor {
        SimpleTextureFloatColor {
            width: self.width,
            height: self.height,
            data: self.data,
        }
    }
}

/// Extension trait for WorldWrapper to add advanced features
pub trait AdvancedWorldExt {
    /// Load specific map layers dynamically
    fn load_map_layers(&self, layers: MapLayers);

    /// Unload specific map layers to save memory
    fn unload_map_layers(&self, layers: MapLayers);

    /// Get bounding boxes of all level elements with specific tag
    fn get_level_bounding_boxes(&self, tag: CityObjectLabel) -> Vec<SimpleLevelBoundingBox>;

    /// Get environment objects with specific tag  
    fn query_environment_objects(&self, tag: CityObjectLabel) -> Vec<SimpleEnvironmentObject>;

    /// Enable or disable specific environment objects
    fn set_environment_objects_enabled(&self, object_ids: &[u64], enabled: bool);

    /// Reset all traffic lights to their initial state
    fn reset_all_traffic_lights(&self);

    /// Freeze or unfreeze all traffic lights
    fn freeze_all_traffic_lights(&self, frozen: bool);

    /// Get light states of all vehicles
    fn get_all_vehicle_light_states(&self) -> HashMap<u32, u32>;

    /// Apply color texture to a named object
    fn apply_color_texture(
        &self,
        object_name: &str,
        texture: &SimpleTextureColor,
        material: MaterialParameter,
    );

    /// Apply HDR float color texture to a named object
    fn apply_float_color_texture(
        &self,
        object_name: &str,
        texture: &SimpleTextureFloatColor,
        material: MaterialParameter,
    );

    /// Get names of all objects that can have textures applied
    fn get_texturable_object_names(&self) -> Vec<String>;

    /// Set the random seed for pedestrian navigation
    fn set_pedestrian_navigation_seed(&self, seed: u32);
}

impl AdvancedWorldExt for WorldWrapper {
    fn load_map_layers(&self, layers: MapLayers) {
        self.load_level_layer(layers.bits);
    }

    fn unload_map_layers(&self, layers: MapLayers) {
        self.unload_level_layer(layers.bits);
    }

    fn get_level_bounding_boxes(&self, tag: CityObjectLabel) -> Vec<SimpleLevelBoundingBox> {
        self.get_level_bbs(tag as u8)
    }

    fn query_environment_objects(&self, tag: CityObjectLabel) -> Vec<SimpleEnvironmentObject> {
        self.get_environment_objects(tag as u8)
    }

    fn set_environment_objects_enabled(&self, object_ids: &[u64], enabled: bool) {
        self.enable_environment_objects(object_ids, enabled);
    }

    fn reset_all_traffic_lights(&self) {
        self.reset_all_traffic_lights();
    }

    fn freeze_all_traffic_lights(&self, frozen: bool) {
        self.freeze_all_traffic_lights(frozen);
    }

    fn get_all_vehicle_light_states(&self) -> HashMap<u32, u32> {
        let states = self.get_vehicles_light_states();
        let mut result = HashMap::new();

        for state in states {
            if state.command_type == 99 {
                // Special marker we set in C++
                result.insert(state.actor_id, state.int_flag1 as u32);
            }
        }

        result
    }

    fn apply_color_texture(
        &self,
        object_name: &str,
        texture: &SimpleTextureColor,
        material: MaterialParameter,
    ) {
        self.apply_color_texture_to_object(object_name, texture, material as u8);
    }

    fn apply_float_color_texture(
        &self,
        object_name: &str,
        texture: &SimpleTextureFloatColor,
        material: MaterialParameter,
    ) {
        self.apply_float_color_texture_to_object(object_name, texture, material as u8);
    }

    fn get_texturable_object_names(&self) -> Vec<String> {
        self.get_names_of_all_objects()
    }

    fn set_pedestrian_navigation_seed(&self, seed: u32) {
        self.set_pedestrians_seed(seed);
    }
}

/// Helper functions for creating common textures
pub mod texture_utils {
    use super::*;

    /// Create a solid color texture
    pub fn solid_color(width: u32, height: u32, r: u8, g: u8, b: u8) -> SimpleTextureColor {
        let mut builder = TextureColorBuilder::new(width, height);
        builder.fill(r, g, b, 255);
        builder.build()
    }

    /// Create a checkerboard pattern texture
    pub fn checkerboard(
        width: u32,
        height: u32,
        size: u32,
        color1: (u8, u8, u8),
        color2: (u8, u8, u8),
    ) -> SimpleTextureColor {
        let mut builder = TextureColorBuilder::new(width, height);

        for y in 0..height {
            for x in 0..width {
                let checker = ((x / size) + (y / size)) % 2 == 0;
                let (r, g, b) = if checker { color1 } else { color2 };
                builder.set_pixel(x, y, r, g, b, 255);
            }
        }

        builder.build()
    }

    /// Create a gradient texture
    pub fn gradient(
        width: u32,
        height: u32,
        start_color: (u8, u8, u8),
        end_color: (u8, u8, u8),
        horizontal: bool,
    ) -> SimpleTextureColor {
        let mut builder = TextureColorBuilder::new(width, height);

        for y in 0..height {
            for x in 0..width {
                let t = if horizontal {
                    x as f32 / (width - 1) as f32
                } else {
                    y as f32 / (height - 1) as f32
                };

                let r = (start_color.0 as f32 * (1.0 - t) + end_color.0 as f32 * t) as u8;
                let g = (start_color.1 as f32 * (1.0 - t) + end_color.1 as f32 * t) as u8;
                let b = (start_color.2 as f32 * (1.0 - t) + end_color.2 as f32 * t) as u8;

                builder.set_pixel(x, y, r, g, b, 255);
            }
        }

        builder.build()
    }
}
