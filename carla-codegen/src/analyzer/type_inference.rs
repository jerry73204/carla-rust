//! Smart type inference for untyped parameters

use super::{RustType, TypeContext, TypeRegistry};
use crate::parser::yaml_schema::Parameter;
use std::collections::HashMap;

/// Smart type inference engine
pub struct TypeInferrer {
    /// Type registry for lookups
    registry: TypeRegistry,
    /// Parameter patterns database
    patterns: HashMap<String, String>,
}

impl TypeInferrer {
    /// Create a new type inferrer
    pub fn new(registry: TypeRegistry) -> Self {
        let mut inferrer = Self {
            registry,
            patterns: HashMap::new(),
        };
        inferrer.init_patterns();
        inferrer
    }

    /// Initialize parameter name patterns
    fn init_patterns(&mut self) {
        // ID patterns
        self.patterns
            .insert("actor_id".to_string(), "i32".to_string());
        self.patterns
            .insert("actor_ids".to_string(), "Vec<i32>".to_string());
        self.patterns.insert("id".to_string(), "i32".to_string());
        self.patterns
            .insert("ids".to_string(), "Vec<i32>".to_string());
        self.patterns
            .insert("vehicle_id".to_string(), "i32".to_string());
        self.patterns
            .insert("walker_id".to_string(), "i32".to_string());

        // Actor patterns
        self.patterns
            .insert("actor".to_string(), "crate::carla::Actor".to_string());
        self.patterns
            .insert("actors".to_string(), "Vec<crate::carla::Actor>".to_string());
        self.patterns
            .insert("vehicle".to_string(), "crate::carla::Vehicle".to_string());
        self.patterns
            .insert("walker".to_string(), "crate::carla::Walker".to_string());
        self.patterns.insert(
            "parent".to_string(),
            "Option<Box<crate::carla::Actor>>".to_string(),
        );
        self.patterns
            .insert("attach_to".to_string(), "crate::carla::Actor".to_string());

        // Transform patterns
        self.patterns.insert(
            "transform".to_string(),
            "crate::carla::Transform".to_string(),
        );
        self.patterns
            .insert("location".to_string(), "crate::carla::Location".to_string());
        self.patterns
            .insert("rotation".to_string(), "crate::carla::Rotation".to_string());
        self.patterns
            .insert("velocity".to_string(), "crate::carla::Vector3D".to_string());
        self.patterns.insert(
            "angular_velocity".to_string(),
            "crate::carla::Vector3D".to_string(),
        );

        // Control patterns
        self.patterns.insert(
            "control".to_string(),
            "crate::carla::VehicleControl".to_string(),
        );
        self.patterns.insert(
            "ackermann_control".to_string(),
            "crate::carla::VehicleAckermannControl".to_string(),
        );
        self.patterns.insert(
            "walker_control".to_string(),
            "crate::carla::WalkerControl".to_string(),
        );
        self.patterns.insert(
            "physics_control".to_string(),
            "crate::carla::VehiclePhysicsControl".to_string(),
        );

        // Settings patterns
        self.patterns.insert(
            "settings".to_string(),
            "crate::carla::WorldSettings".to_string(),
        );
        self.patterns.insert(
            "weather".to_string(),
            "crate::carla::WeatherParameters".to_string(),
        );
        self.patterns.insert(
            "blueprint".to_string(),
            "crate::carla::ActorBlueprint".to_string(),
        );

        // Common patterns
        self.patterns
            .insert("distance".to_string(), "f32".to_string());
        self.patterns
            .insert("radius".to_string(), "f32".to_string());
        self.patterns.insert("angle".to_string(), "f32".to_string());
        self.patterns.insert("speed".to_string(), "f32".to_string());
        self.patterns.insert("time".to_string(), "f32".to_string());
        self.patterns
            .insert("seconds".to_string(), "f32".to_string());
        self.patterns
            .insert("percentage".to_string(), "f32".to_string());
        self.patterns
            .insert("intensity".to_string(), "f32".to_string());

        self.patterns
            .insert("enabled".to_string(), "bool".to_string());
        self.patterns
            .insert("enable".to_string(), "bool".to_string());
        self.patterns
            .insert("active".to_string(), "bool".to_string());
        self.patterns
            .insert("visible".to_string(), "bool".to_string());

        self.patterns
            .insert("name".to_string(), "String".to_string());
        self.patterns
            .insert("text".to_string(), "String".to_string());
        self.patterns
            .insert("message".to_string(), "String".to_string());
        self.patterns
            .insert("path".to_string(), "String".to_string());
        self.patterns
            .insert("filename".to_string(), "String".to_string());

        // Callback patterns
        self.patterns.insert(
            "callback".to_string(),
            "Box<dyn Fn() + Send + Sync>".to_string(),
        );
        self.patterns.insert(
            "on_tick".to_string(),
            "Box<dyn Fn(crate::carla::WorldSnapshot) + Send + Sync>".to_string(),
        );
    }

    /// Infer type for a parameter
    pub fn infer_parameter_type(
        &self,
        param: &Parameter,
        context: &TypeContext,
    ) -> Option<RustType> {
        // Special case: 'other' in comparison methods
        if param.param_name == "other" {
            if let Some(method_name) = context.method_name() {
                if method_name == "__eq__"
                    || method_name == "__ne__"
                    || method_name == "eq"
                    || method_name == "ne"
                {
                    // Return same type as containing class
                    let class_type = format!("crate::carla::{}", context.containing_class);
                    return Some(RustType::Custom(class_type));
                }
            }
        }

        // Check pattern database
        if let Some(type_str) = self.patterns.get(&param.param_name) {
            return self.parse_type_string(type_str);
        }

        // Check for plural forms
        if param.param_name.ends_with('s') {
            let singular = &param.param_name[..param.param_name.len() - 1];
            if let Some(type_str) = self.patterns.get(singular) {
                // Make it a Vec
                if let Some(rust_type) = self.parse_type_string(type_str) {
                    return Some(RustType::Vec(Box::new(rust_type)));
                }
            }
        }

        // Method-specific inference
        if let Some(method_name) = context.method_name() {
            match method_name {
                "spawn_actor" | "try_spawn_actor" => match param.param_name.as_str() {
                    "blueprint" => return self.parse_type_string("crate::carla::ActorBlueprint"),
                    "transform" => return self.parse_type_string("crate::carla::Transform"),
                    _ => {}
                },
                "apply_settings" => {
                    if param.param_name.contains("settings") {
                        return self.parse_type_string("crate::carla::WorldSettings");
                    }
                }
                _ => {}
            }
        }

        None
    }

    /// Parse a type string into RustType
    fn parse_type_string(&self, type_str: &str) -> Option<RustType> {
        // Handle Vec<T>
        if type_str.starts_with("Vec<") && type_str.ends_with('>') {
            let inner = &type_str[4..type_str.len() - 1];
            if let Some(inner_type) = self.parse_type_string(inner) {
                return Some(RustType::Vec(Box::new(inner_type)));
            }
        }

        // Handle Option<T>
        if type_str.starts_with("Option<") && type_str.ends_with('>') {
            let inner = &type_str[7..type_str.len() - 1];
            if let Some(inner_type) = self.parse_type_string(inner) {
                return Some(RustType::Option(Box::new(inner_type)));
            }
        }

        // Handle Box<T>
        if type_str.starts_with("Box<") && type_str.ends_with('>') {
            let _inner = &type_str[4..type_str.len() - 1];
            // For now, treat Box<T> as Custom type
            return Some(RustType::Custom(type_str.to_string()));
        }

        // Check if it's a primitive
        match type_str {
            "bool" | "i8" | "i16" | "i32" | "i64" | "i128" | "isize" | "u8" | "u16" | "u32"
            | "u64" | "u128" | "usize" | "f32" | "f64" | "char" => {
                Some(RustType::Primitive(type_str.to_string()))
            }
            "String" => Some(RustType::Custom("String".to_string())),
            _ => Some(RustType::Custom(type_str.to_string())),
        }
    }

    /// Infer return type based on method context
    pub fn infer_return_type(&self, method_name: &str, _context: &TypeContext) -> Option<RustType> {
        // Common patterns
        match method_name {
            "get_id" | "id" => return self.parse_type_string("i32"),
            "get_name" | "name" => return self.parse_type_string("String"),
            "get_transform" | "transform" => {
                return self.parse_type_string("crate::carla::Transform")
            }
            "get_location" | "location" => return self.parse_type_string("crate::carla::Location"),
            "get_velocity" | "velocity" => return self.parse_type_string("crate::carla::Vector3D"),
            _ => {}
        }

        // Check for getter pattern
        if let Some(property) = method_name.strip_prefix("get_") {
            if let Some(type_str) = self.patterns.get(property) {
                return self.parse_type_string(type_str);
            }
        }

        // Boolean methods
        if method_name.starts_with("is_")
            || method_name.starts_with("has_")
            || method_name.starts_with("can_")
        {
            return self.parse_type_string("bool");
        }

        // List methods
        if method_name.starts_with("list_") || method_name.ends_with("_list") {
            // Try to infer the element type
            if method_name.contains("actor") {
                return self.parse_type_string("Vec<crate::carla::Actor>");
            }
            // Default to Vec<String>
            return self.parse_type_string("Vec<String>");
        }

        None
    }
}
