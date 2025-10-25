//! Centralized type registry for all known types

use std::collections::{HashMap, HashSet};

/// Information about a registered type
#[derive(Debug, Clone)]
pub struct TypeInfo {
    /// Full Rust path to the type
    pub rust_path: String,
    /// Module where the type is defined
    pub module: String,
    /// Whether this type needs to be imported
    pub needs_import: bool,
    /// Import statement if needed
    pub import_statement: Option<String>,
    /// Whether this type can derive Copy
    pub is_copy: bool,
    /// Type aliases for this type
    pub aliases: Vec<String>,
}

/// Centralized registry of all known types
#[derive(Debug)]
pub struct TypeRegistry {
    /// All registered types by name
    types: HashMap<String, TypeInfo>,
    /// Standard Rust types that don't need prefixing
    standard_types: HashSet<String>,
    /// Type aliases mapping
    aliases: HashMap<String, String>,
}

impl TypeRegistry {
    /// Create a new type registry
    pub fn new() -> Self {
        let mut registry = Self {
            types: HashMap::new(),
            standard_types: HashSet::new(),
            aliases: HashMap::new(),
        };

        registry.init_standard_types();
        registry.init_carla_types();
        registry
    }

    /// Initialize standard Rust types
    fn init_standard_types(&mut self) {
        // Primitive types
        let primitives = [
            "bool", "i8", "i16", "i32", "i64", "i128", "isize", "u8", "u16", "u32", "u64", "u128",
            "usize", "f32", "f64", "char", "str",
        ];
        self.standard_types
            .extend(primitives.iter().map(|s| s.to_string()));

        // Standard library types
        let std_types = [
            "String", "Vec", "Option", "Result", "Box", "HashMap", "HashSet", "BTreeMap",
            "BTreeSet", "Rc", "Arc", "RefCell", "Cell", "Mutex", "RwLock",
        ];
        self.standard_types
            .extend(std_types.iter().map(|s| s.to_string()));

        // Register with full info
        self.register_type(
            "String",
            TypeInfo {
                rust_path: "String".to_string(),
                module: "std::string".to_string(),
                needs_import: false,
                import_statement: None,
                is_copy: false,
                aliases: vec!["std::string::String".to_string()],
            },
        );

        self.register_type(
            "HashMap",
            TypeInfo {
                rust_path: "HashMap".to_string(),
                module: "std::collections".to_string(),
                needs_import: true,
                import_statement: Some("use std::collections::HashMap;".to_string()),
                is_copy: false,
                aliases: vec!["std::collections::HashMap".to_string()],
            },
        );

        self.register_type(
            "HashSet",
            TypeInfo {
                rust_path: "HashSet".to_string(),
                module: "std::collections".to_string(),
                needs_import: true,
                import_statement: Some("use std::collections::HashSet;".to_string()),
                is_copy: false,
                aliases: vec!["std::collections::HashSet".to_string()],
            },
        );
    }

    /// Initialize CARLA types
    fn init_carla_types(&mut self) {
        // Core types
        let core_types = [
            ("Actor", true),
            ("Vehicle", false),
            ("Walker", false),
            ("Sensor", false),
            ("TrafficLight", false),
            ("TrafficSign", false),
            ("World", false),
            ("Client", false),
            ("Map", false),
            ("Waypoint", true),
            ("Junction", true),
            ("Landmark", true),
            ("Transform", true),
            ("Location", true),
            ("Rotation", true),
            ("Vector3D", true),
            ("Vector2D", true),
            ("GeoLocation", true),
            ("BoundingBox", true),
        ];

        for (type_name, is_copy) in &core_types {
            self.register_type(
                type_name,
                TypeInfo {
                    rust_path: format!("crate::carla::{type_name}"),
                    module: "crate::carla".to_string(),
                    needs_import: false,
                    import_statement: None,
                    is_copy: *is_copy,
                    aliases: vec![format!("carla.{}", type_name), type_name.to_string()],
                },
            );
        }

        // Enums
        let enum_types = [
            "ActorState",
            "TrafficLightState",
            "VehicleLightState",
            "VehicleDoor",
            "VehicleWheelLocation",
            "LaneType",
            "LaneChange",
            "LaneMarkingType",
            "LaneMarkingColor",
            "AttachmentType",
            "CityObjectLabel",
            "MaterialParameter",
            "MapLayer",
        ];

        for type_name in &enum_types {
            self.register_type(
                type_name,
                TypeInfo {
                    rust_path: format!("crate::carla::{type_name}"),
                    module: "crate::carla".to_string(),
                    needs_import: false,
                    import_statement: None,
                    is_copy: true, // Enums are usually Copy
                    aliases: vec![format!("carla.{}", type_name)],
                },
            );
        }

        // Special types
        self.register_type(
            "ActorOrId",
            TypeInfo {
                rust_path: "crate::carla::ActorOrId".to_string(),
                module: "crate::carla".to_string(),
                needs_import: false,
                import_statement: None,
                is_copy: false,
                aliases: vec![],
            },
        );

        // Type aliases
        self.aliases
            .insert("ActorId".to_string(), "i32".to_string());
        self.aliases
            .insert("Timestamp".to_string(), "f64".to_string());
        self.aliases.insert(
            "AckermannVehicleControl".to_string(),
            "VehicleAckermannControl".to_string(),
        );
        self.aliases.insert(
            "GBufferTextureID".to_string(),
            "GBufferTextureId".to_string(),
        );
    }

    /// Register a new type
    pub fn register_type(&mut self, name: &str, info: TypeInfo) {
        // Also register all aliases
        for alias in &info.aliases {
            self.types.insert(alias.clone(), info.clone());
        }
        self.types.insert(name.to_string(), info);
    }

    /// Look up a type by name
    pub fn lookup(&self, name: &str) -> Option<&TypeInfo> {
        // Check aliases first
        if let Some(real_name) = self.aliases.get(name) {
            return self.types.get(real_name);
        }

        // Direct lookup
        self.types.get(name)
    }

    /// Check if a type is a standard Rust type
    pub fn is_standard_type(&self, name: &str) -> bool {
        self.standard_types.contains(name)
    }

    /// Get the correct Rust path for a type
    pub fn get_rust_path(&self, name: &str) -> Option<String> {
        self.lookup(name).map(|info| info.rust_path.clone())
    }

    /// Get all imports needed for a set of types
    pub fn collect_imports<'a>(&self, types: impl Iterator<Item = &'a str>) -> Vec<String> {
        let mut imports = HashSet::new();

        for type_name in types {
            if let Some(info) = self.lookup(type_name) {
                if let Some(ref import) = info.import_statement {
                    imports.insert(import.clone());
                }
            }
        }

        let mut sorted: Vec<_> = imports.into_iter().collect();
        sorted.sort();
        sorted
    }

    /// Check if a type can derive Copy
    pub fn is_copyable(&self, name: &str) -> bool {
        self.lookup(name).map(|info| info.is_copy).unwrap_or(false)
    }

    /// Resolve a type alias
    pub fn resolve_alias(&self, alias: &str) -> String {
        self.aliases
            .get(alias)
            .cloned()
            .unwrap_or_else(|| alias.to_string())
    }
}

impl Default for TypeRegistry {
    fn default() -> Self {
        Self::new()
    }
}
