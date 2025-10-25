//! Missing type definitions that need to be generated

/// Generate additional type definitions that are referenced but not in YAML
pub fn generate_missing_types() -> String {
    let mut content = String::new();

    // ActorOrId type - used for methods that accept either Actor or actor ID
    content.push_str(
        r#"
/// Type that can represent either an Actor reference or an actor ID
#[derive(Debug, Clone)]
pub enum ActorOrId {
    /// Direct actor reference
    Actor(Box<crate::carla::Actor>),
    /// Actor ID
    Id(i32),
}

impl From<i32> for ActorOrId {
    fn from(id: i32) -> Self {
        ActorOrId::Id(id)
    }
}

impl From<crate::carla::Actor> for ActorOrId {
    fn from(actor: crate::carla::Actor) -> Self {
        ActorOrId::Actor(Box::new(actor))
    }
}
"#,
    );

    // Common type aliases
    content.push_str(
        r#"
/// Type alias for actor ID
pub type ActorId = i32;

/// Type alias for timestamp (seconds)
pub type Timestamp = f64;

/// Type alias for VehicleAckermannControl (alternative name)
pub type AckermannVehicleControl = crate::carla::VehicleAckermannControl;

/// Type alias for GBufferTextureId (alternative capitalization)
pub type GBufferTextureID = crate::carla::GBufferTextureId;

/// Command trait for CARLA commands
pub trait Command: Send + Sync {
    /// Execute the command
    fn execute(&self) -> crate::error::Result<()>;
}

// Re-export for command module
pub use self::Command as CommandTrait;
"#,
    );

    content
}
