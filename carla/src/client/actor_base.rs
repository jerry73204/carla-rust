use crate::geom::Transform;
use carla_sys::*;

/// Common interface for all actor types.
/// This trait provides polymorphic operations on actors regardless of their specific type.
pub trait ActorBase {
    /// Get the raw C pointer to the actor.
    fn raw_ptr(&self) -> *mut carla_actor_t;

    /// Get the actor's unique ID.
    fn id(&self) -> u32;

    /// Get the actor's type ID (e.g., "vehicle.tesla.model3").
    fn type_id(&self) -> String;

    /// Get the actor's current transform (location and rotation).
    fn get_transform(&self) -> Transform;

    /// Check if the actor is still alive in the simulation.
    fn is_alive(&self) -> bool;
}
