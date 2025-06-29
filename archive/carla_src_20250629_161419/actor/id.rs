/// Unique identifier for actors in the simulation.
pub type ActorId = u32;

/// Invalid actor ID constant.
const INVALID_ACTOR_ID: ActorId = 0;

/// Utility functions for actor IDs.
impl ActorIdExt for ActorId {
    fn is_valid(&self) -> bool {
        *self != INVALID_ACTOR_ID
    }
}

/// Extension trait for ActorId.
pub trait ActorIdExt {
    /// Check if the actor ID is valid.
    fn is_valid(&self) -> bool;
}
