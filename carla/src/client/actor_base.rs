use carla_sys::*;

/// This trait defines a basic actor in the simulation. It is
/// implemented on all actor type variants.
pub trait ActorBase: Clone {
    /// Get the raw C pointer to the actor.
    fn raw_ptr(&self) -> *mut carla_actor_t;

    // TODO: Uncomment and implement when full actor system is migrated
    // fn into_actor(self) -> Result<Actor> {
    //     Actor::from_raw_ptr(self.raw_ptr())
    // }

    // fn id(&self) -> ActorId {
    //     unsafe { carla_actor_get_id(self.raw_ptr()) }
    // }
}
