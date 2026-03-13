/// Actor state indicating the current lifecycle status of an actor in the simulation.
///
/// # Variants
///
/// - `Invalid` — Actor is not valid
/// - `Active` — Actor is active and participating in the simulation
/// - `Dormant` — Actor is dormant (hybrid mode — outside active distance)
/// - `PendingKill` — Actor is pending destruction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ActorState {
    /// Actor is not valid.
    Invalid = 0,
    /// Actor is active and participating in the simulation.
    Active = 1,
    /// Actor is dormant (hybrid mode — outside the active distance).
    Dormant = 2,
    /// Actor is pending destruction.
    PendingKill = 3,
}

impl ActorState {
    /// Creates an `ActorState` from its raw `u8` representation.
    ///
    /// Returns `None` if the value does not correspond to a valid state.
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Invalid),
            1 => Some(Self::Active),
            2 => Some(Self::Dormant),
            3 => Some(Self::PendingKill),
            _ => None,
        }
    }
}

impl std::fmt::Display for ActorState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Invalid => write!(f, "Invalid"),
            Self::Active => write!(f, "Active"),
            Self::Dormant => write!(f, "Dormant"),
            Self::PendingKill => write!(f, "PendingKill"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_actor_state_from_u8() {
        assert_eq!(ActorState::from_u8(0), Some(ActorState::Invalid));
        assert_eq!(ActorState::from_u8(1), Some(ActorState::Active));
        assert_eq!(ActorState::from_u8(2), Some(ActorState::Dormant));
        assert_eq!(ActorState::from_u8(3), Some(ActorState::PendingKill));
        assert_eq!(ActorState::from_u8(4), None);
    }

    #[test]
    fn test_actor_state_display() {
        assert_eq!(ActorState::Active.to_string(), "Active");
        assert_eq!(ActorState::Dormant.to_string(), "Dormant");
    }
}
