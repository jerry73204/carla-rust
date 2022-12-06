use super::{Actor, Sensor, Vehicle};

#[derive(Debug, Clone)]
pub enum ActorKind {
    Vehicle(Vehicle),
    Sensor(Sensor),
    Other(Actor),
}

impl ActorKind {
    /// Returns `true` if the actor kind is [`Vehicle`].
    ///
    /// [`Vehicle`]: ActorKind::Vehicle
    #[must_use]
    pub fn is_vehicle(&self) -> bool {
        matches!(self, Self::Vehicle(..))
    }

    /// Returns `true` if the actor kind is [`Sensor`].
    ///
    /// [`Sensor`]: ActorKind::Sensor
    #[must_use]
    pub fn is_sensor(&self) -> bool {
        matches!(self, Self::Sensor(..))
    }

    /// Returns `true` if the actor kind is [`Other`].
    ///
    /// [`Other`]: ActorKind::Other
    #[must_use]
    pub fn is_other(&self) -> bool {
        matches!(self, Self::Other(..))
    }

    pub fn try_into_vehicle(self) -> Result<Vehicle, Self> {
        if let Self::Vehicle(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_sensor(self) -> Result<Sensor, Self> {
        if let Self::Sensor(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_other(self) -> Result<Actor, Self> {
        if let Self::Other(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }
}

impl From<Actor> for ActorKind {
    fn from(v: Actor) -> Self {
        Self::Other(v)
    }
}

impl From<Sensor> for ActorKind {
    fn from(v: Sensor) -> Self {
        Self::Sensor(v)
    }
}

impl From<Vehicle> for ActorKind {
    fn from(v: Vehicle) -> Self {
        Self::Vehicle(v)
    }
}
