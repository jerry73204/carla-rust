use std::ffi::c_int;

use crate::sensor::data::Color;
use carla_sys::{carla::rpc::ActorAttributeType, carla_rust::client::FfiActorAttributeValue};
use derivative::Derivative;

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActorAttributeValue {
    #[derivative(Debug = "ignore")]
    inner: FfiActorAttributeValue,
}

impl ActorAttributeValue {
    pub fn id(&self) -> String {
        self.inner.GetId().to_string()
    }

    pub fn type_(&self) -> ActorAttributeType {
        self.inner.GetType()
    }

    pub fn value(&self) -> Option<ActorAttributeValueKind> {
        use ActorAttributeType as T;

        Some(match self.type_() {
            T::Bool => self.inner.to_bool().into(),
            T::Int => self.inner.to_int().0.into(),
            T::Float => self.inner.to_float().into(),
            T::String => self.inner.to_string().to_string().into(),
            T::RGBColor => self.inner.to_color().into(),
            _ => return None,
        })
    }

    pub fn value_string(&self) -> String {
        self.inner.GetValue().to_string()
    }
}

#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub enum ActorAttributeValueKind {
    Bool(bool),
    Int(c_int),
    F32(f32),
    String(String),
    Color(#[derivative(Debug = "ignore")] Color),
}

impl ActorAttributeValueKind {
    pub fn try_into_bool(self) -> Result<bool, Self> {
        if let Self::Bool(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_int(self) -> Result<c_int, Self> {
        if let Self::Int(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_f32(self) -> Result<f32, Self> {
        if let Self::F32(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_string(self) -> Result<String, Self> {
        if let Self::String(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_color(self) -> Result<Color, Self> {
        if let Self::Color(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    /// Returns `true` if the actor attribute value kind is [`Bool`].
    ///
    /// [`Bool`]: ActorAttributeValueKind::Bool
    #[must_use]
    pub fn is_bool(&self) -> bool {
        matches!(self, Self::Bool(..))
    }

    /// Returns `true` if the actor attribute value kind is [`Int`].
    ///
    /// [`Int`]: ActorAttributeValueKind::Int
    #[must_use]
    pub fn is_int(&self) -> bool {
        matches!(self, Self::Int(..))
    }

    /// Returns `true` if the actor attribute value kind is [`F32`].
    ///
    /// [`F32`]: ActorAttributeValueKind::F32
    #[must_use]
    pub fn is_f32(&self) -> bool {
        matches!(self, Self::F32(..))
    }

    /// Returns `true` if the actor attribute value kind is [`String`].
    ///
    /// [`String`]: ActorAttributeValueKind::String
    #[must_use]
    pub fn is_string(&self) -> bool {
        matches!(self, Self::String(..))
    }

    /// Returns `true` if the actor attribute value kind is [`Color`].
    ///
    /// [`Color`]: ActorAttributeValueKind::Color
    #[must_use]
    pub fn is_color(&self) -> bool {
        matches!(self, Self::Color(..))
    }
}

impl From<Color> for ActorAttributeValueKind {
    fn from(v: Color) -> Self {
        Self::Color(v)
    }
}

impl From<String> for ActorAttributeValueKind {
    fn from(v: String) -> Self {
        Self::String(v)
    }
}

impl From<f32> for ActorAttributeValueKind {
    fn from(v: f32) -> Self {
        Self::F32(v)
    }
}

impl From<c_int> for ActorAttributeValueKind {
    fn from(v: c_int) -> Self {
        Self::Int(v)
    }
}

impl From<bool> for ActorAttributeValueKind {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}
