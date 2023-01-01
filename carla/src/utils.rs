use crate::geom::Location;
use carla_sys::{
    carla::{
        geom::Vector2D,
        rpc::{GearPhysicsControl, WheelPhysicsControl},
    },
    carla_rust::utils::{
        new_ffi_location_vector, new_gear_physics_control_vector, new_uint32_t_vector,
        new_uint64_t_vector, new_uint8_t_vector, new_vector_2d_vector,
        new_wheel_physics_control_vector,
    },
};
use cxx::{kind::Trivial, vector::VectorElement, CxxVector, ExternType, UniquePtr};

pub trait NewCxxVectorElement
where
    Self: VectorElement + ExternType<Kind = Trivial>,
{
    fn new_vector() -> UniquePtr<CxxVector<Self>>;
}

impl NewCxxVectorElement for u8 {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_uint8_t_vector()
    }
}

impl NewCxxVectorElement for u32 {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_uint32_t_vector()
    }
}

impl NewCxxVectorElement for u64 {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_uint64_t_vector()
    }
}

impl NewCxxVectorElement for Vector2D {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_vector_2d_vector()
    }
}

impl NewCxxVectorElement for GearPhysicsControl {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_gear_physics_control_vector()
    }
}

impl NewCxxVectorElement for WheelPhysicsControl {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_wheel_physics_control_vector()
    }
}

impl NewCxxVectorElement for Location {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_ffi_location_vector()
    }
}

// impl NewCxxVectorElement for SharedPtr<FfiActor> {
//     fn new_vector() -> UniquePtr<CxxVector<Self>> {
//         new_ffi_location_vector()
//     }
// }

pub trait CxxVectorExt<T>
where
    T: NewCxxVectorElement,
{
    fn new_typed() -> UniquePtr<CxxVector<T>> {
        T::new_vector()
    }
}

impl<T> CxxVectorExt<T> for CxxVector<T> where T: NewCxxVectorElement {}

pub trait IteratorExt
where
    Self: Iterator,
{
    fn collect_cxx_vector(self) -> UniquePtr<CxxVector<Self::Item>>
    where
        Self::Item: NewCxxVectorElement;
}

impl<I> IteratorExt for I
where
    I: Iterator,
{
    fn collect_cxx_vector(self) -> UniquePtr<CxxVector<Self::Item>>
    where
        Self::Item: NewCxxVectorElement,
    {
        let mut vec = Self::Item::new_vector();
        self.for_each(|item| vec.pin_mut().push(item));
        vec
    }
}
