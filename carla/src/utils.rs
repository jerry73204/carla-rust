//! Internal utilities.

use core::slice;

#[cfg(not(carla_version_0100))]
use carla_sys::{
    carla::rpc::WheelPhysicsControl, carla_rust::utils::new_wheel_physics_control_vector,
};
use carla_sys::{
    carla::{geom::Vector2D, rpc::GearPhysicsControl},
    carla_rust::{
        geom::FfiLocation,
        utils::{
            new_ffi_location_vector, new_float_vector, new_gear_physics_control_vector,
            new_uint8_t_vector, new_uint32_t_vector, new_uint64_t_vector, new_vector_2d_vector,
        },
    },
};
use cxx::{CxxVector, ExternType, UniquePtr, kind::Trivial, vector::VectorElement};

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

#[cfg(not(carla_version_0100))]
impl NewCxxVectorElement for WheelPhysicsControl {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_wheel_physics_control_vector()
    }
}

impl NewCxxVectorElement for f32 {
    fn new_vector() -> UniquePtr<CxxVector<Self>> {
        new_float_vector()
    }
}

impl NewCxxVectorElement for FfiLocation {
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

/// Build a slice from an FFI container's `data()` pointer and length.
///
/// An empty C++ container is entitled to return a null `data()` pointer, and
/// `slice::from_raw_parts` rejects null even for a zero length -- it is undefined behaviour,
/// and Rust's precondition check aborts the process rather than unwinding, so a caller
/// cannot defend against it. Every container here can legitimately be empty: an actor with
/// no attributes, a lidar frame with no returns, a map with no spawn points. Return the
/// empty slice for that case instead of forming a slice from the pointer at all.
///
/// # Safety
///
/// For a non-zero `len`, `ptr` must be valid, aligned, and point to `len` initialised
/// elements that outlive `'a` -- the usual `slice::from_raw_parts` contract.
pub(crate) unsafe fn ffi_slice<'a, T>(ptr: *const T, len: usize) -> &'a [T] {
    if len == 0 {
        return &[];
    }
    debug_assert!(
        !ptr.is_null(),
        "non-empty FFI container has a null data pointer"
    );
    debug_assert!(
        (ptr as usize).is_multiple_of(std::mem::align_of::<T>()),
        "FFI container data pointer is not aligned for its element type"
    );
    unsafe { slice::from_raw_parts(ptr, len) }
}

#[cfg(test)]
mod tests {
    use super::ffi_slice;

    /// The case that aborted a bridge process: an empty container whose `data()` is null.
    #[test]
    fn empty_null_pointer_yields_empty_slice() {
        let slice: &[u32] = unsafe { ffi_slice(std::ptr::null(), 0) };
        assert!(slice.is_empty());
    }

    #[test]
    fn non_empty_slice_is_passed_through() {
        let data = [1u32, 2, 3];
        let slice = unsafe { ffi_slice(data.as_ptr(), data.len()) };
        assert_eq!(slice, &data);
    }
}
