//! Internal utilities.

use crate::geom::Location;
use anyhow::{anyhow, Result};
use std::{
    ffi::{CStr, CString},
    os::raw::c_char,
};

/// Error handling utilities for C FFI calls.
pub fn check_carla_error(error: carla_sys::carla_error_t) -> Result<()> {
    match error {
        carla_sys::carla_error_t_CARLA_ERROR_NONE => Ok(()),
        carla_sys::carla_error_t_CARLA_ERROR_CONNECTION_FAILED => Err(anyhow!("Connection failed")),
        carla_sys::carla_error_t_CARLA_ERROR_TIMEOUT => Err(anyhow!("Operation timed out")),
        carla_sys::carla_error_t_CARLA_ERROR_INVALID_ARGUMENT => Err(anyhow!("Invalid argument")),
        carla_sys::carla_error_t_CARLA_ERROR_NOT_FOUND => Err(anyhow!("Not found")),
        carla_sys::carla_error_t_CARLA_ERROR_NOT_IMPLEMENTED => Err(anyhow!("Not implemented")),
        carla_sys::carla_error_t_CARLA_ERROR_UNKNOWN => Err(anyhow!("Unknown error")),
        _ => Err(anyhow!("Unknown CARLA error: {}", error)),
    }
}

/// Safe conversion from Rust string to C string.
pub fn rust_string_to_c(s: &str) -> Result<CString> {
    CString::new(s).map_err(|e| anyhow!("String contains null byte: {}", e))
}

/// Safe conversion from C string to Rust string.
///
/// # Safety
/// The pointer must be valid and point to a null-terminated C string.
pub unsafe fn c_string_to_rust(ptr: *const c_char) -> Result<String> {
    if ptr.is_null() {
        return Err(anyhow!("Null pointer passed to c_string_to_rust"));
    }
    Ok(CStr::from_ptr(ptr).to_string_lossy().into_owned())
}

/// Helper trait for converting collections to C arrays and back.
///
/// Note: These implementations use standard Rust allocation and are
/// intended for temporary conversions. For production use, implement
/// proper CARLA allocator integration.
pub trait ArrayConversionExt<T> {
    /// Create from a C array.
    ///
    /// # Safety
    /// The pointer must be valid and point to `len` valid elements of type T.
    unsafe fn from_c_array(ptr: *const T, len: usize) -> Self;
}

impl ArrayConversionExt<u8> for Vec<u8> {
    unsafe fn from_c_array(ptr: *const u8, len: usize) -> Self {
        if ptr.is_null() || len == 0 {
            return Vec::new();
        }
        let mut vec = Vec::with_capacity(len);
        std::ptr::copy_nonoverlapping(ptr, vec.as_mut_ptr(), len);
        vec.set_len(len);
        vec
    }
}

impl ArrayConversionExt<Location> for Vec<Location> {
    unsafe fn from_c_array(ptr: *const Location, len: usize) -> Self {
        if ptr.is_null() || len == 0 {
            return Vec::new();
        }
        let mut vec = Vec::with_capacity(len);
        std::ptr::copy_nonoverlapping(ptr, vec.as_mut_ptr(), len);
        vec.set_len(len);
        vec
    }
}

// Note: Vector2D is an alias for Location, so we don't need a separate implementation
