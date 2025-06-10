//! Low-level FFI bindings for the CARLA simulator C library.
//!
//! This crate provides Rust bindings to libcarla_c, a C wrapper around
//! the CARLA simulator's C++ client library. These bindings are generated
//! automatically using bindgen.
//!
//! # Safety
//!
//! All functions in this crate are marked as `unsafe` because they directly
//! interface with C code. Callers must ensure:
//! - Pointers are valid and properly aligned
//! - Memory is not accessed after being freed
//! - C strings are null-terminated
//! - Objects are not used after destruction

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]

// Include the generated bindings
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_enum_values() {
        // Test that error enum values are correctly bound
        assert_eq!(carla_error_t_CARLA_ERROR_NONE, 0);
        assert_eq!(carla_error_t_CARLA_ERROR_CONNECTION_FAILED, 1);
        assert_eq!(carla_error_t_CARLA_ERROR_TIMEOUT, 2);
    }

    #[test]
    fn test_map_layer_enum_values() {
        // Test that map layer enum values are correctly bound
        assert_eq!(carla_map_layer_t_CARLA_MAP_LAYER_NONE, 0);
        assert_eq!(carla_map_layer_t_CARLA_MAP_LAYER_BUILDINGS, 1);
        assert_eq!(carla_map_layer_t_CARLA_MAP_LAYER_ALL, 0xFFFF);
    }
}
