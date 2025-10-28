//! Walker bone control types for custom animations.
//!
//! These types allow control over individual bones in a walker's skeleton
//! for custom animations and poses.

use crate::geom::Transform;

/// Input data for a single bone transformation.
///
/// Corresponds to [`carla.BoneTransformDataIn`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.BoneTransformDataIn) in the Python API
///
/// # Examples
///
/// ```no_run
/// use carla::{geom::Transform, rpc::BoneTransformDataIn};
///
/// let bone_transform = BoneTransformDataIn {
///     bone_name: "crl_leg__L".to_string(),
///     transform: Transform::default(),
/// };
/// ```
#[derive(Debug, Clone)]
pub struct BoneTransformDataIn {
    /// Name of the bone to transform
    pub bone_name: String,
    /// Transform to apply to the bone
    pub transform: Transform,
}

/// Output data for a single bone transformation.
///
/// Contains the bone name and its transforms in different coordinate spaces.
///
/// Corresponds to [`carla.BoneTransformDataOut`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.BoneTransformDataOut) in the Python API
#[derive(Debug, Clone)]
pub struct BoneTransformDataOut {
    /// Name of the bone
    pub bone_name: String,
    /// World-space transform
    pub world: Transform,
    /// Component-space transform
    pub component: Transform,
    /// Relative transform
    pub relative: Transform,
}

// TODO: Re-enable once autocxx opaque type field access is solved
// impl From<carla_sys::carla::rpc::BoneTransformDataOut> for BoneTransformDataOut {
//     fn from(ffi: carla_sys::carla::rpc::BoneTransformDataOut) -> Self {
//         Self {
//             bone_name: ffi.bone_name.to_string_lossy().into_owned(),
//             world: Transform::from(ffi.world),
//             component: Transform::from(ffi.component),
//             relative: Transform::from(ffi.relative),
//         }
//     }
// }

/// Input collection of bone transforms for a walker.
///
/// Used to set custom bone poses on a walker actor.
///
/// Corresponds to [`carla.WalkerBoneControlIn`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WalkerBoneControlIn) in the Python API
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     geom::Transform,
///     rpc::{BoneTransformDataIn, WalkerBoneControlIn},
/// };
///
/// let mut bone_control = WalkerBoneControlIn {
///     bone_transforms: vec![
///         BoneTransformDataIn {
///             bone_name: "crl_arm__L".to_string(),
///             transform: Transform::default(),
///         },
///         BoneTransformDataIn {
///             bone_name: "crl_arm__R".to_string(),
///             transform: Transform::default(),
///         },
///     ],
/// };
/// ```
#[derive(Debug, Clone, Default)]
pub struct WalkerBoneControlIn {
    /// Vector of bone transformations to apply
    pub bone_transforms: Vec<BoneTransformDataIn>,
}

/// Output collection of bone transforms from a walker.
///
/// Contains the current transforms of all bones in a walker's skeleton.
///
/// Corresponds to [`carla.WalkerBoneControlOut`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WalkerBoneControlOut) in the Python API
#[derive(Debug, Clone, Default)]
pub struct WalkerBoneControlOut {
    /// Vector of bone transformations
    pub bone_transforms: Vec<BoneTransformDataOut>,
}

// TODO: Re-enable once autocxx opaque type field access is solved
// impl From<carla_sys::carla::rpc::WalkerBoneControlOut> for WalkerBoneControlOut {
//     fn from(ffi: carla_sys::carla::rpc::WalkerBoneControlOut) -> Self {
//         // Convert from FFI vector to Rust Vec
//         let bone_transforms = ffi
//             .bone_transforms
//             .iter()
//             .map(|ffi_bone| BoneTransformDataOut::from(*ffi_bone))
//             .collect();
//
//         Self { bone_transforms }
//     }
// }

// NOTE: Advanced bone control (set_bones/get_bones_transform) requires additional
// FFI integration work. The C++ wrapper infrastructure is complete (see carla-sys/csrc/carla_rust/rpc/walker_bone_control.hpp),
// but exposing these methods through the Rust API requires resolving autocxx type handling limitations.
// The core pose blending functionality (blend_pose, show_pose, hide_pose) is fully operational.
