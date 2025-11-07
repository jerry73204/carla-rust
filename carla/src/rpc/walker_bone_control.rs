//! Walker bone control types for custom animations.
//!
//! These types allow control over individual bones in a walker's skeleton
//! for custom animations and poses.

use crate::geom::Transform;
use autocxx::WithinBox;
use carla_sys::carla_rust::rpc::{
    FfiBoneTransformDataOut, FfiBoneTransformDataOut_bone_name, FfiBoneTransformDataOut_component,
    FfiBoneTransformDataOut_relative_transform, FfiBoneTransformDataOut_world,
    FfiWalkerBoneControlIn, FfiWalkerBoneControlIn_add_bone, FfiWalkerBoneControlOut,
    FfiWalkerBoneControlOut_bone_at, FfiWalkerBoneControlOut_bone_count,
};

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

/// FFI conversion implementations for WalkerBoneControl types.
///
/// These implementations allow converting between Rust types and the FFI wrapper types
/// that autocxx can handle.
impl BoneTransformDataOut {
    /// Converts from FFI type returned by C++.
    ///
    /// Uses free functions to access fields from the opaque FFI type.
    pub(crate) fn from_ffi(mut ffi_bone: std::pin::Pin<&mut FfiBoneTransformDataOut>) -> Self {
        Self {
            bone_name: FfiBoneTransformDataOut_bone_name(ffi_bone.as_mut())
                .to_string_lossy()
                .into_owned(),
            world: Transform::from_ffi(FfiBoneTransformDataOut_world(ffi_bone.as_mut())),
            component: Transform::from_ffi(FfiBoneTransformDataOut_component(ffi_bone.as_mut())),
            relative: Transform::from_ffi(FfiBoneTransformDataOut_relative_transform(
                ffi_bone.as_mut(),
            )),
        }
    }
}

impl WalkerBoneControlIn {
    /// Converts to FFI type for passing to C++.
    ///
    /// Uses free function to populate the opaque FFI type.
    pub(crate) fn to_ffi(&self) -> FfiWalkerBoneControlIn {
        let mut ffi = FfiWalkerBoneControlIn::new().within_box();
        for bone in &self.bone_transforms {
            cxx::let_cxx_string!(name_cxx = &bone.bone_name);
            FfiWalkerBoneControlIn_add_bone(ffi.as_mut(), &name_cxx, bone.transform.as_ffi());
        }
        // SAFETY: We need to move out of Pin<Box<>> to return the value
        // This is safe because FfiWalkerBoneControlIn is being consumed
        unsafe { *Box::from_raw(Box::into_raw(std::pin::Pin::into_inner_unchecked(ffi))) }
    }
}

impl WalkerBoneControlOut {
    /// Converts from FFI type returned by C++.
    ///
    /// Uses free functions to access elements from the opaque FFI type.
    pub(crate) fn from_ffi(mut ffi_control: std::pin::Pin<&mut FfiWalkerBoneControlOut>) -> Self {
        let bone_count = FfiWalkerBoneControlOut_bone_count(ffi_control.as_mut());
        let mut bone_transforms = Vec::with_capacity(bone_count);

        for i in 0..bone_count {
            let mut ffi_bone = FfiWalkerBoneControlOut_bone_at(ffi_control.as_mut(), i);
            bone_transforms.push(BoneTransformDataOut::from_ffi(ffi_bone.as_mut().unwrap()));
        }

        Self { bone_transforms }
    }
}
