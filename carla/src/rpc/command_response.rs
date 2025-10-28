//! Response types for batch command execution.
//!
//! When commands are executed via [`Client::apply_batch_sync`](crate::client::Client::apply_batch_sync),
//! each command returns a [`CommandResponse`] indicating success or failure.

use crate::rpc::ActorId;
use carla_sys::carla_rust::client::FfiCommandResponse;

/// Response from executing a batch command.
///
/// Indicates whether the command succeeded and provides the actor ID for
/// spawn commands.
#[derive(Debug, Clone)]
pub struct CommandResponse {
    /// Error message if command failed, None if successful
    error: Option<String>,
    /// Actor ID returned for spawn commands, or the affected actor ID
    actor_id: ActorId,
}

impl CommandResponse {
    /// Creates a successful response.
    pub(crate) fn success(actor_id: ActorId) -> Self {
        Self {
            error: None,
            actor_id,
        }
    }

    /// Creates an error response.
    pub(crate) fn from_error(message: String, actor_id: ActorId) -> Self {
        Self {
            error: Some(message),
            actor_id,
        }
    }

    /// Converts from FFI CommandResponse.
    pub(crate) fn from_ffi(ffi_resp: &FfiCommandResponse) -> Self {
        use carla_sys::carla_rust::client::FfiCommandResponse as FfiResp;

        if FfiResp::HasError(ffi_resp) {
            Self::from_error(
                FfiResp::GetErrorMessage(ffi_resp).to_string(),
                FfiResp::GetActorId(ffi_resp),
            )
        } else {
            Self::success(FfiResp::GetActorId(ffi_resp))
        }
    }

    /// Returns true if the command succeeded.
    pub fn is_success(&self) -> bool {
        self.error.is_none()
    }

    /// Returns true if the command failed.
    pub fn has_error(&self) -> bool {
        self.error.is_some()
    }

    /// Returns the error message if the command failed.
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    /// Returns the actor ID.
    ///
    /// For spawn commands, this is the newly created actor's ID.
    /// For other commands, this may be 0 or the target actor ID.
    pub fn actor_id(&self) -> Option<ActorId> {
        if self.actor_id == 0 {
            None
        } else {
            Some(self.actor_id)
        }
    }
}
