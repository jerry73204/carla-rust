//! UI modules
//!
//! User interface components:
//! - FadingText: Temporary notification display with fade animation
//! - HelpText: Full help overlay with all keyboard controls

pub mod fading_text;
pub mod help_text;

pub use fading_text::FadingText;
pub use help_text::HelpText;
