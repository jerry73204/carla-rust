use std::{marker::PhantomData, pin::Pin};

use autocxx::WithinUniquePtr;
use cxx::UniquePtr;

use super::{Color, FloatColor};

/// Trait for pixel types that can be stored in a [`Texture`].
///
/// # Safety
///
/// Implementors must correctly dispatch all FFI methods to the corresponding
/// C++ wrapper type.
pub unsafe trait TexturePixel: Sized {
    #[doc(hidden)]
    type Ffi: cxx::memory::UniquePtrTarget;

    #[doc(hidden)]
    fn ffi_new(width: u32, height: u32) -> UniquePtr<Self::Ffi>;
    #[doc(hidden)]
    fn ffi_width(ffi: &Self::Ffi) -> u32;
    #[doc(hidden)]
    fn ffi_height(ffi: &Self::Ffi) -> u32;
    #[doc(hidden)]
    fn ffi_set_pixel(ffi: Pin<&mut Self::Ffi>, x: u32, y: u32, pixel: Self);
    #[doc(hidden)]
    fn ffi_get_pixel(ffi: &Self::Ffi, x: u32, y: u32) -> Self;
}

/// A 2D texture backed by a C++ `carla::rpc::Texture<T>`.
///
/// Data is stored in C++ memory, avoiding copies when passed to CARLA API methods.
///
/// # Type Aliases
///
/// - [`TextureColor`] = `Texture<Color>` — 8-bit RGBA texture
/// - [`TextureFloatColor`] = `Texture<FloatColor>` — floating-point RGBA texture
///
/// # Examples
///
/// ```ignore
/// use carla::rpc::{Color, TextureColor};
///
/// let mut tex = TextureColor::new(256, 256);
/// tex.set_pixel(0, 0, Color::RED);
/// assert_eq!(tex.pixel(0, 0), Color::RED);
/// ```
pub struct Texture<T: TexturePixel> {
    pub(crate) inner: UniquePtr<T::Ffi>,
    _phantom: PhantomData<T>,
}

impl<T: TexturePixel> Texture<T> {
    /// Create a new texture with the given dimensions.
    ///
    /// All pixels are initialized to zero.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            inner: T::ffi_new(width, height),
            _phantom: PhantomData,
        }
    }

    /// Returns the width of the texture in pixels.
    pub fn width(&self) -> u32 {
        T::ffi_width(&self.inner)
    }

    /// Returns the height of the texture in pixels.
    pub fn height(&self) -> u32 {
        T::ffi_height(&self.inner)
    }

    /// Set the pixel at position `(x, y)`.
    pub fn set_pixel(&mut self, x: u32, y: u32, pixel: T) {
        T::ffi_set_pixel(self.inner.pin_mut(), x, y, pixel);
    }

    /// Get the pixel at position `(x, y)`.
    pub fn pixel(&self, x: u32, y: u32) -> T {
        T::ffi_get_pixel(&self.inner, x, y)
    }
}

// --- Color (sensor::data::Color, BGRA in C++) ---

unsafe impl TexturePixel for Color {
    type Ffi = carla_sys::carla_rust::rpc::FfiTextureColor;

    fn ffi_new(width: u32, height: u32) -> UniquePtr<Self::Ffi> {
        Self::Ffi::new(width, height).within_unique_ptr()
    }

    fn ffi_width(ffi: &Self::Ffi) -> u32 {
        ffi.GetWidth()
    }

    fn ffi_height(ffi: &Self::Ffi) -> u32 {
        ffi.GetHeight()
    }

    fn ffi_set_pixel(ffi: Pin<&mut Self::Ffi>, x: u32, y: u32, pixel: Self) {
        ffi.SetPixel(x, y, pixel.r, pixel.g, pixel.b, pixel.a);
    }

    fn ffi_get_pixel(ffi: &Self::Ffi, x: u32, y: u32) -> Self {
        Color::rgba(
            ffi.GetPixelR(x, y),
            ffi.GetPixelG(x, y),
            ffi.GetPixelB(x, y),
            ffi.GetPixelA(x, y),
        )
    }
}

// --- FloatColor (rpc::FloatColor, RGBA in C++) ---

unsafe impl TexturePixel for FloatColor {
    type Ffi = carla_sys::carla_rust::rpc::FfiTextureFloatColor;

    fn ffi_new(width: u32, height: u32) -> UniquePtr<Self::Ffi> {
        Self::Ffi::new(width, height).within_unique_ptr()
    }

    fn ffi_width(ffi: &Self::Ffi) -> u32 {
        ffi.GetWidth()
    }

    fn ffi_height(ffi: &Self::Ffi) -> u32 {
        ffi.GetHeight()
    }

    fn ffi_set_pixel(ffi: Pin<&mut Self::Ffi>, x: u32, y: u32, pixel: Self) {
        ffi.SetPixel(x, y, pixel.r, pixel.g, pixel.b, pixel.a);
    }

    fn ffi_get_pixel(ffi: &Self::Ffi, x: u32, y: u32) -> Self {
        FloatColor::rgba(
            ffi.GetPixelR(x, y),
            ffi.GetPixelG(x, y),
            ffi.GetPixelB(x, y),
            ffi.GetPixelA(x, y),
        )
    }
}

/// 8-bit RGBA texture backed by C++ `carla::rpc::TextureColor`.
pub type TextureColor = Texture<Color>;

/// Floating-point RGBA texture backed by C++ `carla::rpc::TextureFloatColor`.
pub type TextureFloatColor = Texture<FloatColor>;
