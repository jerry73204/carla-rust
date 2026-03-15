/// RGB/RGBA color type for rendering and visualization.
///
/// Used by debug drawing and other visual features in CARLA.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Color {
    /// Red component (0-255)
    pub r: u8,
    /// Green component (0-255)
    pub g: u8,
    /// Blue component (0-255)
    pub b: u8,
    /// Alpha component (0-255), 255 = fully opaque
    pub a: u8,
}

impl Color {
    /// Create a new color with RGB values and full opacity.
    ///
    /// # Example
    /// ```
    /// # use carla::rpc::Color;
    /// let red = Color::new(255, 0, 0);
    /// assert_eq!(red.a, 255);
    /// ```
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b, a: 255 }
    }

    /// Create a new color with RGBA values.
    ///
    /// # Example
    /// ```
    /// # use carla::rpc::Color;
    /// let semi_transparent_red = Color::rgba(255, 0, 0, 128);
    /// assert_eq!(semi_transparent_red.a, 128);
    /// ```
    pub const fn rgba(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self { r, g, b, a }
    }

    // Predefined color constants

    /// Red color (255, 0, 0)
    pub const RED: Self = Self::new(255, 0, 0);

    /// Green color (0, 255, 0)
    pub const GREEN: Self = Self::new(0, 255, 0);

    /// Blue color (0, 0, 255)
    pub const BLUE: Self = Self::new(0, 0, 255);

    /// White color (255, 255, 255)
    pub const WHITE: Self = Self::new(255, 255, 255);

    /// Black color (0, 0, 0)
    pub const BLACK: Self = Self::new(0, 0, 0);

    /// Yellow color (255, 255, 0)
    pub const YELLOW: Self = Self::new(255, 255, 0);

    /// Cyan color (0, 255, 255)
    pub const CYAN: Self = Self::new(0, 255, 255);

    /// Magenta color (255, 0, 255)
    pub const MAGENTA: Self = Self::new(255, 0, 255);

    /// Orange color (255, 165, 0)
    pub const ORANGE: Self = Self::new(255, 165, 0);

    /// Purple color (128, 0, 128)
    pub const PURPLE: Self = Self::new(128, 0, 128);

    /// Gray color (128, 128, 128)
    pub const GRAY: Self = Self::new(128, 128, 128);
}

impl From<(u8, u8, u8)> for Color {
    /// Create a color from an RGB tuple with full opacity.
    ///
    /// # Example
    /// ```
    /// # use carla::rpc::Color;
    /// let color: Color = (255, 128, 64).into();
    /// assert_eq!(color.r, 255);
    /// assert_eq!(color.g, 128);
    /// assert_eq!(color.b, 64);
    /// assert_eq!(color.a, 255);
    /// ```
    fn from((r, g, b): (u8, u8, u8)) -> Self {
        Self::new(r, g, b)
    }
}

impl From<(u8, u8, u8, u8)> for Color {
    /// Create a color from an RGBA tuple.
    ///
    /// # Example
    /// ```
    /// # use carla::rpc::Color;
    /// let color: Color = (255, 128, 64, 200).into();
    /// assert_eq!(color.r, 255);
    /// assert_eq!(color.g, 128);
    /// assert_eq!(color.b, 64);
    /// assert_eq!(color.a, 200);
    /// ```
    fn from((r, g, b, a): (u8, u8, u8, u8)) -> Self {
        Self::rgba(r, g, b, a)
    }
}

impl From<Color> for carla_sys::carla_rust::sensor::data::FfiColor {
    fn from(color: Color) -> Self {
        Self {
            r: color.r,
            g: color.g,
            b: color.b,
            a: color.a,
        }
    }
}

impl From<carla_sys::carla_rust::sensor::data::FfiColor> for Color {
    fn from(ffi: carla_sys::carla_rust::sensor::data::FfiColor) -> Self {
        Self {
            r: ffi.r,
            g: ffi.g,
            b: ffi.b,
            a: ffi.a,
        }
    }
}

/// Floating-point RGBA color type with components in the range 0.0–1.0.
///
/// Used for texture operations and material parameters in CARLA 0.10.0+.
///
/// Corresponds to [`carla.FloatColor`] in the Python API.
///
/// [`carla.FloatColor`]: https://carla.readthedocs.io/en/latest/python_api/#carla.FloatColor
#[cfg(carla_0100)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FloatColor {
    /// Red component (0.0–1.0)
    pub r: f32,
    /// Green component (0.0–1.0)
    pub g: f32,
    /// Blue component (0.0–1.0)
    pub b: f32,
    /// Alpha component (0.0–1.0), 1.0 = fully opaque
    pub a: f32,
}

#[cfg(carla_0100)]
impl FloatColor {
    /// Create a new float color with RGB values and full opacity.
    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Create a new float color with RGBA values.
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }
}

#[cfg(carla_0100)]
impl From<Color> for FloatColor {
    fn from(c: Color) -> Self {
        Self {
            r: c.r as f32 / 255.0,
            g: c.g as f32 / 255.0,
            b: c.b as f32 / 255.0,
            a: c.a as f32 / 255.0,
        }
    }
}

/// Material parameter type for texture operations.
///
/// Specifies which texture slot to apply when using texture application methods.
///
/// Corresponds to [`carla.MaterialParameter`] in the Python API.
///
/// [`carla.MaterialParameter`]: https://carla.readthedocs.io/en/latest/python_api/#carla.MaterialParameter
#[cfg(carla_0100)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MaterialParameter {
    /// Normal map texture
    TexNormal,
    /// Combined AO, Roughness, Metallic, and Emissive texture
    TexAoRoughnessMetallicEmissive,
    /// Diffuse (albedo) texture
    TexDiffuse,
    /// Emissive texture
    TexEmissive,
}

#[cfg(carla_0100)]
impl MaterialParameter {
    pub(crate) fn as_u8(self) -> u8 {
        match self {
            Self::TexNormal => 0,
            Self::TexAoRoughnessMetallicEmissive => 1,
            Self::TexDiffuse => 2,
            Self::TexEmissive => 3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_color_creation() {
        let color = Color::new(100, 150, 200);
        assert_eq!(color.r, 100);
        assert_eq!(color.g, 150);
        assert_eq!(color.b, 200);
        assert_eq!(color.a, 255);
    }

    #[test]
    fn test_color_rgba() {
        let color = Color::rgba(10, 20, 30, 40);
        assert_eq!(color.r, 10);
        assert_eq!(color.g, 20);
        assert_eq!(color.b, 30);
        assert_eq!(color.a, 40);
    }

    #[test]
    fn test_color_constants() {
        assert_eq!(Color::RED, Color::new(255, 0, 0));
        assert_eq!(Color::GREEN, Color::new(0, 255, 0));
        assert_eq!(Color::BLUE, Color::new(0, 0, 255));
        assert_eq!(Color::WHITE, Color::new(255, 255, 255));
        assert_eq!(Color::BLACK, Color::new(0, 0, 0));
        assert_eq!(Color::YELLOW, Color::new(255, 255, 0));
    }

    #[test]
    fn test_from_rgb_tuple() {
        let color: Color = (100, 150, 200).into();
        assert_eq!(color.r, 100);
        assert_eq!(color.g, 150);
        assert_eq!(color.b, 200);
        assert_eq!(color.a, 255);
    }

    #[test]
    fn test_from_rgba_tuple() {
        let color: Color = (10, 20, 30, 40).into();
        assert_eq!(color.r, 10);
        assert_eq!(color.g, 20);
        assert_eq!(color.b, 30);
        assert_eq!(color.a, 40);
    }
}
