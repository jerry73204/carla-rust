//! Weather parameter utilities for CARLA simulator FFI.

use crate::ffi::bridge::SimpleWeatherParameters;

impl Default for SimpleWeatherParameters {
    fn default() -> Self {
        Self {
            cloudiness: 0.0,
            precipitation: 0.0,
            precipitation_deposits: 0.0,
            wind_intensity: 10.0,
            sun_azimuth_angle: -1.0, // -1.0 means use system default
            sun_altitude_angle: 45.0,
            fog_density: 2.0,
            fog_distance: 0.75,
            fog_falloff: 0.1,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 0.0,
        }
    }
}

/// Weather parameter convenience constructors and presets.
impl SimpleWeatherParameters {
    /// Clear sunny noon conditions
    pub fn clear_noon() -> Self {
        Self {
            cloudiness: 5.0,
            precipitation: 0.0,
            precipitation_deposits: 0.0,
            wind_intensity: 10.0,
            sun_azimuth_angle: -1.0,
            sun_altitude_angle: 45.0,
            fog_density: 2.0,
            fog_distance: 0.75,
            fog_falloff: 0.1,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 0.0,
        }
    }

    /// Cloudy noon conditions
    pub fn cloudy_noon() -> Self {
        Self {
            cloudiness: 80.0,
            precipitation: 0.0,
            precipitation_deposits: 0.0,
            wind_intensity: 10.0,
            sun_azimuth_angle: -1.0,
            sun_altitude_angle: 45.0,
            fog_density: 2.0,
            fog_distance: 0.75,
            fog_falloff: 0.1,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 0.0,
        }
    }

    /// Hard rain noon conditions
    pub fn hard_rain_noon() -> Self {
        Self {
            cloudiness: 100.0,
            precipitation: 100.0,
            precipitation_deposits: 90.0,
            wind_intensity: 100.0,
            sun_azimuth_angle: -1.0,
            sun_altitude_angle: 45.0,
            fog_density: 7.0,
            fog_distance: 0.75,
            fog_falloff: 0.1,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 0.0,
        }
    }

    /// Clear night conditions
    pub fn clear_night() -> Self {
        Self {
            cloudiness: 5.0,
            precipitation: 0.0,
            precipitation_deposits: 0.0,
            wind_intensity: 10.0,
            sun_azimuth_angle: -1.0,
            sun_altitude_angle: -90.0, // Night
            fog_density: 60.0,
            fog_distance: 75.0,
            fog_falloff: 1.0,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 0.0,
        }
    }

    /// Dust storm conditions
    pub fn dust_storm() -> Self {
        Self {
            cloudiness: 100.0,
            precipitation: 0.0,
            precipitation_deposits: 0.0,
            wind_intensity: 100.0,
            sun_azimuth_angle: -1.0,
            sun_altitude_angle: 45.0,
            fog_density: 2.0,
            fog_distance: 0.75,
            fog_falloff: 0.1,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 100.0,
        }
    }
}
