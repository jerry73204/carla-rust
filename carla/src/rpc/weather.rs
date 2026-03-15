use carla_sys::carla::rpc::WeatherParameters;

// Field order: cloudiness, precipitation, precipitation_deposits, wind_intensity,
//              sun_azimuth_angle, sun_altitude_angle, fog_density, fog_distance,
//              fog_falloff, wetness, scattering_intensity, mie_scattering_scale,
//              rayleigh_scattering_scale, dust_storm

const fn wp(
    cloudiness: f32,
    precipitation: f32,
    precipitation_deposits: f32,
    wind_intensity: f32,
    sun_azimuth_angle: f32,
    sun_altitude_angle: f32,
    fog_density: f32,
    fog_distance: f32,
    fog_falloff: f32,
    wetness: f32,
    scattering_intensity: f32,
    mie_scattering_scale: f32,
    rayleigh_scattering_scale: f32,
    dust_storm: f32,
) -> WeatherParameters {
    WeatherParameters {
        cloudiness,
        precipitation,
        precipitation_deposits,
        wind_intensity,
        sun_azimuth_angle,
        sun_altitude_angle,
        fog_density,
        fog_distance,
        fog_falloff,
        wetness,
        scattering_intensity,
        mie_scattering_scale,
        rayleigh_scattering_scale,
        dust_storm,
    }
}

/// Weather presets matching the CARLA Python API.
///
/// These constants reproduce the static presets defined in the C++ `WeatherParameters` class.
///
/// # Examples
///
/// ```no_run
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use carla::{client::Client, rpc::weather};
///
/// let client = Client::connect("localhost", 2000, None)?;
/// let mut world = client.world()?;
/// world.set_weather(&weather::CLEAR_NOON)?;
/// # Ok(())
/// # }
/// ```

//                                              cloud  precip dep    wind   azim   alt    fogD   fogDist fogF   wet    scatI  mieS   raylS    dust
pub const DEFAULT: WeatherParameters = wp(
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const CLEAR_NOON: WeatherParameters = wp(
    5.0, 0.0, 0.0, 10.0, -1.0, 45.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const CLOUDY_NOON: WeatherParameters = wp(
    60.0, 0.0, 0.0, 10.0, -1.0, 45.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const WET_NOON: WeatherParameters = wp(
    5.0, 0.0, 50.0, 10.0, -1.0, 45.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const WET_CLOUDY_NOON: WeatherParameters = wp(
    60.0, 0.0, 50.0, 10.0, -1.0, 45.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const SOFT_RAIN_NOON: WeatherParameters = wp(
    20.0, 30.0, 50.0, 30.0, -1.0, 45.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const MID_RAIN_NOON: WeatherParameters = wp(
    60.0, 60.0, 60.0, 60.0, -1.0, 45.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const HARD_RAIN_NOON: WeatherParameters = wp(
    100.0, 100.0, 90.0, 100.0, -1.0, 45.0, 7.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);

pub const CLEAR_SUNSET: WeatherParameters = wp(
    5.0, 0.0, 0.0, 10.0, -1.0, 15.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const CLOUDY_SUNSET: WeatherParameters = wp(
    60.0, 0.0, 0.0, 10.0, -1.0, 15.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const WET_SUNSET: WeatherParameters = wp(
    5.0, 0.0, 50.0, 10.0, -1.0, 15.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const WET_CLOUDY_SUNSET: WeatherParameters = wp(
    60.0, 0.0, 50.0, 10.0, -1.0, 15.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const SOFT_RAIN_SUNSET: WeatherParameters = wp(
    20.0, 30.0, 50.0, 30.0, -1.0, 15.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const MID_RAIN_SUNSET: WeatherParameters = wp(
    60.0, 60.0, 60.0, 60.0, -1.0, 15.0, 3.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const HARD_RAIN_SUNSET: WeatherParameters = wp(
    100.0, 100.0, 90.0, 100.0, -1.0, 15.0, 7.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);

pub const CLEAR_NIGHT: WeatherParameters = wp(
    5.0, 0.0, 0.0, 10.0, -1.0, -90.0, 60.0, 75.0, 1.0, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const CLOUDY_NIGHT: WeatherParameters = wp(
    60.0, 0.0, 0.0, 10.0, -1.0, -90.0, 60.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const WET_NIGHT: WeatherParameters = wp(
    5.0, 0.0, 50.0, 10.0, -1.0, -90.0, 60.0, 75.0, 1.0, 60.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const WET_CLOUDY_NIGHT: WeatherParameters = wp(
    60.0, 0.0, 50.0, 10.0, -1.0, -90.0, 60.0, 0.75, 0.1, 60.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const SOFT_RAIN_NIGHT: WeatherParameters = wp(
    60.0, 30.0, 50.0, 30.0, -1.0, -90.0, 60.0, 0.75, 0.1, 60.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const MID_RAIN_NIGHT: WeatherParameters = wp(
    80.0, 60.0, 60.0, 60.0, -1.0, -90.0, 60.0, 0.75, 0.1, 80.0, 1.0, 0.03, 0.0331, 0.0,
);
pub const HARD_RAIN_NIGHT: WeatherParameters = wp(
    100.0, 100.0, 90.0, 100.0, -1.0, -90.0, 100.0, 0.75, 0.1, 100.0, 1.0, 0.03, 0.0331, 0.0,
);

pub const DUST_STORM: WeatherParameters = wp(
    100.0, 0.0, 0.0, 100.0, -1.0, 45.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 100.0,
);
