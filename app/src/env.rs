//! Modeled surface environment for the data panel.
//!
//! Physically-grounded but simplified: a diurnal insolation cycle drives surface
//! and air temperature, pressure follows the barometric relation for Mars, and
//! the radiation rate comes from the MSL/RAD surface figure. Everything here is
//! labelled MODELED in the UI — only elevation and imagery are measured data.

use bevy::prelude::*;

/// Mars scale height (m) and datum pressure (Pa).
const SCALE_HEIGHT_M: f32 = 11_100.0;
const DATUM_PRESSURE_PA: f32 = 610.0;
/// MSL/RAD surface dose-equivalent rate, mSv per sol.
const DOSE_MSV_PER_SOL: f32 = 0.66;
const SOL_HOURS: f32 = 24.66;

#[derive(Resource)]
pub struct Env {
    pub sol: u32,
    /// Local true solar time, hours.
    pub local_hour: f32,
}

impl Default for Env {
    fn default() -> Self {
        Env { sol: 1284, local_hour: 11.6 }
    }
}

impl Env {
    /// Advance the clock; `rate` is sol-hours per real second.
    pub fn advance(&mut self, dt: f32, rate: f32) {
        self.local_hour += dt * rate;
        while self.local_hour >= SOL_HOURS {
            self.local_hour -= SOL_HOURS;
            self.sol += 1;
        }
    }

    /// 0 at night, 1 at local noon.
    pub fn insolation(&self) -> f32 {
        let x = (self.local_hour - 6.0) / 12.33;
        if !(0.0..=1.0).contains(&x) {
            0.0
        } else {
            (x * std::f32::consts::PI).sin()
        }
    }

    pub fn solar_elevation_deg(&self) -> f32 {
        self.insolation() * 71.0
    }

    /// Ground temperature: swings hugely between day and night on Mars.
    pub fn surface_temp_c(&self, elev_m: f32) -> f32 {
        let s = self.insolation();
        let base = -88.0 + 68.0 * s;
        // mild lapse with height above the local datum
        base - 0.0022 * (elev_m + 2600.0)
    }

    /// Air temperature at ~1.5 m: much colder than the ground in daylight.
    pub fn air_temp_c(&self, elev_m: f32) -> f32 {
        self.surface_temp_c(elev_m) - 26.0 * self.insolation()
    }

    pub fn pressure_pa(&self, elev_m: f32) -> f32 {
        DATUM_PRESSURE_PA * (-elev_m / SCALE_HEIGHT_M).exp()
    }

    /// Slope winds pick up through the afternoon.
    pub fn wind_ms(&self) -> f32 {
        3.0 + 7.0 * self.insolation().powf(1.5)
    }

    pub fn dust_opacity(&self) -> f32 {
        0.48
    }

    pub fn dose_rate_usv_h(&self) -> f32 {
        DOSE_MSV_PER_SOL * 1000.0 / SOL_HOURS
    }

    pub fn clock(&self) -> String {
        let h = self.local_hour.floor() as u32;
        let m = ((self.local_hour - h as f32) * 60.0).round() as u32;
        format!("{:02}:{:02}", h.min(24), m.min(59))
    }

    /// Daylight / twilight / night, which gates visual localization in practice.
    pub fn light_state(&self) -> &'static str {
        let e = self.solar_elevation_deg();
        if e > 10.0 {
            "DAYLIGHT"
        } else if e > 0.5 {
            "TWILIGHT"
        } else {
            "NIGHT"
        }
    }
}
