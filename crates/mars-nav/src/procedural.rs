//! Synthetic Jezero-like DEM — a stand-in used for development and as a fallback
//! when real NASA assets are not present. Ported from the HTML prototype's
//! value-noise terrain, but emitting physical elevations (meters, MOLA-ish) and
//! a real Jezero bounding box so downstream code behaves identically to real data.

use crate::dem::Dem;

// Local bounding box around the Jezero western delta (planetocentric E lon / lat).
pub const WEST: f64 = 77.40;
pub const EAST: f64 = 77.52;
pub const SOUTH: f64 = 18.38;
pub const NORTH: f64 = 18.50;

const FLOOR_M: f32 = -2635.0;
const RELIEF_M: f32 = 72.0;
const MARS_RADIUS_M: f64 = 3_389_500.0;

struct Noise {
    lattice: Vec<f32>,
    size: usize,
}

impl Noise {
    fn new(seed: u32, size: usize) -> Self {
        let mut s = seed;
        let mut next = move || {
            // mulberry32
            s = s.wrapping_add(0x6D2B_79F5);
            let mut t = s;
            t = (t ^ (t >> 15)).wrapping_mul(1 | t);
            t ^= t.wrapping_add((t ^ (t >> 7)).wrapping_mul(61 | t));
            (t ^ (t >> 14)) as f32 / 4_294_967_296.0
        };
        let lattice = (0..size * size).map(|_| next()).collect();
        Noise { lattice, size }
    }

    #[inline]
    fn val(&self, xi: i32, yi: i32) -> f32 {
        let s = self.size as i32;
        let x = (xi.rem_euclid(s)) as usize;
        let y = (yi.rem_euclid(s)) as usize;
        self.lattice[y * self.size + x]
    }

    fn value(&self, x: f32, y: f32) -> f32 {
        let xi = x.floor() as i32;
        let yi = y.floor() as i32;
        let xf = x - x.floor();
        let yf = y - y.floor();
        let sm = |t: f32| t * t * (3.0 - 2.0 * t);
        let (u, v) = (sm(xf), sm(yf));
        let a = self.val(xi, yi);
        let b = self.val(xi + 1, yi);
        let c = self.val(xi, yi + 1);
        let d = self.val(xi + 1, yi + 1);
        let top = a + (b - a) * u;
        let bot = c + (d - c) * u;
        top + (bot - top) * v
    }

    fn fbm(&self, x: f32, y: f32) -> f32 {
        let mut s = 0.0;
        let mut amp = 0.5;
        let mut f = 1.0;
        let mut norm = 0.0;
        for _ in 0..5 {
            s += amp * self.value(x * f, y * f);
            norm += amp;
            f *= 2.0;
            amp *= 0.5;
        }
        s / norm
    }
}

fn feature(u: f32, v: f32) -> f32 {
    let (cx, cy) = (0.56f32, 0.52f32);
    let (dx, dy) = (u - cx, v - cy);
    let d = (dx * dx + dy * dy).sqrt();
    let mut h = 0.0;
    // crater rim ring
    h += 0.55 * (-((d - 0.47) / 0.055).powi(2)).exp();
    // interior depression
    h += -0.22 * (1.0 / (1.0 + ((d - 0.44) * 22.0).exp()));
    // western delta fan
    let (fdx, fdy) = (u - 0.34, v - 0.50);
    let fan = (-(fdx * fdx / 0.016 + fdy * fdy / 0.05)).exp();
    h += 0.30 * fan;
    // layered foreset ridges
    if u < 0.45 {
        h += 0.05 * fan * (u * 70.0 + v * 8.0).sin();
    }
    // Kodiak butte
    h += 0.34 * (-(((u - 0.385) / 0.016).powi(2) + ((v - 0.395) / 0.016).powi(2))).exp();
    h
}

/// Build a synthetic Jezero DEM `width`×`height`.
pub fn jezero_synthetic(width: usize, height: usize) -> Dem {
    let noise = Noise::new(20_260_325, 256);
    let mut data = vec![0.0f32; width * height];
    for y in 0..height {
        let v = y as f32 / (height - 1).max(1) as f32;
        for x in 0..width {
            let u = x as f32 / (width - 1).max(1) as f32;
            let h = 0.55 * noise.fbm(u * 7.0 + 3.0, v * 7.0 + 11.0) + feature(u, v);
            data[y * width + x] = FLOOR_M + h * RELIEF_M;
        }
    }

    // Approximate ground sample distance from the bbox at the site latitude.
    let mid_lat = ((NORTH + SOUTH) * 0.5).to_radians();
    let deg_lon_m = MARS_RADIUS_M * mid_lat.cos() * std::f64::consts::PI / 180.0;
    let span_m = (EAST - WEST) * deg_lon_m;
    let cell = (span_m / width as f64) as f32;

    Dem::new(width, height, data, (WEST, EAST, SOUTH, NORTH), cell)
}

/// Seed waypoints (lon, lat, name) matching the prototype traverse, for the
/// fallback manifest. The data-prep CLI overrides these for real sites.
pub fn seed_waypoints() -> Vec<(f64, f64, &'static str)> {
    // Derived from the prototype's normalized positions mapped into the bbox.
    let map = |u: f64, v: f64| (WEST + u * (EAST - WEST), NORTH - v * (NORTH - SOUTH));
    vec![
        {
            let (lon, lat) = map(0.735, 0.622);
            (lon, lat, "Butler Station")
        },
        {
            let (lon, lat) = map(0.629, 0.675);
            (lon, lat, "Máaz Volcanic Contact")
        },
        {
            let (lon, lat) = map(0.497, 0.581);
            (lon, lat, "Delta Front Overlook")
        },
        {
            let (lon, lat) = map(0.416, 0.506);
            (lon, lat, "Carbonate Shoreline")
        },
        {
            let (lon, lat) = map(0.386, 0.397);
            (lon, lat, "Kodiak Remnant")
        },
        {
            let (lon, lat) = map(0.580, 0.469);
            (lon, lat, "Cache Drop & Turn")
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn produces_realistic_elevations() {
        let dem = jezero_synthetic(128, 128);
        // Jezero floor is roughly −2600 m; allow a generous band.
        assert!(dem.min > -2700.0 && dem.min < -2500.0, "min {}", dem.min);
        assert!(dem.max > dem.min);
        assert!(dem.max - dem.min > 20.0, "should have real relief");
    }

    #[test]
    fn cell_size_is_sane() {
        let dem = jezero_synthetic(512, 512);
        // ~6.7 km / 512 ≈ 13 m per cell; bounded check.
        assert!(dem.cell_size_m > 5.0 && dem.cell_size_m < 40.0, "cell {}", dem.cell_size_m);
    }

    #[test]
    fn waypoints_inside_bbox() {
        for (lon, lat, _) in seed_waypoints() {
            assert!(lon >= WEST && lon <= EAST);
            assert!(lat >= SOUTH && lat <= NORTH);
        }
    }
}
