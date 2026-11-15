//! Layer image builders: turn DEM / slope grids into RGBA textures.
//!
//! Kept in core (not just wasm) so the native data-prep CLI can bake the same
//! textures offline. Output is tightly-packed RGBA8, row-major, top row first.

use crate::dem::Dem;
use crate::slope::{SlopeGrid, AVOID_DEG, CAUTION_DEG};

type Rgb = [f32; 3];

#[inline]
fn mix(a: Rgb, b: Rgb, t: f32) -> Rgb {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// Piecewise-linear colour ramp; `t` clamped to [0,1].
fn ramp(stops: &[Rgb], t: f32) -> Rgb {
    let t = t.clamp(0.0, 1.0);
    let n = stops.len() - 1;
    let seg = t * n as f32;
    let i = (seg.floor() as usize).min(n - 1);
    mix(stops[i], stops[i + 1], seg - i as f32)
}

#[inline]
fn put(out: &mut [u8], idx: usize, c: Rgb, a: u8) {
    out[idx * 4] = c[0].round().clamp(0.0, 255.0) as u8;
    out[idx * 4 + 1] = c[1].round().clamp(0.0, 255.0) as u8;
    out[idx * 4 + 2] = c[2].round().clamp(0.0, 255.0) as u8;
    out[idx * 4 + 3] = a;
}

const MARS_SHADOW: Rgb = [26.0, 17.0, 11.0];
const MARS_LIT: Rgb = [66.0, 46.0, 32.0];
const MARS_HI: Rgb = [214.0, 164.0, 112.0];

const HYPSO: [Rgb; 4] = [
    [45.0, 58.0, 134.0],
    [31.0, 157.0, 107.0],
    [231.0, 209.0, 74.0],
    [194.0, 59.0, 42.0],
];
const MAGMA: [Rgb; 5] = [
    [36.0, 26.0, 94.0],
    [120.0, 40.0, 70.0],
    [181.0, 50.0, 46.0],
    [255.0, 150.0, 70.0],
    [255.0, 207.0, 107.0],
];

/// Optical base: warm Mars tint driven by normalized elevation, shaded by the
/// supplied hillshade (0..1 per cell). Fully opaque.
pub fn base_rgba(dem: &Dem, hillshade: &[f32]) -> Vec<u8> {
    let n = dem.width * dem.height;
    let mut out = vec![0u8; n * 4];
    for i in 0..n {
        let (x, y) = (i % dem.width, i / dem.width);
        let hn = dem.normalized(x, y);
        let albedo = mix(MARS_LIT, MARS_HI, hn * 0.8 + 0.1);
        let shade = 0.25 + hillshade[i] * 0.85;
        let c = mix(MARS_SHADOW, albedo, shade.clamp(0.0, 1.0));
        put(&mut out, i, c, 255);
    }
    out
}

/// Hypsometric (elevation) tint. Semi-transparent so it blends over the base.
pub fn hypsometric_rgba(dem: &Dem) -> Vec<u8> {
    let n = dem.width * dem.height;
    let mut out = vec![0u8; n * 4];
    for i in 0..n {
        let (x, y) = (i % dem.width, i / dem.width);
        let c = ramp(&HYPSO, dem.normalized(x, y));
        put(&mut out, i, c, 160);
    }
    out
}

/// Slope hazard overlay: transparent where nominal, amber for caution, red for
/// avoid — so it reads as a warning layer on top of the base.
pub fn slope_hazard_rgba(slope: &SlopeGrid) -> Vec<u8> {
    let n = slope.width * slope.height;
    let mut out = vec![0u8; n * 4];
    for i in 0..n {
        let deg = slope.degrees[i];
        let (c, a): (Rgb, u8) = if deg >= AVOID_DEG {
            ([255.0, 91.0, 82.0], 180)
        } else if deg >= CAUTION_DEG {
            ([233.0, 186.0, 78.0], 130)
        } else {
            ([95.0, 212.0, 160.0], 0)
        };
        put(&mut out, i, c, a);
    }
    out
}

/// Thermal-inertia proxy overlay (modeled): rocky / steep ground reads hot,
/// smooth dusty ground reads cool. Replaced by real THEMIS data when available.
pub fn thermal_rgba(slope: &SlopeGrid) -> Vec<u8> {
    let n = slope.width * slope.height;
    let max = slope.max_deg.max(1.0);
    let mut out = vec![0u8; n * 4];
    for i in 0..n {
        let inertia = (0.25 + 0.75 * (slope.degrees[i] / max)).clamp(0.0, 1.0);
        let c = ramp(&MAGMA, inertia);
        put(&mut out, i, c, 150);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;
    use crate::slope;

    fn tilted(w: usize, h: usize) -> Dem {
        let mut d = vec![0.0f32; w * h];
        for y in 0..h {
            for x in 0..w {
                d[y * w + x] = (x + y) as f32;
            }
        }
        Dem::new(w, h, d, (0.0, 1.0, 0.0, 1.0), 10.0)
    }

    #[test]
    fn rgba_lengths() {
        let dem = tilted(16, 16);
        let sg = slope::compute(&dem);
        let hs = vec![0.5f32; 256];
        assert_eq!(base_rgba(&dem, &hs).len(), 256 * 4);
        assert_eq!(hypsometric_rgba(&dem).len(), 256 * 4);
        assert_eq!(slope_hazard_rgba(&sg).len(), 256 * 4);
        assert_eq!(thermal_rgba(&sg).len(), 256 * 4);
    }

    #[test]
    fn nominal_slope_is_transparent() {
        let dem = Dem::new(8, 8, vec![5.0; 64], (0.0, 1.0, 0.0, 1.0), 10.0);
        let sg = slope::compute(&dem);
        let rgba = slope_hazard_rgba(&sg);
        // flat ground → alpha 0 everywhere
        for i in 0..64 {
            assert_eq!(rgba[i * 4 + 3], 0);
        }
    }
}
