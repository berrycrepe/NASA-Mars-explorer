//! Slope, aspect, and suited-walker hazard classification.
//!
//! Slope is computed with Horn's 3×3 method (the same estimator ArcGIS/GDAL use),
//! which is robust to single-cell noise in the DEM.

use crate::dem::Dem;

#[derive(Clone, Debug)]
pub struct SlopeGrid {
    pub width: usize,
    pub height: usize,
    /// Slope in degrees, row-major.
    pub degrees: Vec<f32>,
    /// Downslope aspect in degrees clockwise from north (0..360), row-major.
    pub aspect: Vec<f32>,
    pub max_deg: f32,
}

impl SlopeGrid {
    #[inline]
    pub fn deg_at(&self, x: usize, y: usize) -> f32 {
        self.degrees[y * self.width + x]
    }
}

/// Hazard tier for a suited astronaut on foot carrying equipment.
///
/// Thresholds mirror the prototype console and are deliberately conservative:
/// a pressurized suit plus hand tools makes even moderate grades costly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hazard {
    /// < 10° — nominal walking.
    Nominal,
    /// 10°–20° — caution: slower, higher exertion.
    Caution,
    /// > 20° — avoid: route around if at all possible.
    Avoid,
}

pub const CAUTION_DEG: f32 = 10.0;
pub const AVOID_DEG: f32 = 20.0;

#[inline]
pub fn classify(deg: f32) -> Hazard {
    if deg < CAUTION_DEG {
        Hazard::Nominal
    } else if deg < AVOID_DEG {
        Hazard::Caution
    } else {
        Hazard::Avoid
    }
}

/// Compute slope + aspect for the whole DEM.
pub fn compute(dem: &Dem) -> SlopeGrid {
    let (w, h) = (dem.width, dem.height);
    let mut degrees = vec![0.0f32; w * h];
    let mut aspect = vec![0.0f32; w * h];
    let cell = dem.cell_size_m.max(1e-3);
    let mut max_deg = 0.0f32;

    for y in 0..h {
        for x in 0..w {
            let xi = x as isize;
            let yi = y as isize;
            // Horn 3×3 window (clamped at edges):
            //   a b c
            //   d e f
            //   g h i
            let a = dem.at(xi - 1, yi - 1);
            let b = dem.at(xi, yi - 1);
            let c = dem.at(xi + 1, yi - 1);
            let d = dem.at(xi - 1, yi);
            let f = dem.at(xi + 1, yi);
            let g = dem.at(xi - 1, yi + 1);
            let hh = dem.at(xi, yi + 1);
            let i = dem.at(xi + 1, yi + 1);

            let dzdx = ((c + 2.0 * f + i) - (a + 2.0 * d + g)) / (8.0 * cell);
            // y increases southward, so north-south gradient sign is handled in aspect.
            let dzdy = ((g + 2.0 * hh + i) - (a + 2.0 * b + c)) / (8.0 * cell);

            let rise = (dzdx * dzdx + dzdy * dzdy).sqrt();
            let deg = rise.atan().to_degrees();
            degrees[y * w + x] = deg;
            if deg > max_deg {
                max_deg = deg;
            }

            // Aspect: direction the slope faces, clockwise from north.
            let mut asp = dzdy.atan2(-dzdx).to_degrees();
            asp = 90.0 - asp;
            if asp < 0.0 {
                asp += 360.0;
            }
            aspect[y * w + x] = if rise < 1e-6 { -1.0 } else { asp % 360.0 };
        }
    }

    SlopeGrid {
        width: w,
        height: h,
        degrees,
        aspect,
        max_deg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;

    fn ramp_east(w: usize, h: usize, rise_per_cell: f32, cell: f32) -> Dem {
        // elevation increases by `rise_per_cell` each step in +x.
        let mut data = vec![0.0f32; w * h];
        for y in 0..h {
            for x in 0..w {
                data[y * w + x] = x as f32 * rise_per_cell;
            }
        }
        Dem::new(w, h, data, (0.0, 1.0, 0.0, 1.0), cell)
    }

    #[test]
    fn flat_is_zero_slope() {
        let dem = Dem::new(5, 5, vec![42.0; 25], (0.0, 1.0, 0.0, 1.0), 10.0);
        let s = compute(&dem);
        for &d in &s.degrees {
            assert!(d.abs() < 1e-3);
        }
    }

    #[test]
    fn known_45_degree_ramp() {
        // rise == run → 45°. Use interior cell to avoid edge clamping bias.
        let dem = ramp_east(5, 5, 10.0, 10.0);
        let s = compute(&dem);
        let center = s.deg_at(2, 2);
        assert!((center - 45.0).abs() < 0.5, "expected ~45°, got {center}");
    }

    #[test]
    fn classify_tiers() {
        assert_eq!(classify(3.0), Hazard::Nominal);
        assert_eq!(classify(14.0), Hazard::Caution);
        assert_eq!(classify(31.0), Hazard::Avoid);
    }
}
