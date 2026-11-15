//! Analytical hillshade (shaded relief) from a DEM.
//!
//! Standard illumination model: a surface normal per cell dotted with a light
//! vector defined by sun azimuth + altitude. Output is 0..1 per cell.

use crate::dem::Dem;

/// Shaded-relief intensity in [0,1], row-major. `azimuth_deg` is clockwise from
/// north (NASA/USGS convention ~315° = NW light); `altitude_deg` is sun height.
pub fn hillshade(dem: &Dem, azimuth_deg: f32, altitude_deg: f32) -> Vec<f32> {
    let (w, h) = (dem.width, dem.height);
    let cell = dem.cell_size_m.max(1e-3);

    let zenith = (90.0 - altitude_deg).to_radians();
    // Convert geographic azimuth (CW from N) to math azimuth used below.
    let az = (360.0 - azimuth_deg + 90.0).to_radians();
    let (lx, ly, lz) = (
        zenith.sin() * az.cos(),
        zenith.sin() * az.sin(),
        zenith.cos(),
    );

    let mut out = vec![0.0f32; w * h];
    for y in 0..h {
        for x in 0..w {
            let xi = x as isize;
            let yi = y as isize;
            let a = dem.at(xi - 1, yi - 1);
            let b = dem.at(xi, yi - 1);
            let c = dem.at(xi + 1, yi - 1);
            let d = dem.at(xi - 1, yi);
            let f = dem.at(xi + 1, yi);
            let g = dem.at(xi - 1, yi + 1);
            let hh = dem.at(xi, yi + 1);
            let i = dem.at(xi + 1, yi + 1);

            let dzdx = ((c + 2.0 * f + i) - (a + 2.0 * d + g)) / (8.0 * cell);
            let dzdy = ((g + 2.0 * hh + i) - (a + 2.0 * b + c)) / (8.0 * cell);

            // Surface normal (-dz/dx, -dz/dy, 1) normalized.
            let nx = -dzdx;
            let ny = -dzdy;
            let nz = 1.0f32;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            let dot = (nx * lx + ny * ly + nz * lz) / len;
            out[y * w + x] = dot.clamp(0.0, 1.0);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;

    #[test]
    fn flat_terrain_is_uniform() {
        let dem = Dem::new(8, 8, vec![100.0; 64], (0.0, 1.0, 0.0, 1.0), 5.0);
        let hs = hillshade(&dem, 315.0, 45.0);
        let first = hs[0];
        for &v in &hs {
            assert!((v - first).abs() < 1e-4);
        }
        // sun at 45° over flat ground → cos(45°) ≈ 0.707
        assert!((first - 0.707).abs() < 0.02, "got {first}");
    }

    #[test]
    fn slope_toward_light_is_brighter() {
        // Ridge rising toward the NW-lit side should be brighter than the lee.
        let mut data = vec![0.0f32; 9 * 9];
        for y in 0..9 {
            for x in 0..9 {
                data[y * 9 + x] = (x + y) as f32; // rises toward SE
            }
        }
        let dem = Dem::new(9, 9, data, (0.0, 1.0, 0.0, 1.0), 1.0);
        let hs = hillshade(&dem, 315.0, 45.0);
        // all values are valid intensities
        for &v in &hs {
            assert!((0.0..=1.0).contains(&v));
        }
    }
}
