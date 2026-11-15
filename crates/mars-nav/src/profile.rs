//! Elevation + slope profile sampled along a planned path.

use crate::dem::Dem;
use crate::slope::SlopeGrid;

#[derive(Clone, Copy, Debug)]
pub struct ProfilePoint {
    /// Cumulative ground distance from the start, meters.
    pub dist_m: f32,
    pub elev_m: f32,
    pub slope_deg: f32,
}

/// Sample elevation/slope at each cell of `cells` (a planned path), accumulating
/// ground distance. `cell_size_m` comes from the DEM.
pub fn along_path(dem: &Dem, slope: &SlopeGrid, cells: &[(usize, usize)]) -> Vec<ProfilePoint> {
    let w = dem.width;
    let cell = dem.cell_size_m.max(1e-3);
    let mut out = Vec::with_capacity(cells.len());
    let mut dist = 0.0f32;
    for (i, &(x, y)) in cells.iter().enumerate() {
        if i > 0 {
            let (px, py) = cells[i - 1];
            let step = if px != x && py != y {
                std::f32::consts::SQRT_2
            } else {
                1.0
            };
            dist += step * cell;
        }
        out.push(ProfilePoint {
            dist_m: dist,
            elev_m: dem.data[y * w + x],
            slope_deg: slope.degrees[y * w + x],
        });
    }
    out
}

/// Average slope (deg) over a path, weighted by segment length.
pub fn average_slope(profile: &[ProfilePoint]) -> f32 {
    if profile.len() < 2 {
        return 0.0;
    }
    let mut num = 0.0;
    let mut den = 0.0;
    for pair in profile.windows(2) {
        let seg = (pair[1].dist_m - pair[0].dist_m).max(0.0);
        let s = 0.5 * (pair[0].slope_deg + pair[1].slope_deg);
        num += s * seg;
        den += seg;
    }
    if den > 0.0 {
        num / den
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;
    use crate::slope;

    #[test]
    fn distance_accumulates() {
        let dem = Dem::new(5, 5, vec![0.0; 25], (0.0, 1.0, 0.0, 1.0), 10.0);
        let sg = slope::compute(&dem);
        let cells = vec![(0, 0), (1, 0), (2, 0)];
        let prof = along_path(&dem, &sg, &cells);
        assert_eq!(prof.len(), 3);
        assert!((prof[2].dist_m - 20.0).abs() < 1e-3);
    }

    #[test]
    fn diagonal_is_longer() {
        let dem = Dem::new(5, 5, vec![0.0; 25], (0.0, 1.0, 0.0, 1.0), 10.0);
        let sg = slope::compute(&dem);
        let diag = along_path(&dem, &sg, &[(0, 0), (1, 1)]);
        assert!((diag[1].dist_m - 14.142).abs() < 0.1);
    }
}
