//! Terrain line-of-sight for comms/relay coverage.
//!
//! Over a few-kilometre local area on Mars, planetary curvature is negligible
//! (~0.2 m drop over 3 km), so we treat the ground as locally flat and test only
//! terrain occlusion along the straight line between observer and target.

use crate::dem::Dem;

/// Is `target` visible from `observer`? Heights are antenna/mast heights in
/// meters added to the ground elevation at each end.
pub fn line_of_sight(
    dem: &Dem,
    observer: (usize, usize),
    observer_h: f32,
    target: (usize, usize),
    target_h: f32,
) -> bool {
    let w = dem.width;
    let oz = dem.data[observer.1 * w + observer.0] + observer_h;
    let tz = dem.data[target.1 * w + target.0] + target_h;

    let (x0, y0) = (observer.0 as f32, observer.1 as f32);
    let (x1, y1) = (target.0 as f32, target.1 as f32);
    let dx = x1 - x0;
    let dy = y1 - y0;
    let dist = (dx * dx + dy * dy).sqrt();
    let steps = dist.ceil() as usize;
    if steps == 0 {
        return true;
    }

    for s in 1..steps {
        let t = s as f32 / steps as f32;
        let gx = x0 + dx * t;
        let gy = y0 + dy * t;
        let ground = dem.sample(gx, gy);
        // Sightline height at this fraction (linear interp of endpoint heights).
        let sight = oz + (tz - oz) * t;
        if ground > sight {
            return false;
        }
    }
    true
}

/// Fraction (0..1) of sampled waypoint cells visible from `observer` — a quick
/// comms-coverage score for a relay sited at the observer.
pub fn coverage_fraction(
    dem: &Dem,
    observer: (usize, usize),
    observer_h: f32,
    targets: &[(usize, usize)],
    target_h: f32,
) -> f32 {
    if targets.is_empty() {
        return 1.0;
    }
    let visible = targets
        .iter()
        .filter(|&&t| line_of_sight(dem, observer, observer_h, t, target_h))
        .count();
    visible as f32 / targets.len() as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;

    #[test]
    fn flat_ground_is_visible() {
        let dem = Dem::new(20, 1, vec![0.0; 20], (0.0, 1.0, 0.0, 1.0), 10.0);
        assert!(line_of_sight(&dem, (0, 0), 2.0, (19, 0), 2.0));
    }

    #[test]
    fn ridge_blocks_sight() {
        // A tall spike in the middle blocks two low endpoints.
        let mut data = vec![0.0f32; 21];
        data[10] = 500.0;
        let dem = Dem::new(21, 1, data, (0.0, 1.0, 0.0, 1.0), 10.0);
        assert!(!line_of_sight(&dem, (0, 0), 2.0, (20, 0), 2.0));
    }

    #[test]
    fn tall_mast_sees_over_low_rise() {
        let mut data = vec![0.0f32; 21];
        data[10] = 5.0;
        let dem = Dem::new(21, 1, data, (0.0, 1.0, 0.0, 1.0), 10.0);
        // 2 m mast is blocked, 50 m tower clears it.
        assert!(!line_of_sight(&dem, (0, 0), 2.0, (20, 0), 2.0));
        assert!(line_of_sight(&dem, (0, 0), 50.0, (20, 0), 50.0));
    }
}
