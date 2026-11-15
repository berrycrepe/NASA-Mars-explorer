//! EVA consumables + radiation dose model.
//!
//! Numbers are intentionally the same order as the prototype console and are
//! grounded in public figures (e.g. MSL/RAD measured a surface dose-equivalent
//! rate of ~0.64–0.66 mSv/sol). This is a planning model, not flight software.

/// Suit / crew parameters for an EVA.
#[derive(Clone, Copy, Debug)]
pub struct SuitParams {
    /// Portable life-support O2 endurance, minutes.
    pub o2_minutes: f32,
    /// Suit battery endurance at nominal draw, minutes.
    pub power_minutes: f32,
    /// Suited walking pace on nominal ground, km/h.
    pub walk_speed_kmh: f32,
    /// Baseline surface GCR dose-equivalent rate, mSv per sol (24.6 h).
    pub dose_rate_msv_sol: f32,
    /// Fraction of the baseline dose blocked while in deep terrain shadow.
    pub shadow_shielding: f32,
    /// Extra time multiplier per unit of average slope (deg) — encumbrance.
    pub slope_time_gain: f32,
}

impl Default for SuitParams {
    fn default() -> Self {
        SuitParams {
            o2_minutes: 8.0 * 60.0,
            power_minutes: 8.0 * 60.0,
            walk_speed_kmh: 0.9,
            dose_rate_msv_sol: 0.66,
            shadow_shielding: 0.35,
            slope_time_gain: 0.015,
        }
    }
}

/// A single integrated telemetry sample at some point along the traverse.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Telemetry {
    pub distance_km: f32,
    pub elapsed_min: f32,
    pub o2_remaining_min: f32,
    pub power_pct: f32,
    pub dose_msv: f32,
}

const SOL_MINUTES: f32 = 24.6 * 60.0;

/// Integrate consumables/dose for a traverse of `distance_km` travelled,
/// `walk_minutes` spent moving, `dwell_minutes` spent stationary at stations,
/// and a `shadow_fraction` (0..1) of time spent in shielding terrain shadow.
pub fn integrate(
    distance_km: f32,
    walk_minutes: f32,
    dwell_minutes: f32,
    shadow_fraction: f32,
    p: &SuitParams,
) -> Telemetry {
    let elapsed = walk_minutes + dwell_minutes;
    let o2_remaining = (p.o2_minutes - elapsed).max(0.0);
    // Battery drains a bit slower than O2 is consumed (life support dominates).
    let power_pct = (100.0 - (elapsed / p.power_minutes) * 100.0 * 0.82).clamp(0.0, 100.0);
    let effective_rate =
        p.dose_rate_msv_sol * (1.0 - shadow_fraction.clamp(0.0, 1.0) * p.shadow_shielding);
    let dose = effective_rate * (elapsed / SOL_MINUTES);
    Telemetry {
        distance_km,
        elapsed_min: elapsed,
        o2_remaining_min: o2_remaining,
        power_pct,
        dose_msv: dose,
    }
}

/// Minutes of walking for a distance at a given average slope, accounting for
/// suited encumbrance on grade.
pub fn walk_minutes(distance_km: f32, avg_slope_deg: f32, p: &SuitParams) -> f32 {
    let base = distance_km / p.walk_speed_kmh * 60.0;
    base * (1.0 + avg_slope_deg.max(0.0) * p.slope_time_gain)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dose_scales_with_time() {
        let p = SuitParams::default();
        let a = integrate(1.0, 60.0, 0.0, 0.0, &p);
        let b = integrate(2.0, 120.0, 0.0, 0.0, &p);
        assert!(b.dose_msv > a.dose_msv);
        // ~0.66 mSv/sol over 1 h ≈ 0.0268 mSv
        assert!((a.dose_msv - 0.0268).abs() < 0.002, "{}", a.dose_msv);
    }

    #[test]
    fn shadow_reduces_dose() {
        let p = SuitParams::default();
        let open = integrate(1.0, 60.0, 0.0, 0.0, &p);
        let shadowed = integrate(1.0, 60.0, 0.0, 1.0, &p);
        assert!(shadowed.dose_msv < open.dose_msv);
    }

    #[test]
    fn o2_depletes_to_zero_floor() {
        let p = SuitParams::default();
        let t = integrate(50.0, 1000.0, 0.0, 0.0, &p);
        assert_eq!(t.o2_remaining_min, 0.0);
        assert_eq!(t.power_pct, 0.0);
    }

    #[test]
    fn slope_slows_walking() {
        let p = SuitParams::default();
        assert!(walk_minutes(1.0, 20.0, &p) > walk_minutes(1.0, 0.0, &p));
    }
}
