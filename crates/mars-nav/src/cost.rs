//! Traversal cost surface for route planning.
//!
//! Per-cell cost encodes how expensive / dangerous it is for a suited walker to
//! cross a cell, as a function of slope (primary) plus optional roughness. Cells
//! above the "avoid" slope are made very expensive but not strictly infinite, so
//! the planner can still thread a gap if no safer path exists.

use crate::slope::{SlopeGrid, AVOID_DEG, CAUTION_DEG};

#[derive(Clone, Copy, Debug)]
pub struct CostParams {
    /// Flat-ground baseline cost multiplier.
    pub base: f32,
    /// Extra cost per degree of slope within the nominal band.
    pub slope_gain: f32,
    /// Multiplier applied once slope exceeds the caution threshold.
    pub caution_penalty: f32,
    /// Multiplier applied once slope exceeds the avoid threshold.
    pub avoid_penalty: f32,
    /// Weight on the optional roughness layer (0..1 input).
    pub roughness_gain: f32,
}

impl Default for CostParams {
    fn default() -> Self {
        CostParams {
            base: 1.0,
            slope_gain: 0.12,
            caution_penalty: 4.0,
            avoid_penalty: 40.0,
            roughness_gain: 2.5,
        }
    }
}

/// Build a per-cell cost surface (row-major, same dims as `slope`).
/// `roughness` is optional, values in [0,1]; pass `None` to ignore.
pub fn cost_surface(slope: &SlopeGrid, roughness: Option<&[f32]>, p: &CostParams) -> Vec<f32> {
    let n = slope.width * slope.height;
    let mut out = vec![0.0f32; n];
    for idx in 0..n {
        let deg = slope.degrees[idx];
        let mut c = p.base + deg * p.slope_gain;
        if deg >= AVOID_DEG {
            c *= p.avoid_penalty;
        } else if deg >= CAUTION_DEG {
            c *= p.caution_penalty;
        }
        if let Some(r) = roughness {
            c += r[idx].clamp(0.0, 1.0) * p.roughness_gain;
        }
        out[idx] = c;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;
    use crate::slope;

    #[test]
    fn steeper_costs_more() {
        // Two ramps: gentle vs steep, compare interior cost.
        let gentle = ramp(0.5);
        let steep = ramp(5.0);
        let sg = slope::compute(&gentle);
        let ss = slope::compute(&steep);
        let p = CostParams::default();
        let cg = cost_surface(&sg, None, &p);
        let cs = cost_surface(&ss, None, &p);
        let w = gentle.width;
        assert!(cs[2 * w + 2] > cg[2 * w + 2]);
    }

    fn ramp(rise_per_cell: f32) -> Dem {
        let (w, h) = (5, 5);
        let mut data = vec![0.0f32; w * h];
        for y in 0..h {
            for x in 0..w {
                data[y * w + x] = x as f32 * rise_per_cell;
            }
        }
        Dem::new(w, h, data, (0.0, 1.0, 0.0, 1.0), 10.0)
    }
}
