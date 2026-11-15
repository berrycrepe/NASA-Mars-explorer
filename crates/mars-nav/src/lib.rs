//! # mars-nav
//!
//! Geospatial + GPS-free navigation compute for the Mars Explorer. Pure, I/O-free,
//! and dependency-free so it links straight into the native Bevy app and the
//! data-prep CLI.
//!
//! Pipeline: a [`dem::Dem`] feeds [`slope`], [`hillshade`], and [`render`]; [`cost`]
//! turns slope into a traversal surface that [`route`] plans a safest path over;
//! [`visibility`] derives landmark/comms line-of-sight. The [`nav`] module adds the
//! hybrid INS + visual-localization + EKF sensor-fusion simulation.

pub mod cost;
pub mod dem;
pub mod hillshade;
pub mod mission;
pub mod nav;
pub mod procedural;
pub mod profile;
pub mod render;
pub mod route;
pub mod slope;
pub mod visibility;

pub use dem::Dem;
pub use mission::{SuitParams, Telemetry};
pub use route::PathResult;
pub use slope::{Hazard, SlopeGrid};

/// Everything the app needs derived from a DEM, computed once and reused.
pub struct Scene {
    pub dem: Dem,
    pub slope: SlopeGrid,
}

impl Scene {
    pub fn from_dem(dem: Dem) -> Self {
        let slope = slope::compute(&dem);
        Scene { dem, slope }
    }

    /// Default cost surface for route planning.
    pub fn cost_surface(&self) -> Vec<f32> {
        cost::cost_surface(&self.slope, None, &cost::CostParams::default())
    }
}

#[cfg(test)]
mod integration {
    use super::*;

    #[test]
    fn end_to_end_plan_on_synthetic_jezero() {
        let dem = procedural::jezero_synthetic(256, 256);
        let scene = Scene::from_dem(dem);
        let cost = scene.cost_surface();

        // Plan from one corner of the floor to the delta region.
        let start = (200, 160);
        let goal = (90, 100);
        let path = route::a_star(&scene.dem, &cost, start, goal).expect("route exists");
        assert!(path.length_m > 0.0);
        assert_eq!(path.cells.first().unwrap(), &start);
        assert_eq!(path.cells.last().unwrap(), &goal);

        // Profile + telemetry derive cleanly.
        let prof = profile::along_path(&scene.dem, &scene.slope, &path.cells);
        assert_eq!(prof.len(), path.cells.len());
        let avg = profile::average_slope(&prof);
        let p = SuitParams::default();
        let wm = mission::walk_minutes(path.length_m / 1000.0, avg, &p);
        let tel = mission::integrate(path.length_m / 1000.0, wm, 120.0, 0.2, &p);
        assert!(tel.dose_msv > 0.0);
        assert!(tel.o2_remaining_min <= p.o2_minutes);
    }
}
