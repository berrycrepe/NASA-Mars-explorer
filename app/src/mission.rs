//! Astronaut state, designated target, and the GPS-free navigation estimate.
//!
//! There is no predefined route. The astronaut moves freely; when a target is
//! designated the safest path to it is planned on demand, and navigation guidance
//! (bearing, range, route) is produced for that point.

use bevy::prelude::*;
use mars_nav::nav::NavSim;

use crate::route::PlannedRoute;

/// Realistic suited walking pace. The view may move faster, but the INS/EKF always
/// integrates as if walking at this pace, so uncertainty grows with distance
/// covered rather than with wall-clock fast-forward.
pub const REAL_WALK_SPEED: f64 = 1.3;

#[derive(Resource)]
pub struct Astronaut {
    /// World position, on the surface.
    pub pos: Vec3,
    /// Travel heading (radians), atan2(z, x).
    pub heading: f32,
}

impl Astronaut {
    pub fn new(pos: Vec3) -> Self {
        Astronaut { pos, heading: 0.0 }
    }
}

/// The designated point and the route to it (both optional — none at start).
#[derive(Resource)]
pub struct Mission {
    /// Standalone landmark. Not connected to anything.
    pub base: Vec3,
    pub target: Option<Vec3>,
    pub route: Option<PlannedRoute>,
    /// Cumulative distance along `route.pts`.
    pub arc: Vec<f32>,
    pub total: f32,
    /// Auto-navigate along the planned route.
    pub following: bool,
    pub progress: f32,
}

impl Mission {
    pub fn new(base: Vec3) -> Self {
        Mission {
            base,
            target: None,
            route: None,
            arc: Vec::new(),
            total: 0.0,
            following: false,
            progress: 0.0,
        }
    }

    pub fn set_route(&mut self, route: PlannedRoute) {
        let mut arc = vec![0.0f32];
        for i in 1..route.pts.len() {
            let d = route.pts[i].distance(route.pts[i - 1]);
            arc.push(arc[i - 1] + d);
        }
        self.total = *arc.last().unwrap_or(&0.0);
        self.arc = arc;
        self.route = Some(route);
        self.progress = 0.0;
    }

    pub fn clear(&mut self) {
        self.target = None;
        self.route = None;
        self.arc.clear();
        self.total = 0.0;
        self.following = false;
        self.progress = 0.0;
    }

    /// Position + horizontal tangent at a fraction along the planned route.
    pub fn sample(&self, frac: f32) -> Option<(Vec3, Vec3)> {
        let route = self.route.as_ref()?;
        let pts = &route.pts;
        if pts.len() < 2 {
            return None;
        }
        let target = frac.clamp(0.0, 1.0) * self.total;
        let mut i = 1;
        while i < self.arc.len() && self.arc[i] < target {
            i += 1;
        }
        let i0 = (i - 1).min(pts.len() - 1);
        let i1 = i.min(pts.len() - 1);
        let seg = (self.arc[i1] - self.arc[i0]).max(1e-4);
        let t = ((target - self.arc[i0]) / seg).clamp(0.0, 1.0);
        let pos = pts[i0].lerp(pts[i1], t);
        // Look several samples ahead so the tangent (and the camera) stays smooth
        // through corners instead of snapping segment to segment.
        let ahead = pts[(i1 + 3).min(pts.len() - 1)];
        let mut tan = ahead - pos;
        tan.y = 0.0;
        if tan.length_squared() < 1e-6 {
            tan = Vec3::Z;
        }
        Some((pos, tan.normalize()))
    }
}

/// The GPS-free navigation estimate (INS + visual localization fused by an EKF).
#[derive(Resource)]
pub struct NavState {
    pub sim: NavSim,
}

impl NavState {
    pub fn new(pos: Vec3, heading: f32) -> Self {
        NavState { sim: NavSim::new(pos.x as f64, pos.z as f64, heading as f64) }
    }
}

// ---- scene markers ----
#[derive(Component)]
pub struct BaseMarker;
#[derive(Component)]
pub struct TargetMarker;
#[derive(Component)]
pub struct AstronautMarker;
#[derive(Component)]
pub struct EstMarker;
#[derive(Component)]
pub struct UncertaintyRing;
#[derive(Component)]
pub struct RouteRibbon;
