//! On-demand route planning: the safest path between any two points the user
//! picks, over a slope-derived traversal cost surface. No routes exist until the
//! astronaut designates a target.

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;
use mars_nav::{cost, route, slope, slope::SlopeGrid};

use crate::terrain::Terrain;

/// Slope + traversal cost for the loaded terrain. Computed once, reused for every
/// route the astronaut requests.
#[derive(Resource)]
pub struct NavGrid {
    pub slope: SlopeGrid,
    pub cost: Vec<f32>,
}

impl NavGrid {
    pub fn build(terrain: &Terrain) -> Self {
        let slope = slope::compute(&terrain.dem);
        let cost = cost::cost_surface(&slope, None, &cost::CostParams::default());
        NavGrid { slope, cost }
    }

    /// Slope in degrees at a world position.
    pub fn slope_at(&self, terrain: &Terrain, world: Vec3) -> f32 {
        let (gx, gy) = terrain.world_to_grid(world.x, world.z);
        let x = (gx.round().max(0.0) as usize).min(self.slope.width - 1);
        let y = (gy.round().max(0.0) as usize).min(self.slope.height - 1);
        self.slope.deg_at(x, y)
    }
}

/// One sample along the route, for the profile chart.
pub struct RouteSample {
    pub dist_m: f32,
    pub elev_m: f32,
    pub slope_deg: f32,
}

pub struct PlannedRoute {
    /// Smoothed world-space path, lying on the surface.
    pub pts: Vec<Vec3>,
    /// Ground length of the planned path, meters.
    pub length_m: f32,
    /// Total climb and drop along the path, meters.
    pub ascent_m: f32,
    pub descent_m: f32,
    pub max_slope_deg: f32,
    /// Share of the route in each hazard tier: nominal, caution, avoid.
    pub frac: [f32; 3],
    /// Evenly-spaced samples for the elevation profile.
    pub samples: Vec<RouteSample>,
    pub elev_min: f32,
    pub elev_max: f32,
}

/// The path sits just clear of the ground — high enough to avoid z-fighting from
/// orbit, low enough that at eye level it reads as a marking painted on the
/// surface rather than a band floating in front of the astronaut.
const PATH_LIFT: f32 = 0.4;

/// Plan the safest route between two world points. `None` if unreachable or if the
/// two points land on the same cell.
pub fn plan_between(
    terrain: &Terrain,
    grid: &NavGrid,
    from: Vec3,
    to: Vec3,
) -> Option<PlannedRoute> {
    let dem = &terrain.dem;
    let clamp = |v: f32, max: usize| (v.round().max(0.0) as usize).min(max);
    let (fx, fy) = terrain.world_to_grid(from.x, from.z);
    let (tx, ty) = terrain.world_to_grid(to.x, to.z);
    let start = (clamp(fx, dem.width - 1), clamp(fy, dem.height - 1));
    let goal = (clamp(tx, dem.width - 1), clamp(ty, dem.height - 1));
    if start == goal {
        return None;
    }

    let path = route::a_star(dem, &grid.cost, start, goal)?;
    let raw: Vec<Vec3> = path
        .cells
        .iter()
        .map(|&(gx, gy)| terrain.grid_to_world(gx as f32, gy as f32, PATH_LIFT))
        .collect();
    let pts = smooth_path(terrain, &raw, 4);
    Some(profile_route(terrain, grid, pts))
}

const PROFILE_SAMPLES: usize = 64;

/// Measure the smoothed path: length, climb/drop, slope extremes, hazard mix, and
/// an evenly-spaced elevation profile.
fn profile_route(terrain: &Terrain, grid: &NavGrid, pts: Vec<Vec3>) -> PlannedRoute {
    let elev_at = |p: Vec3| {
        let (gx, gy) = terrain.world_to_grid(p.x, p.z);
        terrain.dem.sample(gx, gy)
    };

    // cumulative ground distance
    let mut arc = vec![0.0f32];
    for i in 1..pts.len() {
        let d = Vec3::new(pts[i].x - pts[i - 1].x, 0.0, pts[i].z - pts[i - 1].z).length();
        arc.push(arc[i - 1] + d);
    }
    let length_m = *arc.last().unwrap_or(&0.0);

    let (mut ascent, mut descent, mut max_slope) = (0.0f32, 0.0f32, 0.0f32);
    let mut tier_len = [0.0f32; 3];
    for i in 1..pts.len() {
        let de = elev_at(pts[i]) - elev_at(pts[i - 1]);
        if de > 0.0 { ascent += de } else { descent -= de }
        let mid = pts[i - 1].lerp(pts[i], 0.5);
        let sl = grid.slope_at(terrain, mid);
        max_slope = max_slope.max(sl);
        let seg = arc[i] - arc[i - 1];
        let t = if sl >= 20.0 { 2 } else if sl >= 10.0 { 1 } else { 0 };
        tier_len[t] += seg;
    }
    let total = length_m.max(1e-3);
    let frac = [tier_len[0] / total, tier_len[1] / total, tier_len[2] / total];

    // evenly spaced profile samples
    let mut samples = Vec::with_capacity(PROFILE_SAMPLES);
    let (mut emin, mut emax) = (f32::MAX, f32::MIN);
    for k in 0..PROFILE_SAMPLES {
        let target = length_m * k as f32 / (PROFILE_SAMPLES - 1).max(1) as f32;
        let mut i = 1;
        while i < arc.len() && arc[i] < target {
            i += 1;
        }
        let i0 = (i - 1).min(pts.len() - 1);
        let i1 = i.min(pts.len() - 1);
        let seg = (arc[i1] - arc[i0]).max(1e-4);
        let t = ((target - arc[i0]) / seg).clamp(0.0, 1.0);
        let p = pts[i0].lerp(pts[i1], t);
        let e = elev_at(p);
        emin = emin.min(e);
        emax = emax.max(e);
        samples.push(RouteSample { dist_m: target, elev_m: e, slope_deg: grid.slope_at(terrain, p) });
    }

    PlannedRoute {
        pts,
        length_m,
        ascent_m: ascent,
        descent_m: descent,
        max_slope_deg: max_slope,
        frac,
        samples,
        elev_min: emin,
        elev_max: emax,
    }
}

/// Chaikin corner-cutting, re-projected onto the surface. Removes the 8-connected
/// A* staircase so the line reads naturally and the walk follows it smoothly.
fn smooth_path(terrain: &Terrain, pts: &[Vec3], iters: usize) -> Vec<Vec3> {
    let mut cur = pts.to_vec();
    for _ in 0..iters {
        if cur.len() < 3 {
            break;
        }
        let mut next = Vec::with_capacity(cur.len() * 2);
        next.push(cur[0]);
        for w in cur.windows(2) {
            next.push(w[0].lerp(w[1], 0.25));
            next.push(w[0].lerp(w[1], 0.75));
        }
        next.push(*cur.last().unwrap());
        cur = next;
    }
    for p in cur.iter_mut() {
        let (gx, gy) = terrain.world_to_grid(p.x, p.z);
        p.y = terrain.height_world(gx, gy) + PATH_LIFT;
    }
    cur
}

/// Flat ground ribbon for the planned path — hugs the surface so it guides in
/// first-person without blocking the view.
pub fn build_ribbon_mesh(pts: &[Vec3], width: f32) -> Mesh {
    let hw = width * 0.5;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(pts.len() * 2);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(pts.len() * 2);
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(pts.len() * 2);

    for i in 0..pts.len() {
        let prev = pts[i.saturating_sub(1)];
        let next = pts[(i + 1).min(pts.len() - 1)];
        let mut dir = next - prev;
        dir.y = 0.0;
        if dir.length_squared() < 1e-6 {
            dir = Vec3::Z;
        }
        dir = dir.normalize();
        let perp = Vec3::new(dir.z, 0.0, -dir.x);
        let l = pts[i] - perp * hw;
        let r = pts[i] + perp * hw;
        positions.push([l.x, l.y, l.z]);
        positions.push([r.x, r.y, r.z]);
        normals.push([0.0, 1.0, 0.0]);
        normals.push([0.0, 1.0, 0.0]);
        uvs.push([0.0, i as f32]);
        uvs.push([1.0, i as f32]);
    }

    let mut indices = Vec::with_capacity(pts.len().saturating_sub(1) * 6);
    for i in 0..pts.len().saturating_sub(1) {
        let (a, b, c, d) = ((i * 2) as u32, (i * 2 + 1) as u32, (i * 2 + 2) as u32, (i * 2 + 3) as u32);
        indices.extend_from_slice(&[a, c, b, b, c, d]);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}
