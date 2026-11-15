//! Terrain: turn a `mars-nav` DEM into a Bevy mesh and own the grid↔world mapping
//! that the route and navigation overlays share.

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;
use mars_nav::Dem;


/// The loaded terrain + its placement in world space. World units are meters; the
/// grid is centered on the origin, Y is up and vertically exaggerated by `vex`.
#[derive(Resource)]
pub struct Terrain {
    pub dem: Dem,
    pub vex: f32,
    pub span_x: f32,
    pub span_z: f32,
}

impl Terrain {
    pub fn new(dem: Dem, vex: f32) -> Self {
        let span_x = dem.cell_size_m * (dem.width as f32 - 1.0);
        let span_z = dem.cell_size_m * (dem.height as f32 - 1.0);
        Terrain { dem, vex, span_x, span_z }
    }

    /// Fractional grid cell → world position on the surface (+ optional lift).
    pub fn grid_to_world(&self, gx: f32, gy: f32, lift: f32) -> Vec3 {
        let x = (gx / (self.dem.width as f32 - 1.0) - 0.5) * self.span_x;
        let z = (gy / (self.dem.height as f32 - 1.0) - 0.5) * self.span_z;
        let y = (self.dem.sample(gx, gy) - self.dem.min) * self.vex + lift;
        Vec3::new(x, y, z)
    }

    /// World (x,z) → fractional grid cell.
    pub fn world_to_grid(&self, x: f32, z: f32) -> (f32, f32) {
        let gx = (x / self.span_x + 0.5) * (self.dem.width as f32 - 1.0);
        let gy = (z / self.span_z + 0.5) * (self.dem.height as f32 - 1.0);
        (gx, gy)
    }

    pub fn height_world(&self, gx: f32, gy: f32) -> f32 {
        (self.dem.sample(gx, gy) - self.dem.min) * self.vex
    }
}

/// Build a terrain mesh from the DEM. `color` supplies the vertex colour for each
/// cell, which is how the data layers (elevation, hazard, thermal) are drawn.
pub fn build_mesh_with(terrain: &Terrain, color: impl Fn(usize, usize) -> [f32; 4]) -> Mesh {
    let dem = &terrain.dem;
    let (w, h) = (dem.width, dem.height);
    let cell = dem.cell_size_m;
    let vex = terrain.vex;

    let mut positions = Vec::with_capacity(w * h);
    let mut normals = Vec::with_capacity(w * h);
    let mut uvs = Vec::with_capacity(w * h);
    let mut colors = Vec::with_capacity(w * h);

    for j in 0..h {
        for i in 0..w {
            let e = dem.data[j * w + i];
            let x = (i as f32 / (w as f32 - 1.0) - 0.5) * terrain.span_x;
            let z = (j as f32 / (h as f32 - 1.0) - 0.5) * terrain.span_z;
            let y = (e - dem.min) * vex;
            positions.push([x, y, z]);

            // normal from central differences of the exaggerated surface
            let il = i.saturating_sub(1);
            let ir = (i + 1).min(w - 1);
            let jt = j.saturating_sub(1);
            let jb = (h + 1).min(j + 1).min(h - 1);
            let dydx = (dem.data[j * w + ir] - dem.data[j * w + il]) * vex / (2.0 * cell);
            let dydz = (dem.data[jb * w + i] - dem.data[jt * w + i]) * vex / (2.0 * cell);
            let n = Vec3::new(-dydx, 1.0, -dydz).normalize();
            normals.push([n.x, n.y, n.z]);

            uvs.push([i as f32 / (w as f32 - 1.0), j as f32 / (h as f32 - 1.0)]);
            colors.push(color(i, j));
        }
    }

    let mut indices = Vec::with_capacity((w - 1) * (h - 1) * 6);
    for j in 0..h - 1 {
        for i in 0..w - 1 {
            let a = (j * w + i) as u32;
            let b = (j * w + i + 1) as u32;
            let c = ((j + 1) * w + i) as u32;
            let d = ((j + 1) * w + i + 1) as u32;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}
