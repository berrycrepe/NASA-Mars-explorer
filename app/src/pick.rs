//! Terrain ray-picking: turn a screen click into a point on the Martian surface.
//!
//! Marches the ray against the DEM heightfield (no physics/mesh-picking needed),
//! then binary-searches the crossing for a precise surface hit.

use bevy::prelude::*;

use crate::terrain::Terrain;

/// First surface intersection of `origin + dir * t` with the terrain, if any.
pub fn ray_terrain(terrain: &Terrain, origin: Vec3, dir: Vec3, max_dist: f32) -> Option<Vec3> {
    let dir = dir.normalize_or_zero();
    if dir == Vec3::ZERO {
        return None;
    }
    let half_x = terrain.span_x * 0.5;
    let half_z = terrain.span_z * 0.5;
    let base_step = (terrain.dem.cell_size_m * 0.5).max(1.0);

    let height_at = |p: Vec3| -> f32 {
        let (gx, gy) = terrain.world_to_grid(p.x, p.z);
        terrain.height_world(gx, gy)
    };

    let mut t = 0.0f32;
    let mut last_outside_t = 0.0f32;
    let mut have_above = false;

    while t < max_dist {
        let p = origin + dir * t;
        let inside = p.x.abs() <= half_x && p.z.abs() <= half_z;

        if !inside {
            // Skip ahead quickly while outside the terrain footprint.
            last_outside_t = t;
            t += base_step * 8.0;
            have_above = false;
            continue;
        }

        let diff = p.y - height_at(p);
        if diff <= 0.0 {
            // Crossed the surface: refine between the last above-ground sample and here.
            let mut lo = if have_above { (t - base_step).max(last_outside_t) } else { last_outside_t };
            let mut hi = t;
            for _ in 0..28 {
                let mid = 0.5 * (lo + hi);
                let pm = origin + dir * mid;
                if pm.y - height_at(pm) > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let hit = origin + dir * hi;
            return Some(Vec3::new(hit.x, height_at(hit), hit.z));
        }

        have_above = true;
        // Adaptive: step proportional to height above ground, bounded.
        t += (diff * 0.7).clamp(base_step, base_step * 24.0);
    }
    None
}

/// Convert the cursor position into a terrain point for the active camera.
pub fn cursor_to_terrain(
    terrain: &Terrain,
    camera: &Camera,
    cam_tf: &GlobalTransform,
    cursor: Vec2,
) -> Option<Vec3> {
    let ray = camera.viewport_to_world(cam_tf, cursor)?;
    let max = terrain.span_x * 4.0;
    ray_terrain(terrain, ray.origin, *ray.direction, max)
}
