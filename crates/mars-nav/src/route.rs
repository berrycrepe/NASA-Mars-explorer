//! Hazard-aware route planning over a cost surface (A*, 8-connected).
//!
//! Edge weight between neighbouring cells = mean of their traversal costs scaled
//! by the ground distance stepped (orthogonal vs diagonal). The heuristic is the
//! straight-line ground distance to the goal times the minimum cell cost, which
//! keeps A* admissible.

use crate::dem::Dem;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Debug)]
pub struct PathResult {
    /// Ordered grid cells from start to goal (inclusive).
    pub cells: Vec<(usize, usize)>,
    /// Ground length along the path, meters.
    pub length_m: f32,
    /// Total positive elevation gain along the path, meters.
    pub ascent_m: f32,
    /// Accumulated traversal cost (unitless).
    pub cost: f32,
}

#[derive(Copy, Clone)]
struct Node {
    f: f32,
    idx: usize,
}
impl PartialEq for Node {
    fn eq(&self, o: &Self) -> bool {
        self.f == o.f
    }
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        // Reverse for min-heap behaviour on a max-heap (BinaryHeap).
        o.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

const NEIGHBORS: [(isize, isize); 8] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
];

/// A* between two cells over `cost` (row-major, dims == dem). Returns `None` if
/// unreachable (shouldn't happen on a connected grid with finite costs).
pub fn a_star(
    dem: &Dem,
    cost: &[f32],
    start: (usize, usize),
    goal: (usize, usize),
) -> Option<PathResult> {
    let w = dem.width;
    let h = dem.height;
    let n = w * h;
    let cell = dem.cell_size_m.max(1e-3);
    let idx = |x: usize, y: usize| y * w + x;

    let min_cost = cost.iter().cloned().fold(f32::INFINITY, f32::min).max(1e-3);
    let goal_idx = idx(goal.0, goal.1);

    let heuristic = |i: usize| -> f32 {
        let (x, y) = (i % w, i / w);
        let dx = (x as f32 - goal.0 as f32).abs();
        let dy = (y as f32 - goal.1 as f32).abs();
        (dx * dx + dy * dy).sqrt() * cell * min_cost
    };

    let mut g = vec![f32::INFINITY; n];
    let mut came: Vec<i32> = vec![-1; n];
    let mut closed = vec![false; n];

    let start_idx = idx(start.0, start.1);
    g[start_idx] = 0.0;
    let mut open = BinaryHeap::new();
    open.push(Node {
        f: heuristic(start_idx),
        idx: start_idx,
    });

    while let Some(Node { idx: ci, .. }) = open.pop() {
        if ci == goal_idx {
            return Some(reconstruct(dem, cost, &came, start_idx, goal_idx));
        }
        if closed[ci] {
            continue;
        }
        closed[ci] = true;
        let (cx, cy) = (ci % w, ci / w);

        for (dx, dy) in NEIGHBORS {
            let nx = cx as isize + dx;
            let ny = cy as isize + dy;
            if nx < 0 || ny < 0 || nx >= w as isize || ny >= h as isize {
                continue;
            }
            let ni = idx(nx as usize, ny as usize);
            if closed[ni] {
                continue;
            }
            let step = if dx != 0 && dy != 0 {
                std::f32::consts::SQRT_2
            } else {
                1.0
            };
            let edge = 0.5 * (cost[ci] + cost[ni]) * step * cell;
            let tentative = g[ci] + edge;
            if tentative < g[ni] {
                g[ni] = tentative;
                came[ni] = ci as i32;
                open.push(Node {
                    f: tentative + heuristic(ni),
                    idx: ni,
                });
            }
        }
    }
    None
}

fn reconstruct(
    dem: &Dem,
    cost: &[f32],
    came: &[i32],
    start_idx: usize,
    goal_idx: usize,
) -> PathResult {
    let w = dem.width;
    let cell = dem.cell_size_m.max(1e-3);
    let mut cells = Vec::new();
    let mut cur = goal_idx as i32;
    while cur >= 0 {
        let u = cur as usize;
        cells.push((u % w, u / w));
        if u == start_idx {
            break;
        }
        cur = came[u];
    }
    cells.reverse();

    let mut length_m = 0.0;
    let mut ascent_m = 0.0;
    let mut cost_sum = 0.0;
    for pair in cells.windows(2) {
        let (ax, ay) = pair[0];
        let (bx, by) = pair[1];
        let step = if ax != bx && ay != by {
            std::f32::consts::SQRT_2
        } else {
            1.0
        };
        length_m += step * cell;
        let ea = dem.data[ay * w + ax];
        let eb = dem.data[by * w + bx];
        if eb > ea {
            ascent_m += eb - ea;
        }
        cost_sum += 0.5 * (cost[ay * w + ax] + cost[by * w + bx]) * step * cell;
    }

    PathResult {
        cells,
        length_m,
        ascent_m,
        cost: cost_sum,
    }
}

/// Chain A* through an ordered list of waypoint cells, concatenating segments.
pub fn multi_waypoint(dem: &Dem, cost: &[f32], waypoints: &[(usize, usize)]) -> Option<PathResult> {
    if waypoints.len() < 2 {
        return None;
    }
    let mut all: Vec<(usize, usize)> = Vec::new();
    let mut length_m = 0.0;
    let mut ascent_m = 0.0;
    let mut cost_sum = 0.0;
    for seg in waypoints.windows(2) {
        let pr = a_star(dem, cost, seg[0], seg[1])?;
        if all.is_empty() {
            all.extend(pr.cells.iter().cloned());
        } else {
            // skip duplicated junction cell
            all.extend(pr.cells.iter().skip(1).cloned());
        }
        length_m += pr.length_m;
        ascent_m += pr.ascent_m;
        cost_sum += pr.cost;
    }
    Some(PathResult {
        cells: all,
        length_m,
        ascent_m,
        cost: cost_sum,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Dem;

    fn flat_dem(w: usize, h: usize) -> Dem {
        Dem::new(w, h, vec![0.0; w * h], (0.0, 1.0, 0.0, 1.0), 10.0)
    }

    #[test]
    fn straight_line_on_uniform_cost() {
        let dem = flat_dem(10, 10);
        let cost = vec![1.0f32; 100];
        let pr = a_star(&dem, &cost, (0, 0), (9, 0)).unwrap();
        assert_eq!(pr.cells.first().unwrap(), &(0, 0));
        assert_eq!(pr.cells.last().unwrap(), &(9, 0));
        // 9 orthogonal steps * 10 m
        assert!((pr.length_m - 90.0).abs() < 1e-3, "len {}", pr.length_m);
    }

    #[test]
    fn routes_around_a_wall() {
        // Vertical high-cost wall at x=5 spanning y=0..8, with a gap at y=9.
        let dem = flat_dem(10, 10);
        let mut cost = vec![1.0f32; 100];
        for y in 0..9 {
            cost[y * 10 + 5] = 1000.0;
        }
        let pr = a_star(&dem, &cost, (0, 0), (9, 0)).unwrap();
        // The path must dip down to the gap row (y == 9) to get through cheaply.
        let max_y = pr.cells.iter().map(|&(_, y)| y).max().unwrap();
        assert!(max_y >= 8, "expected detour toward the gap, max_y={max_y}");
    }

    #[test]
    fn multi_waypoint_chains() {
        let dem = flat_dem(10, 10);
        let cost = vec![1.0f32; 100];
        let pr = multi_waypoint(&dem, &cost, &[(0, 0), (9, 0), (9, 9)]).unwrap();
        assert_eq!(pr.cells.first().unwrap(), &(0, 0));
        assert_eq!(pr.cells.last().unwrap(), &(9, 9));
    }
}
