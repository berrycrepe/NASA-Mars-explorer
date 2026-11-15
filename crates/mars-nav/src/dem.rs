//! Digital Elevation Model: a regular grid of elevations with a geographic bounding box.
//!
//! The grid is row-major, origin at the **north-west** corner (row 0 = north edge),
//! matching the convention of GeoTIFF rasters produced by the data-prep CLI.

/// A rectangular elevation grid tied to a lon/lat bounding box.
#[derive(Clone, Debug)]
pub struct Dem {
    pub width: usize,
    pub height: usize,
    /// Row-major elevations in meters (datum per `manifest`). Length == width*height.
    pub data: Vec<f32>,
    pub min: f32,
    pub max: f32,
    /// Geographic bounds in degrees (planetocentric east longitude, latitude).
    pub west: f64,
    pub east: f64,
    pub south: f64,
    pub north: f64,
    /// Approximate ground sample distance, meters per cell.
    pub cell_size_m: f32,
}

impl Dem {
    pub fn new(
        width: usize,
        height: usize,
        data: Vec<f32>,
        (west, east, south, north): (f64, f64, f64, f64),
        cell_size_m: f32,
    ) -> Self {
        assert_eq!(data.len(), width * height, "DEM data length must equal width*height");
        let (mut min, mut max) = (f32::INFINITY, f32::NEG_INFINITY);
        for &v in &data {
            if v < min {
                min = v;
            }
            if v > max {
                max = v;
            }
        }
        Dem {
            width,
            height,
            data,
            min,
            max,
            west,
            east,
            south,
            north,
            cell_size_m,
        }
    }

    #[inline]
    pub fn index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }

    /// Elevation at an integer cell, clamped to the grid edges.
    #[inline]
    pub fn at(&self, x: isize, y: isize) -> f32 {
        let cx = x.clamp(0, self.width as isize - 1) as usize;
        let cy = y.clamp(0, self.height as isize - 1) as usize;
        self.data[self.index(cx, cy)]
    }

    /// Bilinearly sampled elevation at fractional grid coordinates.
    pub fn sample(&self, fx: f32, fy: f32) -> f32 {
        let x0 = fx.floor();
        let y0 = fy.floor();
        let tx = fx - x0;
        let ty = fy - y0;
        let x0 = x0 as isize;
        let y0 = y0 as isize;
        let a = self.at(x0, y0);
        let b = self.at(x0 + 1, y0);
        let c = self.at(x0, y0 + 1);
        let d = self.at(x0 + 1, y0 + 1);
        let top = a + (b - a) * tx;
        let bot = c + (d - c) * tx;
        top + (bot - top) * ty
    }

    /// Normalized elevation in [0,1] (min→0, max→1).
    #[inline]
    pub fn normalized(&self, x: usize, y: usize) -> f32 {
        let range = self.max - self.min;
        if range <= f32::EPSILON {
            0.0
        } else {
            (self.data[self.index(x, y)] - self.min) / range
        }
    }

    /// Grid cell → (lon, lat) degrees at the cell center.
    pub fn grid_to_lonlat(&self, x: usize, y: usize) -> (f64, f64) {
        let u = if self.width > 1 {
            x as f64 / (self.width - 1) as f64
        } else {
            0.0
        };
        let v = if self.height > 1 {
            y as f64 / (self.height - 1) as f64
        } else {
            0.0
        };
        let lon = self.west + u * (self.east - self.west);
        // row 0 is the north edge, so latitude decreases as y increases.
        let lat = self.north - v * (self.north - self.south);
        (lon, lat)
    }

    /// (lon, lat) degrees → fractional grid coordinates.
    pub fn lonlat_to_grid(&self, lon: f64, lat: f64) -> (f32, f32) {
        let u = (lon - self.west) / (self.east - self.west);
        let v = (self.north - lat) / (self.north - self.south);
        (
            (u * (self.width - 1) as f64) as f32,
            (v * (self.height - 1) as f64) as f32,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(w: usize, h: usize, v: f32) -> Dem {
        Dem::new(w, h, vec![v; w * h], (77.0, 78.0, 18.0, 19.0), 10.0)
    }

    #[test]
    fn minmax_computed() {
        let dem = Dem::new(2, 2, vec![-10.0, 5.0, 0.0, 20.0], (0.0, 1.0, 0.0, 1.0), 1.0);
        assert_eq!(dem.min, -10.0);
        assert_eq!(dem.max, 20.0);
    }

    #[test]
    fn bilinear_midpoint() {
        // corners 0,10,10,20 -> center should be mean = 10
        let dem = Dem::new(2, 2, vec![0.0, 10.0, 10.0, 20.0], (0.0, 1.0, 0.0, 1.0), 1.0);
        assert!((dem.sample(0.5, 0.5) - 10.0).abs() < 1e-5);
    }

    #[test]
    fn edge_clamping() {
        let dem = flat(4, 4, 3.0);
        assert_eq!(dem.at(-5, -5), 3.0);
        assert_eq!(dem.at(100, 100), 3.0);
    }

    #[test]
    fn lonlat_roundtrip() {
        let dem = flat(101, 101, 0.0);
        let (lon, lat) = dem.grid_to_lonlat(50, 50);
        let (gx, gy) = dem.lonlat_to_grid(lon, lat);
        assert!((gx - 50.0).abs() < 1e-3);
        assert!((gy - 50.0).abs() < 1e-3);
    }

    #[test]
    fn north_is_row_zero() {
        let dem = flat(10, 10, 0.0);
        let (_, lat_top) = dem.grid_to_lonlat(0, 0);
        let (_, lat_bot) = dem.grid_to_lonlat(0, 9);
        assert!(lat_top > lat_bot, "row 0 must be the northern edge");
    }
}
