//! mars-dataprep — produce the runtime asset bundle the web app loads.
//!
//! Two subcommands:
//!   synthetic <width> <height> <out_dir>
//!       Write a procedural Jezero bundle (height grid + manifest). No GDAL,
//!       always available — this is what ships committed so the app runs offline.
//!
//!   convert --dem <geotiff> [--bbox W,E,S,N] --size <N> --out <dir>
//!       (requires `--features real-data`) Reproject/crop/resample a real NASA DEM
//!       GeoTIFF into the same bundle format. See data/PROVENANCE.md for sources.
//!
//! Bundle = `<out>/jezero.height.f32` (row-major little-endian f32, north row
//! first) + `<out>/jezero.manifest.json`. The web app builds every layer texture
//! from the height grid via WASM, so no image files are required at runtime.

use mars_nav::{procedural, Dem};
use serde::Serialize;
use std::fs;
use std::io::Write;
use std::path::Path;

#[derive(Serialize)]
struct Bbox {
    west: f64,
    east: f64,
    south: f64,
    north: f64,
}

#[derive(Serialize)]
struct Waypoint {
    name: String,
    lon: f64,
    lat: f64,
}

#[derive(Serialize)]
struct Manifest {
    site: String,
    datum: String,
    width: usize,
    height: usize,
    bbox: Bbox,
    #[serde(rename = "cellSizeM")]
    cell_size_m: f32,
    #[serde(rename = "elevMinM")]
    elev_min_m: f32,
    #[serde(rename = "elevMaxM")]
    elev_max_m: f32,
    #[serde(rename = "heightFile")]
    height_file: String,
    #[serde(rename = "heightFormat")]
    height_format: String,
    source: String,
    credits: Vec<String>,
    waypoints: Vec<Waypoint>,
}

fn write_bundle(dem: &Dem, out_dir: &Path, source: &str) -> std::io::Result<()> {
    fs::create_dir_all(out_dir)?;
    let height_file = "jezero.height.f32";

    // Raw little-endian f32 grid.
    let mut buf = Vec::with_capacity(dem.data.len() * 4);
    for &v in &dem.data {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    fs::File::create(out_dir.join(height_file))?.write_all(&buf)?;

    let waypoints = procedural::seed_waypoints()
        .into_iter()
        .map(|(lon, lat, name)| Waypoint {
            name: name.to_string(),
            lon,
            lat,
        })
        .collect();

    let manifest = Manifest {
        site: "Jezero Crater — Western Delta".to_string(),
        datum: "Mars 2000 / MOLA areoid (meters)".to_string(),
        width: dem.width,
        height: dem.height,
        bbox: Bbox {
            west: dem.west,
            east: dem.east,
            south: dem.south,
            north: dem.north,
        },
        cell_size_m: dem.cell_size_m,
        elev_min_m: dem.min,
        elev_max_m: dem.max,
        height_file: height_file.to_string(),
        height_format: "f32le".to_string(),
        source: source.to_string(),
        credits: vec![
            "NASA/JPL-Caltech".to_string(),
            "USGS Astrogeology Science Center".to_string(),
            "MRO HiRISE / CTX (LPL, U. Arizona; Murray Lab, Caltech)".to_string(),
            "MGS MOLA; ESA/DLR/FU Berlin HRSC".to_string(),
            "MSL/RAD surface dose reference".to_string(),
        ],
        waypoints,
    };
    let json = serde_json::to_string_pretty(&manifest).expect("serialize manifest");
    fs::File::create(out_dir.join("jezero.manifest.json"))?.write_all(json.as_bytes())?;

    println!(
        "Wrote bundle to {} ({}×{}, {:.1} m/cell, elev {:.0}..{:.0} m)",
        out_dir.display(),
        dem.width,
        dem.height,
        dem.cell_size_m,
        dem.min,
        dem.max
    );
    Ok(())
}

fn cmd_synthetic(args: &[String]) -> Result<(), String> {
    let width: usize = args.first().and_then(|s| s.parse().ok()).unwrap_or(512);
    let height: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(512);
    let out = args
        .get(2)
        .cloned()
        .unwrap_or_else(|| "web/public/assets".to_string());
    let dem = procedural::jezero_synthetic(width, height);
    write_bundle(
        &dem,
        Path::new(&out),
        "synthetic (procedural) — placeholder for MOLA/HRSC/HiRISE DEM",
    )
    .map_err(|e| e.to_string())
}

fn usage() {
    eprintln!(
        "mars-dataprep\n\
         \n\
         USAGE:\n\
         \tmars-dataprep synthetic [width] [height] [out_dir]\n\
         \tmars-dataprep convert --dem <geotiff> [--bbox W,E,S,N] --size <N> --out <dir>   (needs --features real-data)\n\
         \n\
         Default out_dir is web/public/assets."
    );
}

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let result = match argv.first().map(String::as_str) {
        Some("synthetic") => cmd_synthetic(&argv[1..]),
        Some("convert") => cmd_convert(&argv[1..]),
        _ => {
            usage();
            std::process::exit(2);
        }
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

// ----------------------------------------------------------------------------
// Real-data path (GDAL) — compiled only with `--features real-data`.
// ----------------------------------------------------------------------------
#[cfg(feature = "real-data")]
fn cmd_convert(args: &[String]) -> Result<(), String> {
    use gdal::Dataset;

    let mut dem_path: Option<String> = None;
    let mut out = "web/public/assets".to_string();
    let mut size: usize = 512;
    let mut bbox: Option<(f64, f64, f64, f64)> = None;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--dem" => {
                dem_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--out" => {
                out = args.get(i + 1).cloned().ok_or("missing --out value")?;
                i += 2;
            }
            "--size" => {
                size = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .ok_or("bad --size")?;
                i += 2;
            }
            "--bbox" => {
                let parts: Vec<f64> = args
                    .get(i + 1)
                    .ok_or("missing --bbox")?
                    .split(',')
                    .filter_map(|s| s.parse().ok())
                    .collect();
                if parts.len() != 4 {
                    return Err("--bbox must be W,E,S,N".into());
                }
                bbox = Some((parts[0], parts[1], parts[2], parts[3]));
                i += 2;
            }
            other => return Err(format!("unknown arg {other}")),
        }
    }

    let dem_path = dem_path.ok_or("--dem is required")?;
    let ds = Dataset::open(Path::new(&dem_path)).map_err(|e| format!("open DEM: {e}"))?;
    let band = ds.rasterband(1).map_err(|e| e.to_string())?;
    let (rx, ry) = ds.raster_size();
    let gt = ds.geo_transform().map_err(|e| e.to_string())?;

    // Read the whole band, then resample into a `size`×`size` grid over the bbox.
    let buf = band
        .read_as::<f32>((0, 0), (rx, ry), (rx, ry), None)
        .map_err(|e| e.to_string())?;
    let src = buf.data();

    // Pixel → lon/lat using the affine geo-transform.
    let (west, east, south, north) = bbox.unwrap_or_else(|| {
        let lon0 = gt[0];
        let lat0 = gt[3];
        let lon1 = gt[0] + gt[1] * rx as f64;
        let lat1 = gt[3] + gt[5] * ry as f64;
        (lon0.min(lon1), lon0.max(lon1), lat1.min(lat0), lat1.max(lat0))
    });

    let mut data = vec![0.0f32; size * size];
    for j in 0..size {
        let lat = north - (j as f64 / (size - 1) as f64) * (north - south);
        for k in 0..size {
            let lon = west + (k as f64 / (size - 1) as f64) * (east - west);
            // invert affine (no rotation assumed)
            let px = ((lon - gt[0]) / gt[1]).round() as isize;
            let py = ((lat - gt[3]) / gt[5]).round() as isize;
            let v = if px >= 0 && py >= 0 && (px as usize) < rx && (py as usize) < ry {
                src[py as usize * rx + px as usize]
            } else {
                f32::NAN
            };
            data[j * size + k] = v;
        }
    }

    // Fill any NaNs (nodata) with the grid mean so downstream math stays finite.
    let finite: Vec<f32> = data.iter().cloned().filter(|v| v.is_finite()).collect();
    let mean = if finite.is_empty() {
        0.0
    } else {
        finite.iter().sum::<f32>() / finite.len() as f32
    };
    for v in data.iter_mut() {
        if !v.is_finite() {
            *v = mean;
        }
    }

    let mars_radius = 3_389_500.0f64;
    let mid_lat = ((north + south) * 0.5).to_radians();
    let deg_lon_m = mars_radius * mid_lat.cos() * std::f64::consts::PI / 180.0;
    let cell = ((east - west) * deg_lon_m / size as f64) as f32;

    let dem = Dem::new(size, size, data, (west, east, south, north), cell);
    write_bundle(
        &dem,
        Path::new(&out),
        &format!("real NASA DEM via GDAL: {dem_path}"),
    )
    .map_err(|e| e.to_string())
}

#[cfg(not(feature = "real-data"))]
fn cmd_convert(_args: &[String]) -> Result<(), String> {
    Err("`convert` requires building with `--features real-data` (needs libgdal). \
         Use `synthetic` for the offline bundle."
        .to_string())
}
