//! Load the real NASA elevation bundle (raw little-endian f32 grid + manifest),
//! falling back to the procedural stand-in when the assets are absent.

use mars_nav::Dem;
use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize)]
pub struct Manifest {
    pub width: usize,
    pub height: usize,
    pub west: f64,
    pub east: f64,
    pub south: f64,
    pub north: f64,
    pub cell_size_m: f32,
    pub source: String,
    /// Optional surface imagery file (relative to the Bevy assets folder).
    #[serde(default)]
    pub imagery: Option<String>,
    #[serde(default)]
    pub thermal: Option<String>,
}

pub struct Loaded {
    pub dem: Dem,
    pub source: String,
    pub imagery: Option<String>,
    pub thermal: Option<String>,
    /// True when real measured data was loaded (false = procedural stand-in).
    pub real: bool,
}

pub fn load(dir: &Path) -> Loaded {
    match try_load(dir) {
        Some(l) => l,
        None => Loaded {
            dem: mars_nav::procedural::jezero_synthetic(384, 384),
            source: "Procedural Jezero stand-in (no real bundle found)".into(),
            imagery: None,
            thermal: None,
            real: false,
        },
    }
}

fn try_load(dir: &Path) -> Option<Loaded> {
    let text = std::fs::read_to_string(dir.join("jezero.json")).ok()?;
    let m: Manifest = serde_json::from_str(&text).ok()?;
    let bytes = std::fs::read(dir.join("jezero_dem.f32")).ok()?;
    if bytes.len() != m.width * m.height * 4 {
        eprintln!(
            "real DEM size mismatch: {} bytes for {}x{}",
            bytes.len(),
            m.width,
            m.height
        );
        return None;
    }

    let mut data: Vec<f32> = bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect();

    // MOLA carries a large-negative nodata; replace it so slope/routing stay sane.
    let valid: Vec<f32> = data
        .iter()
        .copied()
        .filter(|v| v.is_finite() && *v > -20_000.0)
        .collect();
    if valid.is_empty() {
        return None;
    }
    let mean = valid.iter().sum::<f32>() / valid.len() as f32;
    for v in data.iter_mut() {
        if !v.is_finite() || *v <= -20_000.0 {
            *v = mean;
        }
    }

    let dem = Dem::new(
        m.width,
        m.height,
        data,
        (m.west, m.east, m.south, m.north),
        m.cell_size_m,
    );
    Some(Loaded {
        dem,
        source: m.source,
        imagery: m.imagery,
        thermal: m.thermal,
        real: true,
    })
}
