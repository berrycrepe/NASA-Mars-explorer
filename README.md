# Mars Explorer

A **native desktop instrument** for Mars surface navigation. Explore real Martian
terrain freely, designate any point as a target, and the system plans the **safest
route** to it and navigates you there — **without GPS**, using inertial navigation
corrected by visual localization and fused with an Extended Kalman Filter.

**Pure Rust.** Bevy for rendering, `mars-nav` for the geospatial and navigation
maths. No web stack, no JavaScript, no Electron/Tauri.

## Why

Mars has no satellite navigation. The research question: *how can astronauts
determine their location and navigate safely without it?* This tool makes the
proposed answer operable — **INS** dead-reckons continuously, **visual
localization** corrects against orbital terrain maps (~0.25 m, after NASA's Mars
Global Localization), and an **EKF** fuses them into a reported position with an
honest uncertainty. The panel reports whether the last update came from a visual fix or from
inertial dead-reckoning, and how uncertain the position currently is.

## How it works

Nothing is pre-planned. There are no built-in routes.

1. A **base landmark** sits on the map — a single point, nothing attached to it.
2. **Click any point on the surface** to designate a target. The pin drops and the
   safest path is planned from wherever you are, over a slope-derived traversal
   cost surface (A*, then smoothed so it follows the ground naturally).
3. Press **G** to navigate it hands-free, or just **walk there yourself**. The
   route is guidance, never a rail.
4. The route is drawn as a flat ribbon **painted on the ground**, so it guides
   without hiding the terrain it crosses.

## Real NASA data

The terrain is **real**, cropped from public USGS/NASA products over Jezero Crater
(77.10–78.10 E, 17.90–18.90 N — about 58 km across):

| Layer | Product |
|---|---|
| Elevation | **MGS MOLA** 463 m global DEM — real relief, −2862 m floor to −1472 m rim |
| Imagery | **MRO CTX** Jezero mosaic (USGS human-exploration-zone mosaic) |
| Thermal | **Odyssey THEMIS** IR-day 100 m global mosaic |
| Danger zones | Derived: slope computed from the MOLA elevation |

Measured layers are labelled MEASURED in the panel; the environment block
(temperature, pressure, wind, radiation) is labelled MODELED.

## Controls

| Input | Action |
|---|---|
| **Click surface** | Designate a target and plan the safest route to it |
| **G** | Go / stop auto-navigation along the route |
| **X** | Clear the target and its route |
| **I** | Open / close the data panel |
| — | A **route panel** opens by itself whenever a path is planned |
| **1 / 2 / 3** | Explore (3D) · Map (overview) · Navigate (top-down follow, north-up) |
| **4 / 5 / 6 / 7** | Imagery · Elevation · Danger zones · Thermal IR |
| **WASD** | Move across the map in Navigate; **Shift** to move fast |
| Drag / scroll | Orbit and zoom (Explore/Map); scroll zooms the navigation view |
| **F12** | Save a screenshot |

The full key list is shown in the bottom-right corner of the app.

Launch options: `MARS_MODE=explore|map|navigate` and
`MARS_LAYER=imagery|elevation|danger|thermal` set the initial view;
`MARS_DEMO=1` designates a test target on startup.

## Route panel

Planning a path opens a panel showing an **elevation profile** along the route —
bar height is elevation, bar colour is the slope-hazard tier — plus distance,
remaining distance, straight-line distance, climb and drop, maximum slope, the
share of the route in each danger tier, and estimated walking time and radiation
dose at a 1.3 m/s suited pace.

A **white marker** is where you actually are; a **cyan marker and ring** is where
the GPS-free system *believes* you are, and how uncertain it is.

## Architecture

```
crates/
  mars-nav/       Pure Rust, no dependencies:
                  dem, slope, cost, route (A*), visibility, procedural terrain
                  nav/ekf.rs   3-state Extended Kalman Filter
                  nav/mod.rs   INS dead-reckoning + visual fixes + fusion
  mars-dataprep/  CLI: real NASA DEM (GeoTIFF/IMG) -> heightmap + manifest (GDAL)
app/              Bevy binary — the instrument:
  terrain.rs   DEM -> 3D mesh, grid<->world mapping
  pick.rs      ray-marched terrain picking (click -> surface point)
  route.rs     on-demand A* between any two points + path smoothing + ribbon
  mission.rs   astronaut, target, route state, navigation estimate
  camera.rs    free orbit/pan/zoom explorer camera
  ui.rs        instrument overlay + toggleable surface-data panel
  main.rs      wiring, view modes, cinematic first-person camera
```

The navigation maths is unit-tested (`cargo test -p mars-nav`): uncertainty grows
under dead-reckoning, shrinks on a visual fix, and the covariance stays
positive-definite.

## Build & run

```bash
cargo run -p mars-explorer
```

If `cargo` isn't found, Homebrew's rustup lives outside your PATH:

```bash
export PATH="/opt/homebrew/opt/rustup/bin:$PATH"
```

First build compiles Bevy (a few minutes, once). After that it's seconds.

## Regenerating the data bundle

The committed bundle lives in `app/assets/` (`jezero_dem.f32`, `jezero_ctx.png`,
`jezero_themis.f32`, `jezero.json`). It was cropped straight from the public USGS
S3 mirror with GDAL; see `data/PROVENANCE.md`. If the bundle is missing the app
falls back to a procedural Jezero stand-in and says so in the panel.

## Status

Demonstration software, not flight navigation. The terrain, routing, EKF fusion
and drift behaviour are real and runnable; the "true" pose the filter is corrected
toward is simulated ground truth that a real system never has. Accuracy figures
(e.g. MGL ~25 cm) come from the cited research and should be checked against
NASA's primary sources before being presented as guaranteed performance.
