//! Mars Explorer — a native surface-navigation instrument for Mars astronauts.
//!
//! Real NASA data (MOLA elevation, CTX imagery, THEMIS thermal), free
//! exploration, click-to-designate targets with on-demand safest-route planning,
//! a top-down navigation view, and GPS-free positioning (INS + visual
//! localization fused by an EKF). Pure Rust: Bevy + `mars-nav`.

mod camera;
mod env;
mod mission;
mod pick;
mod realdem;
mod route;
mod terrain;
mod ui;

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy::render::view::screenshot::ScreenshotManager;
use bevy::window::{PrimaryWindow, Window};
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::path::Path;

use camera::{orbit_camera_system, CameraActive, OrbitCamera};
use env::Env;
use mission::{
    Astronaut, AstronautMarker, BaseMarker, EstMarker, Mission, NavState, RouteRibbon,
    TargetMarker, UncertaintyRing, REAL_WALK_SPEED,
};
use route::{build_ribbon_mesh, plan_between, NavGrid};
use terrain::{build_mesh_with, Terrain};
use ui::UiState;

const SKY: Color = Color::srgb(0.72, 0.52, 0.39);
/// Movement is accelerated relative to a real suited pace so a 58 km map is
/// usable; the navigation filter still integrates at a true walking pace.
const WALK_SPEED: f32 = 120.0;
const SPRINT_MULT: f32 = 5.0;
const FOLLOW_SPEED: f32 = 320.0;

/// Base landmark — a standalone point, nothing attached to it.
const BASE_LON: f64 = 77.4610;
const BASE_LAT: f64 = 18.4389;

#[derive(Resource, Clone, Copy, PartialEq, Eq)]
pub enum AppMode {
    Explore,
    Map,
    Navigate,
}

#[derive(Resource, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceLayer {
    Imagery,
    Elevation,
    Hazard,
    Thermal,
}

impl SurfaceLayer {
    pub fn label(&self) -> &'static str {
        match self {
            SurfaceLayer::Imagery => "CTX IMAGERY",
            SurfaceLayer::Elevation => "ELEVATION",
            SurfaceLayer::Hazard => "DANGER ZONES",
            SurfaceLayer::Thermal => "THERMAL IR",
        }
    }
    pub fn legend(&self) -> &'static str {
        match self {
            SurfaceLayer::Imagery => "MRO CTX  measured",
            SurfaceLayer::Elevation => "low -> high   MOLA",
            SurfaceLayer::Hazard => "green ok  amber caution  red avoid",
            SurfaceLayer::Thermal => "cold -> warm   THEMIS IR",
        }
    }
}

/// Real THEMIS thermal grid, when the bundle supplied one.
#[derive(Resource)]
pub struct ThermalData {
    pub values: Option<Vec<f32>>,
    pub min: f32,
    pub max: f32,
}

#[derive(Resource)]
pub struct DataSource {
    pub text: String,
    pub real: bool,
}

#[derive(Resource)]
struct TerrainMaterials {
    imagery: Handle<StandardMaterial>,
    plain: Handle<StandardMaterial>,
}

/// Camera altitude for the top-down navigation view.
#[derive(Resource)]
struct NavZoom(f32);

#[derive(Component)]
struct TerrainMesh;
#[derive(Component)]
struct MainCamera;

#[derive(Resource, Default)]
struct DragState {
    down: bool,
    moved: f32,
}

#[derive(Resource, Default)]
struct CamSnap(bool);

/// Optional launch state: MARS_MODE=explore|map|navigate, MARS_LAYER=imagery|elevation|danger|thermal
fn initial_state() -> (AppMode, SurfaceLayer) {
    let mode = match std::env::var("MARS_MODE").as_deref() {
        Ok("map") => AppMode::Map,
        Ok("navigate") => AppMode::Navigate,
        _ => AppMode::Explore,
    };
    let layer = match std::env::var("MARS_LAYER").as_deref() {
        Ok("elevation") => SurfaceLayer::Elevation,
        Ok("danger") => SurfaceLayer::Hazard,
        Ok("thermal") => SurfaceLayer::Thermal,
        _ => SurfaceLayer::Imagery,
    };
    (mode, layer)
}

fn main() {
    let (mode0, layer0) = initial_state();
    App::new()
        .insert_resource(ClearColor(SKY))
        .insert_resource(AmbientLight {
            color: Color::srgb(0.86, 0.72, 0.60),
            brightness: 260.0,
        })
        .insert_resource(CameraActive { orbit: mode0 != AppMode::Navigate })
        .insert_resource(mode0)
        .insert_resource(layer0)
        .insert_resource(DragState::default())
        .insert_resource(CamSnap(true))
        .insert_resource(UiState::default())
        .insert_resource(Env::default())
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Mars Explorer".into(),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                input_system,
                orbit_camera_system,
                pick_system,
                ribbon_width_system,
                demo_designate_system,
                layer_system,
                env_system,
                (move_system, nav_step_system, markers_system, nav_camera_system).chain(),
                ui::update_ui,
                ui::toggle_info_panel,
                ui::rebuild_route_chart,
                screenshot_system,
            ),
        )
        .run();
}

fn unlit(materials: &mut Assets<StandardMaterial>, color: Color) -> Handle<StandardMaterial> {
    materials.add(StandardMaterial { base_color: color, unlit: true, ..default() })
}

fn ramp(stops: &[[f32; 3]], t: f32) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let n = stops.len() - 1;
    let seg = t * n as f32;
    let i = (seg.floor() as usize).min(n - 1);
    let f = seg - i as f32;
    let (a, b) = (stops[i], stops[i + 1]);
    [
        a[0] + (b[0] - a[0]) * f,
        a[1] + (b[1] - a[1]) * f,
        a[2] + (b[2] - a[2]) * f,
        1.0,
    ]
}

const ELEV_RAMP: [[f32; 3]; 6] = [
    [0.09, 0.13, 0.42],
    [0.08, 0.42, 0.55],
    [0.22, 0.58, 0.30],
    [0.88, 0.78, 0.30],
    [0.82, 0.40, 0.18],
    [0.97, 0.94, 0.92],
];
const THERMAL_RAMP: [[f32; 3]; 5] = [
    [0.05, 0.09, 0.34],
    [0.44, 0.14, 0.52],
    [0.86, 0.24, 0.20],
    [0.99, 0.62, 0.14],
    [1.0, 0.96, 0.74],
];

/// Vertex colour for a cell under the active layer.
fn cell_color(
    layer: SurfaceLayer,
    terr: &Terrain,
    grid: &NavGrid,
    thermal: &ThermalData,
    i: usize,
    j: usize,
) -> [f32; 4] {
    let w = terr.dem.width;
    match layer {
        // Warm tint multiplied over the real greyscale CTX imagery.
        SurfaceLayer::Imagery => [0.90, 0.66, 0.47, 1.0],
        SurfaceLayer::Elevation => {
            let range = (terr.dem.max - terr.dem.min).max(1e-3);
            ramp(&ELEV_RAMP, (terr.dem.data[j * w + i] - terr.dem.min) / range)
        }
        SurfaceLayer::Hazard => {
            let deg = grid.slope.deg_at(i, j);
            if deg >= 20.0 {
                [0.90, 0.22, 0.17, 1.0]
            } else if deg >= 10.0 {
                [0.96, 0.72, 0.18, 1.0]
            } else {
                [0.20, 0.72, 0.42, 1.0]
            }
        }
        SurfaceLayer::Thermal => match &thermal.values {
            Some(v) => {
                let range = (thermal.max - thermal.min).max(1e-3);
                ramp(&THERMAL_RAMP, (v[j * w + i] - thermal.min) / range)
            }
            None => {
                // Modeled fallback: rocky/steep ground holds heat, smooth dust doesn't.
                let s = (grid.slope.deg_at(i, j) / grid.slope.max_deg.max(1.0)).clamp(0.0, 1.0);
                ramp(&THERMAL_RAMP, 0.25 + 0.6 * s)
            }
        },
    }
}

fn build_terrain_mesh(
    layer: SurfaceLayer,
    terr: &Terrain,
    grid: &NavGrid,
    thermal: &ThermalData,
) -> Mesh {
    build_mesh_with(terr, |i, j| cell_color(layer, terr, grid, thermal, i, j))
}

/// A thin pin: post + head, used for the base and target landmarks.
fn spawn_pin(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    pos: Vec3,
    scale: f32,
    color: Color,
    visible: bool,
    tag: impl Component,
) {
    let mat = unlit(materials, color);
    commands
        .spawn((
            SpatialBundle {
                transform: Transform::from_translation(pos),
                visibility: if visible { Visibility::Inherited } else { Visibility::Hidden },
                ..default()
            },
            tag,
        ))
        .with_children(|p| {
            p.spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(scale * 0.10, scale * 2.2)),
                material: mat.clone(),
                transform: Transform::from_xyz(0.0, scale * 1.1, 0.0),
                ..default()
            });
            p.spawn(PbrBundle {
                mesh: meshes.add(Sphere::new(scale * 0.40)),
                material: mat,
                transform: Transform::from_xyz(0.0, scale * 2.4, 0.0),
                ..default()
            });
        });
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // Real NASA bundle if present, otherwise the procedural stand-in. Bevy resolves
    // asset paths against the package directory, so anchor our own reads there too.
    let asset_dir = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/assets"));
    let loaded = realdem::load(asset_dir);
    let thermal_values = loaded.thermal.as_ref().and_then(|f| {
        let bytes = std::fs::read(asset_dir.join(f)).ok()?;
        let n = loaded.dem.width * loaded.dem.height;
        if bytes.len() != n * 4 {
            return None;
        }
        Some(
            bytes
                .chunks_exact(4)
                .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect::<Vec<f32>>(),
        )
    });
    let (tmin, tmax) = match &thermal_values {
        Some(v) => v.iter().fold((f32::MAX, f32::MIN), |(lo, hi), &x| {
            if x.is_finite() { (lo.min(x), hi.max(x)) } else { (lo, hi) }
        }),
        None => (0.0, 1.0),
    };
    let thermal = ThermalData { values: thermal_values, min: tmin, max: tmax };

    let terr = Terrain::new(loaded.dem, 3.0);
    let span = terr.span_x;
    let elev_range = (terr.dem.max - terr.dem.min) * terr.vex;
    let grid = NavGrid::build(&terr);

    // Two materials: real imagery, and a plain one for the data layers.
    let imagery_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        base_color_texture: loaded
            .imagery
            .as_ref()
            .filter(|f| asset_dir.join(f).exists())
            .map(|f| asset_server.load(f.clone())),
        perceptual_roughness: 0.95,
        metallic: 0.0,
        ..default()
    });
    let plain_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(build_terrain_mesh(SurfaceLayer::Imagery, &terr, &grid, &thermal)),
            material: imagery_mat.clone(),
            ..default()
        },
        TerrainMesh,
    ));

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::srgb(1.0, 0.94, 0.84),
            illuminance: 18_000.0,
            shadows_enabled: false,
            ..default()
        },
        transform: Transform::from_xyz(-1.0, 1.1, -0.5).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    let (bgx, bgy) = terr.dem.lonlat_to_grid(BASE_LON, BASE_LAT);
    let base_pos = terr.grid_to_world(bgx, bgy, 0.0);
    let pin = span * 0.0075;
    spawn_pin(&mut commands, &mut meshes, &mut materials, base_pos, pin, Color::srgb(0.35, 0.80, 1.0), true, BaseMarker);
    spawn_pin(&mut commands, &mut meshes, &mut materials, base_pos, pin, Color::srgb(1.0, 0.72, 0.25), false, TargetMarker);

    // Astronaut shown as a heading arrow (a cone laid flat).
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cone { radius: span * 0.0035, height: span * 0.010 }),
            material: unlit(&mut materials, Color::srgb(1.0, 1.0, 1.0)),
            transform: Transform::from_translation(base_pos),
            ..default()
        },
        AstronautMarker,
    ));
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Sphere::new(span * 0.0024)),
            material: unlit(&mut materials, Color::srgb(0.35, 0.80, 1.0)),
            transform: Transform::from_translation(base_pos),
            ..default()
        },
        EstMarker,
    ));
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Torus { minor_radius: 0.02, major_radius: 1.0 }),
            material: unlit(&mut materials, Color::srgba(0.35, 0.80, 1.0, 0.9)),
            transform: Transform::from_translation(base_pos),
            ..default()
        },
        UncertaintyRing,
    ));

    let orbit = OrbitCamera {
        focus: Vec3::new(0.0, elev_range * 0.3, 0.0),
        yaw: 0.6,
        pitch: -0.62,
        dist: span * 0.85,
    };
    let mut cam_tf = Transform::default();
    orbit.apply(&mut cam_tf);
    commands.spawn((
        Camera3dBundle { transform: cam_tf, ..default() },
        FogSettings {
            color: SKY,
            falloff: FogFalloff::Linear { start: span * 0.8, end: span * 2.6 },
            ..default()
        },
        orbit,
        MainCamera,
    ));

    commands.insert_resource(NavZoom(span * 0.26));
    commands.insert_resource(TerrainMaterials { imagery: imagery_mat, plain: plain_mat });
    commands.insert_resource(DataSource { text: loaded.source.clone(), real: loaded.real });
    commands.insert_resource(thermal);
    commands.insert_resource(grid);
    commands.insert_resource(Astronaut::new(base_pos));
    commands.insert_resource(NavState::new(base_pos, 0.0));
    commands.insert_resource(Mission::new(base_pos));
    commands.insert_resource(terr);
    ui::setup_ui(&mut commands);
}

fn lerp_angle(a: f32, b: f32, t: f32) -> f32 {
    let d = (b - a + PI).rem_euclid(TAU) - PI;
    a + d * t
}

fn ribbon_width(mode: &AppMode, terr: &Terrain) -> f32 {
    match mode {
        AppMode::Navigate => terr.span_x * 0.0022,
        _ => terr.span_x * 0.0035,
    }
}

fn ribbon_material(materials: &mut Assets<StandardMaterial>, _mode: &AppMode) -> Handle<StandardMaterial> {
    unlit(materials, Color::srgb(1.0, 0.76, 0.25))
}

/// Advance the surface clock (1 real second = 2 sol-minutes).
fn env_system(time: Res<Time>, mut env: ResMut<Env>) {
    env.advance(time.delta_seconds(), 2.0 / 60.0);
}

/// Rebuild the terrain colouring when the active data layer changes.
#[allow(clippy::too_many_arguments)]
fn layer_system(
    layer: Res<SurfaceLayer>,
    terr: Res<Terrain>,
    grid: Res<NavGrid>,
    thermal: Res<ThermalData>,
    mats: Res<TerrainMaterials>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&mut Handle<Mesh>, &mut Handle<StandardMaterial>), With<TerrainMesh>>,
) {
    if !layer.is_changed() {
        return;
    }
    for (mut mesh_h, mut mat_h) in &mut q {
        *mesh_h = meshes.add(build_terrain_mesh(*layer, &terr, &grid, &thermal));
        *mat_h = if *layer == SurfaceLayer::Imagery {
            mats.imagery.clone()
        } else {
            mats.plain.clone()
        };
    }
}

fn ribbon_width_system(
    mode: Res<AppMode>,
    mission: Res<Mission>,
    terr: Res<Terrain>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    ribbons: Query<Entity, With<RouteRibbon>>,
) {
    if !mode.is_changed() {
        return;
    }
    let Some(route) = mission.route.as_ref() else { return };
    for e in &ribbons {
        commands.entity(e).despawn_recursive();
    }
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(build_ribbon_mesh(&route.pts, ribbon_width(&mode, &terr))),
            material: ribbon_material(&mut materials, &mode),
            ..default()
        },
        RouteRibbon,
    ));
}

#[allow(clippy::too_many_arguments)]
fn input_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut mode: ResMut<AppMode>,
    mut layer: ResMut<SurfaceLayer>,
    mut active: ResMut<CameraActive>,
    mut mission: ResMut<Mission>,
    mut ui_state: ResMut<UiState>,
    mut snap: ResMut<CamSnap>,
    terr: Res<Terrain>,
    mut cam: Query<&mut OrbitCamera, With<MainCamera>>,
    mut commands: Commands,
    ribbons: Query<Entity, With<RouteRibbon>>,
    mut target_vis: Query<&mut Visibility, With<TargetMarker>>,
) {
    let span = terr.span_x;
    let elev_range = (terr.dem.max - terr.dem.min) * terr.vex;

    if keys.just_pressed(KeyCode::Digit1) {
        *mode = AppMode::Explore;
        active.orbit = true;
        if let Ok(mut o) = cam.get_single_mut() {
            o.focus = Vec3::new(0.0, elev_range * 0.3, 0.0);
            o.pitch = -0.62;
            o.dist = span * 0.85;
        }
    }
    if keys.just_pressed(KeyCode::Digit2) {
        *mode = AppMode::Map;
        active.orbit = true;
        if let Ok(mut o) = cam.get_single_mut() {
            o.focus = Vec3::ZERO;
            o.pitch = -1.52;
            o.dist = span * 0.95;
        }
    }
    if keys.just_pressed(KeyCode::Digit3) {
        *mode = AppMode::Navigate;
        active.orbit = false;
        snap.0 = true;
    }
    if keys.just_pressed(KeyCode::Digit4) { *layer = SurfaceLayer::Imagery; }
    if keys.just_pressed(KeyCode::Digit5) { *layer = SurfaceLayer::Elevation; }
    if keys.just_pressed(KeyCode::Digit6) { *layer = SurfaceLayer::Hazard; }
    if keys.just_pressed(KeyCode::Digit7) { *layer = SurfaceLayer::Thermal; }

    if keys.just_pressed(KeyCode::KeyG) && mission.route.is_some() {
        mission.following = !mission.following;
    }
    if keys.just_pressed(KeyCode::KeyX) {
        mission.clear();
        for e in &ribbons {
            commands.entity(e).despawn_recursive();
        }
        if let Ok(mut v) = target_vis.get_single_mut() {
            *v = Visibility::Hidden;
        }
    }
    if keys.just_pressed(KeyCode::KeyI) {
        ui_state.info_open = !ui_state.info_open;
    }
}

#[allow(clippy::too_many_arguments)]
fn pick_system(
    buttons: Res<ButtonInput<MouseButton>>,
    mut motion: EventReader<MouseMotion>,
    mut drag: ResMut<DragState>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cam_q: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
    terr: Res<Terrain>,
    grid: Res<NavGrid>,
    astro: Res<Astronaut>,
    mode: Res<AppMode>,
    mut mission: ResMut<Mission>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    ribbons: Query<Entity, With<RouteRibbon>>,
    mut target_q: Query<(&mut Transform, &mut Visibility), With<TargetMarker>>,
) {
    let delta: f32 = motion.read().map(|e| e.delta.length()).sum();
    if buttons.just_pressed(MouseButton::Left) {
        drag.down = true;
        drag.moved = 0.0;
    }
    if drag.down {
        drag.moved += delta;
    }
    if !buttons.just_released(MouseButton::Left) {
        return;
    }
    let was_click = drag.down && drag.moved < 6.0;
    drag.down = false;
    if !was_click {
        return;
    }

    let Ok(window) = windows.get_single() else { return };
    let Some(cursor) = window.cursor_position() else { return };
    let Ok((camera, cam_tf)) = cam_q.get_single() else { return };
    let Some(hit) = pick::cursor_to_terrain(&terr, camera, cam_tf, cursor) else { return };

    let w = ribbon_width(&mode, &terr);
    designate_target(
        hit, astro.pos, &terr, &grid, &mut mission, &mut commands, &mut meshes,
        &mut materials, &ribbons, &mut target_q, w, &mode,
    );
}

/// Designate a surface point: move the target pin, then plan and draw the safest
/// route to it from the astronaut's current position.
#[allow(clippy::too_many_arguments)]
fn designate_target(
    hit: Vec3,
    astro_pos: Vec3,
    terr: &Terrain,
    grid: &NavGrid,
    mission: &mut Mission,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    ribbons: &Query<Entity, With<RouteRibbon>>,
    target_q: &mut Query<(&mut Transform, &mut Visibility), With<TargetMarker>>,
    width: f32,
    mode: &AppMode,
) {
    mission.target = Some(hit);
    if let Ok((mut tf, mut vis)) = target_q.get_single_mut() {
        tf.translation = hit;
        *vis = Visibility::Inherited;
    }
    for e in ribbons.iter() {
        commands.entity(e).despawn_recursive();
    }
    mission.route = None;
    mission.following = false;
    if let Some(planned) = plan_between(terr, grid, astro_pos, hit) {
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(build_ribbon_mesh(&planned.pts, width)),
                material: ribbon_material(materials, mode),
                ..default()
            },
            RouteRibbon,
        ));
        mission.set_route(planned);
    }
}

/// Self-test (MARS_DEMO=1): designate a target across the crater so the route
/// panel can be checked without mouse input.
#[allow(clippy::too_many_arguments)]
fn demo_designate_system(
    time: Res<Time>,
    mut done: Local<bool>,
    terr: Res<Terrain>,
    grid: Res<NavGrid>,
    astro: Res<Astronaut>,
    mode: Res<AppMode>,
    mut mission: ResMut<Mission>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    ribbons: Query<Entity, With<RouteRibbon>>,
    mut target_q: Query<(&mut Transform, &mut Visibility), With<TargetMarker>>,
) {
    if *done || time.elapsed_seconds() < 1.0 || std::env::var("MARS_DEMO").is_err() {
        return;
    }
    *done = true;
    let tgt = clamp_to_terrain(
        &terr,
        astro.pos + Vec3::new(-terr.span_x * 0.24, 0.0, terr.span_z * 0.18),
    );
    let w = ribbon_width(&mode, &terr);
    designate_target(
        tgt, astro.pos, &terr, &grid, &mut mission, &mut commands, &mut meshes,
        &mut materials, &ribbons, &mut target_q, w, &mode,
    );
}

/// Free movement. In the navigation view WASD moves across the map in screen
/// directions (north-up), so nothing is constrained to the planned route.
#[allow(clippy::too_many_arguments)]
fn move_system(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<AppMode>,
    terr: Res<Terrain>,
    mut astro: ResMut<Astronaut>,
    mut mission: ResMut<Mission>,
) {
    let dt = time.delta_seconds().min(0.05);

    if *mode == AppMode::Navigate {
        // North is -Z, east is +X; the view is north-up so this matches the screen.
        let mut mv = Vec3::ZERO;
        if keys.pressed(KeyCode::KeyW) { mv.z -= 1.0; }
        if keys.pressed(KeyCode::KeyS) { mv.z += 1.0; }
        if keys.pressed(KeyCode::KeyA) { mv.x -= 1.0; }
        if keys.pressed(KeyCode::KeyD) { mv.x += 1.0; }
        if mv.length_squared() > 1e-6 {
            mission.following = false; // manual control overrides auto-navigation
            let speed = WALK_SPEED * if keys.pressed(KeyCode::ShiftLeft) { SPRINT_MULT } else { 1.0 };
            let step = mv.normalize() * speed * dt;
            let want = step.z.atan2(step.x);
            let k = 1.0 - (-10.0 * dt).exp();
            astro.heading = lerp_angle(astro.heading, want, k);
            astro.pos = clamp_to_terrain(&terr, astro.pos + step);
        }
    }

    if mission.following && mission.total > 0.0 {
        let adv = FOLLOW_SPEED * dt / mission.total;
        mission.progress = (mission.progress + adv).min(1.0);
        if let Some((pos, tan)) = mission.sample(mission.progress) {
            astro.pos = clamp_to_terrain(&terr, pos);
            let want = tan.z.atan2(tan.x);
            let k = 1.0 - (-5.0 * dt).exp();
            astro.heading = lerp_angle(astro.heading, want, k);
        }
        if mission.progress >= 1.0 {
            mission.following = false;
        }
    }
}

fn clamp_to_terrain(terr: &Terrain, p: Vec3) -> Vec3 {
    let hx = terr.span_x * 0.5 - terr.dem.cell_size_m;
    let hz = terr.span_z * 0.5 - terr.dem.cell_size_m;
    let x = p.x.clamp(-hx, hx);
    let z = p.z.clamp(-hz, hz);
    let (gx, gy) = terr.world_to_grid(x, z);
    Vec3::new(x, terr.height_world(gx, gy), z)
}

fn nav_step_system(
    time: Res<Time>,
    astro: Res<Astronaut>,
    mut nav: ResMut<NavState>,
    mut prev: Local<Option<Vec3>>,
) {
    let dt = time.delta_seconds().min(0.05) as f64;
    let last = prev.unwrap_or(astro.pos);
    let moved = astro.pos.distance(last) as f64;
    let dt_sim = if moved > 1e-4 { moved / REAL_WALK_SPEED } else { dt };
    nav.sim.step(astro.pos.x as f64, astro.pos.z as f64, astro.heading as f64, dt_sim, true);
    *prev = Some(astro.pos);
}

#[allow(clippy::type_complexity)]
fn markers_system(
    astro: Res<Astronaut>,
    nav: Res<NavState>,
    terr: Res<Terrain>,
    mut sets: ParamSet<(
        Query<&mut Transform, With<AstronautMarker>>,
        Query<&mut Transform, With<EstMarker>>,
        Query<&mut Transform, With<UncertaintyRing>>,
    )>,
) {
    let lift = terr.span_x * 0.002;
    if let Ok(mut t) = sets.p0().get_single_mut() {
        t.translation = astro.pos + Vec3::Y * lift;
        // Lay the cone flat and point it along the heading.
        t.rotation = Quat::from_rotation_y(FRAC_PI_2 - astro.heading) * Quat::from_rotation_x(FRAC_PI_2);
    }
    let (ex, ez, _h, radius) = nav.sim.reported();
    let (gx, gy) = terr.world_to_grid(ex as f32, ez as f32);
    let est = Vec3::new(ex as f32, terr.height_world(gx, gy) + lift, ez as f32);
    if let Ok(mut t) = sets.p1().get_single_mut() {
        t.translation = est;
    }
    if let Ok(mut t) = sets.p2().get_single_mut() {
        // Real 1-sigma radius, with a floor so it stays visible as a map symbol.
        let r = (radius as f32).max(terr.span_x * 0.0016);
        t.translation = Vec3::new(est.x, est.y - lift * 0.5, est.z);
        t.scale = Vec3::new(r, 1.0, r);
    }
}

/// Top-down navigation view: holds the astronaut centred, north-up, like a real
/// navigation display. Smoothed so it glides rather than snapping.
fn nav_camera_system(
    time: Res<Time>,
    mode: Res<AppMode>,
    astro: Res<Astronaut>,
    mut zoom: ResMut<NavZoom>,
    mut wheel: EventReader<MouseWheel>,
    mut snap: ResMut<CamSnap>,
    terr: Res<Terrain>,
    mut cam: Query<&mut Transform, With<MainCamera>>,
) {
    if *mode != AppMode::Navigate {
        wheel.clear();
        return;
    }
    for e in wheel.read() {
        zoom.0 = (zoom.0 * (1.0 - e.y * 0.14)).clamp(terr.span_x * 0.012, terr.span_x * 0.9);
    }
    let dt = time.delta_seconds().min(0.05);
    let eye = Vec3::new(astro.pos.x, astro.pos.y + zoom.0, astro.pos.z);
    // Looking straight down with north (-Z) up the screen.
    let desired_rot = Transform::from_translation(eye)
        .looking_to(Vec3::NEG_Y, Vec3::NEG_Z)
        .rotation;
    for mut t in &mut cam {
        if snap.0 {
            t.translation = eye;
            t.rotation = desired_rot;
        } else {
            let k = 1.0 - (-8.0 * dt).exp();
            t.translation = t.translation.lerp(eye, k);
            t.rotation = t.rotation.slerp(desired_rot, k);
        }
    }
    snap.0 = false;
}

fn screenshot_system(
    input: Res<ButtonInput<KeyCode>>,
    main_window: Query<Entity, With<PrimaryWindow>>,
    mut mgr: ResMut<ScreenshotManager>,
    mut captured: Local<bool>,
    time: Res<Time>,
) {
    let auto = !*captured && std::env::var("MARS_SHOT").is_ok() && time.elapsed_seconds() > 3.0;
    if auto || input.just_pressed(KeyCode::F12) {
        if let Ok(win) = main_window.get_single() {
            let _ = mgr.save_screenshot_to_disk(win, "/tmp/mars_shot.png");
            *captured = true;
        }
    }
}
