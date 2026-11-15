//! Instrument overlay. Reads like equipment, not a web page: monospace, uppercase
//! labels, hairline rules, high contrast. The data panel toggles; the route panel
//! exists only while a path is planned.

use bevy::prelude::*;
use mars_nav::nav::FixSource;

use crate::env::Env;
use crate::mission::{Astronaut, Mission, NavState, REAL_WALK_SPEED};
use crate::route::NavGrid;
use crate::terrain::Terrain;
use crate::{AppMode, DataSource, SurfaceLayer};

const INK: Color = Color::srgb(0.96, 0.975, 0.99);
const LABEL: Color = Color::srgb(0.74, 0.83, 0.90);
const AMBER: Color = Color::srgb(1.0, 0.76, 0.32);
const CYAN: Color = Color::srgb(0.46, 0.87, 1.0);
const PANEL_BG: Color = Color::srgba(0.027, 0.042, 0.058, 0.93);
const LINE: Color = Color::srgba(0.50, 0.75, 0.88, 0.40);

const SAFE: Color = Color::srgb(0.24, 0.78, 0.46);
const CAUTION: Color = Color::srgb(0.98, 0.74, 0.20);
const AVOID: Color = Color::srgb(0.92, 0.26, 0.20);

/// MSL/RAD surface dose rate, mSv per sol, for the route dose estimate.
const DOSE_MSV_PER_SOL: f32 = 0.66;
const SOL_HOURS: f32 = 24.66;

#[derive(Resource)]
pub struct UiState {
    pub info_open: bool,
}
impl Default for UiState {
    fn default() -> Self {
        UiState { info_open: true }
    }
}

#[derive(Component)]
pub struct ModeText;
#[derive(Component)]
pub struct NavLine;
#[derive(Component)]
pub struct InfoPanel;
#[derive(Component)]
pub struct SurfaceText;
#[derive(Component)]
pub struct EnvText;
#[derive(Component)]
pub struct NavInfoText;
#[derive(Component)]
pub struct SourceText;
#[derive(Component)]
pub struct RoutePanel;
#[derive(Component)]
pub struct RouteChart;
#[derive(Component)]
pub struct RouteStatsText;
#[derive(Component)]
pub struct RouteAxisText;

fn label(text: &str, size: f32, color: Color) -> TextBundle {
    TextBundle::from_section(text, TextStyle { font_size: size, color, ..default() })
}

fn panel_node(style: Style) -> NodeBundle {
    NodeBundle {
        style,
        background_color: BackgroundColor(PANEL_BG),
        border_color: BorderColor(LINE),
        ..default()
    }
}

pub fn hazard_color(slope_deg: f32) -> Color {
    if slope_deg >= 20.0 {
        AVOID
    } else if slope_deg >= 10.0 {
        CAUTION
    } else {
        SAFE
    }
}

pub fn setup_ui(commands: &mut Commands) {
    // Top-left: identity, mode, active layer.
    commands
        .spawn(panel_node(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(16.0),
            left: Val::Px(18.0),
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(3.0),
            padding: UiRect::axes(Val::Px(11.0), Val::Px(7.0)),
            border: UiRect::all(Val::Px(1.0)),
            ..default()
        }))
        .with_children(|p| {
            p.spawn(label("MARS  EXPLORER", 16.0, INK));
            p.spawn((label("EXPLORE", 11.0, AMBER), ModeText));
        });

    // Bottom-left: one-line navigation summary.
    commands
        .spawn(panel_node(Style {
            position_type: PositionType::Absolute,
            bottom: Val::Px(16.0),
            left: Val::Px(18.0),
            padding: UiRect::axes(Val::Px(10.0), Val::Px(6.0)),
            border: UiRect::all(Val::Px(1.0)),
            ..default()
        }))
        .with_children(|p| {
            p.spawn((label("", 14.0, INK), NavLine));
        });

    // Top-right: surface / environment / navigation data.
    commands
        .spawn((
            panel_node(Style {
                position_type: PositionType::Absolute,
                top: Val::Px(16.0),
                right: Val::Px(16.0),
                width: Val::Px(266.0),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(13.0)),
                row_gap: Val::Px(5.0),
                border: UiRect::all(Val::Px(1.0)),
                ..default()
            }),
            InfoPanel,
        ))
        .with_children(|p| {
            p.spawn(label("SURFACE  —  MEASURED", 10.0, CYAN));
            p.spawn((label("", 12.5, INK), SurfaceText));
            p.spawn(label("ENVIRONMENT  —  MODELED", 10.0, AMBER));
            p.spawn((label("", 12.5, INK), EnvText));
            p.spawn(label("GPS-FREE  NAVIGATION", 10.0, CYAN));
            p.spawn((label("", 12.5, INK), NavInfoText));
            p.spawn((label("", 9.5, LABEL), SourceText));
        });

    // Bottom-right: the full key list.
    commands
        .spawn(panel_node(Style {
            position_type: PositionType::Absolute,
            bottom: Val::Px(16.0),
            right: Val::Px(16.0),
            width: Val::Px(266.0),
            flex_direction: FlexDirection::Column,
            padding: UiRect::all(Val::Px(13.0)),
            row_gap: Val::Px(5.0),
            border: UiRect::all(Val::Px(1.0)),
            ..default()
        }))
        .with_children(|p| {
            p.spawn(label("CONTROLS", 10.0, CYAN));
            p.spawn(label(
                "CLICK     designate target\n\
                 G         go / stop\n\
                 X         clear route\n\
                 I         data panel\n\
                 \n\
                 1         explore (3D)\n\
                 2         map (overview)\n\
                 3         navigate (top-down)\n\
                 \n\
                 4         CTX imagery\n\
                 5         elevation\n\
                 6         danger zones\n\
                 7         thermal IR\n\
                 \n\
                 WASD      move (navigate)\n\
                 SHIFT     move fast\n\
                 DRAG      orbit / pan\n\
                 SCROLL    zoom\n\
                 F12       screenshot",
                11.5,
                INK,
            ));
        });

    // Bottom-left, above the summary: route profile. Hidden until a path exists.
    commands
        .spawn((
            panel_node(Style {
                position_type: PositionType::Absolute,
                bottom: Val::Px(62.0),
                left: Val::Px(18.0),
                width: Val::Px(372.0),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(12.0)),
                row_gap: Val::Px(7.0),
                border: UiRect::all(Val::Px(1.0)),
                display: Display::None,
                ..default()
            }),
            RoutePanel,
        ))
        .with_children(|p| {
            p.spawn(label("ROUTE  PROFILE", 10.0, CYAN));
            p.spawn((
                NodeBundle {
                    style: Style {
                        width: Val::Percent(100.0),
                        height: Val::Px(74.0),
                        flex_direction: FlexDirection::Row,
                        align_items: AlignItems::FlexEnd,
                        ..default()
                    },
                    ..default()
                },
                RouteChart,
            ));
            p.spawn((label("", 9.5, LABEL), RouteAxisText));
            p.spawn((label("", 12.0, INK), RouteStatsText));
            p.spawn(label("bar height = elevation    colour = slope hazard", 9.5, LABEL));
        });
}

/// Compass bearing (deg from north) of a horizontal direction: +X east, -Z north.
pub fn bearing_of(dir: Vec3) -> f32 {
    let b = dir.x.atan2(-dir.z).to_degrees();
    (b + 360.0) % 360.0
}

fn fmt_dist(m: f32) -> String {
    if m >= 1000.0 {
        format!("{:.2} km", m / 1000.0)
    } else {
        format!("{:.0} m", m)
    }
}

fn fmt_hm(hours: f32) -> String {
    let h = hours.floor() as u32;
    let m = ((hours - h as f32) * 60.0).round() as u32;
    format!("{h}h {m:02}m")
}

pub fn toggle_info_panel(ui: Res<UiState>, mut q: Query<&mut Style, With<InfoPanel>>) {
    if !ui.is_changed() {
        return;
    }
    for mut s in &mut q {
        s.display = if ui.info_open { Display::Flex } else { Display::None };
    }
}

/// Rebuild the profile bars when the planned route actually changes — not every
/// frame while it is being walked.
pub fn rebuild_route_chart(
    mission: Res<Mission>,
    mut commands: Commands,
    chart: Query<Entity, With<RouteChart>>,
    mut panel: Query<&mut Style, With<RoutePanel>>,
    mut last: Local<Option<(usize, f32)>>,
) {
    let key = mission.route.as_ref().map(|r| (r.samples.len(), r.length_m));
    if *last == key {
        return;
    }
    *last = key;

    if let Ok(mut s) = panel.get_single_mut() {
        s.display = if key.is_some() { Display::Flex } else { Display::None };
    }
    let Ok(chart_e) = chart.get_single() else { return };
    commands.entity(chart_e).despawn_descendants();

    let Some(r) = mission.route.as_ref() else { return };
    let range = (r.elev_max - r.elev_min).max(1.0);
    let n = r.samples.len().max(1);
    let w = 100.0 / n as f32;
    for s in &r.samples {
        let h = 14.0 + 86.0 * ((s.elev_m - r.elev_min) / range);
        let color = hazard_color(s.slope_deg);
        commands.entity(chart_e).with_children(|p| {
            p.spawn(NodeBundle {
                style: Style {
                    width: Val::Percent(w),
                    height: Val::Percent(h),
                    ..default()
                },
                background_color: BackgroundColor(color),
                ..default()
            });
        });
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub fn update_ui(
    astro: Res<Astronaut>,
    mission: Res<Mission>,
    nav: Res<NavState>,
    terrain: Res<Terrain>,
    grid: Res<NavGrid>,
    envr: Res<Env>,
    mode: Res<AppMode>,
    layer: Res<SurfaceLayer>,
    source: Res<DataSource>,
    mut texts: ParamSet<(
        Query<&mut Text, With<ModeText>>,
        Query<&mut Text, With<NavLine>>,
        Query<&mut Text, With<SurfaceText>>,
        Query<&mut Text, With<EnvText>>,
        Query<&mut Text, With<NavInfoText>>,
        Query<&mut Text, With<SourceText>>,
        Query<&mut Text, With<RouteStatsText>>,
        Query<&mut Text, With<RouteAxisText>>,
    )>,
) {
    let mode_name = match *mode {
        AppMode::Explore => "EXPLORE",
        AppMode::Map => "MAP",
        AppMode::Navigate => "NAVIGATE",
    };
    if let Ok(mut t) = texts.p0().get_single_mut() {
        t.sections[0].value = format!("{mode_name}   ·   {}", layer.label());
    }

    let (ex, ez, _h, radius) = nav.sim.reported();
    // What the filter actually did on its last step.
    let fix = match nav.sim.last_fix_source {
        FixSource::Visual => "VISUAL",
        FixSource::DeadReckoning => "INERTIAL",
    };

    if let Ok(mut t) = texts.p1().get_single_mut() {
        t.sections[0].value = match mission.target {
            Some(tg) => {
                let d = Vec3::new(tg.x - astro.pos.x, 0.0, tg.z - astro.pos.z);
                let route_txt = mission
                    .route
                    .as_ref()
                    .map(|r| format!("   ROUTE {}", fmt_dist(r.length_m)))
                    .unwrap_or_else(|| "   NO PATH".into());
                format!(
                    "TARGET  {}   BRG {:03.0}{}",
                    fmt_dist(d.length()),
                    bearing_of(d),
                    route_txt
                )
            }
            None => "NO TARGET   —   CLICK THE SURFACE TO DESIGNATE".to_string(),
        };
    }

    // --- surface (measured) ---
    let (gx, gy) = terrain.world_to_grid(astro.pos.x, astro.pos.z);
    let gxi = (gx.round().max(0.0) as usize).min(terrain.dem.width - 1);
    let gyi = (gy.round().max(0.0) as usize).min(terrain.dem.height - 1);
    let (lon, lat) = terrain.dem.grid_to_lonlat(gxi, gyi);
    let elev = terrain.dem.sample(gx, gy);
    let slope_deg = grid.slope_at(&terrain, astro.pos);
    let hazard = if slope_deg >= 20.0 {
        "AVOID"
    } else if slope_deg >= 10.0 {
        "CAUTION"
    } else {
        "NOMINAL"
    };
    let hdg = bearing_of(Vec3::new(astro.heading.cos(), 0.0, astro.heading.sin()));
    let base_d = Vec3::new(mission.base.x - astro.pos.x, 0.0, mission.base.z - astro.pos.z);
    let tgt_line = match mission.target {
        Some(tg) => {
            let d = Vec3::new(tg.x - astro.pos.x, 0.0, tg.z - astro.pos.z);
            format!("{}  BRG {:03.0}", fmt_dist(d.length()), bearing_of(d))
        }
        None => "—".to_string(),
    };

    if let Ok(mut t) = texts.p2().get_single_mut() {
        t.sections[0].value = format!(
            "LAT        {lat:.4} N\n\
             LON        {lon:.4} E\n\
             ELEVATION  {elev:.0} m\n\
             SLOPE      {slope_deg:.1} deg  {hazard}\n\
             HEADING    {hdg:03.0}\n\
             TARGET     {tgt_line}\n\
             BASE       {}  BRG {:03.0}",
            fmt_dist(base_d.length()),
            bearing_of(base_d),
        );
    }

    // --- environment (modeled) ---
    if let Ok(mut t) = texts.p3().get_single_mut() {
        t.sections[0].value = format!(
            "LOCAL TIME {}  SOL {}\n\
             SUN        {:.0} deg  {}\n\
             AIR TEMP   {:.0} C\n\
             GROUND     {:.0} C\n\
             PRESSURE   {:.0} Pa\n\
             WIND       {:.1} m/s\n\
             DUST TAU   {:.2}\n\
             RADIATION  {:.0} uSv/h",
            envr.clock(),
            envr.sol,
            envr.solar_elevation_deg(),
            envr.light_state(),
            envr.air_temp_c(elev),
            envr.surface_temp_c(elev),
            envr.pressure_pa(elev),
            envr.wind_ms(),
            envr.dust_opacity(),
            envr.dose_rate_usv_h(),
        );
    }

    // --- navigation estimate ---
    if let Ok(mut t) = texts.p4().get_single_mut() {
        let (rgx, rgy) = terrain.world_to_grid(ex as f32, ez as f32);
        let (rlon, rlat) = terrain.dem.grid_to_lonlat(
            (rgx.round().max(0.0) as usize).min(terrain.dem.width - 1),
            (rgy.round().max(0.0) as usize).min(terrain.dem.height - 1),
        );
        t.sections[0].value = format!(
            "LAST FIX   {fix}\n\
             UNCERT     +/- {radius:.1} m\n\
             REPORTED   {rlat:.4} N {rlon:.4} E"
        );
    }

    if let Ok(mut t) = texts.p5().get_single_mut() {
        let kind = if source.real { "MEASURED" } else { "SYNTHETIC" };
        t.sections[0].value = format!("{}\nDATA {}  ·  {}", layer.legend(), kind, source.text);
    }

    // --- route distance axis, under the profile ---
    if let Ok(mut t) = texts.p7().get_single_mut() {
        if let Some(r) = mission.route.as_ref() {
            let km = |s: Option<&crate::route::RouteSample>| {
                s.map(|s| format!("{:.1}", s.dist_m / 1000.0)).unwrap_or_default()
            };
            let start = km(r.samples.first());
            let mid = km(r.samples.get(r.samples.len() / 2));
            let end = km(r.samples.last());
            t.sections[0].value = format!("{start:<19}{mid:^19}{end:>16} km");
        }
    }

    // --- route statistics ---
    if let Ok(mut t) = texts.p6().get_single_mut() {
        if let Some(r) = mission.route.as_ref() {
            let direct = mission
                .target
                .map(|tg| Vec3::new(tg.x - mission.base.x, 0.0, tg.z - mission.base.z).length());
            let hours = r.length_m / REAL_WALK_SPEED as f32 / 3600.0;
            let dose = DOSE_MSV_PER_SOL * hours / SOL_HOURS;
            let remaining = r.length_m * (1.0 - mission.progress);
            t.sections[0].value = format!(
                "DISTANCE   {}\n\
                 REMAINING  {}\n\
                 DIRECT     {}\n\
                 CLIMB      +{:.0} m  /  -{:.0} m\n\
                 MAX SLOPE  {:.1} deg\n\
                 TERRAIN    {:.0}% ok  {:.0}% caution  {:.0}% avoid\n\
                 EST TIME   {}  at 1.3 m/s\n\
                 EST DOSE   {:.2} mSv",
                fmt_dist(r.length_m),
                fmt_dist(remaining),
                direct.map(fmt_dist).unwrap_or_else(|| "—".into()),
                r.ascent_m,
                r.descent_m,
                r.max_slope_deg,
                r.frac[0] * 100.0,
                r.frac[1] * 100.0,
                r.frac[2] * 100.0,
                fmt_hm(hours),
                dose,
            );
        }
    }
}
