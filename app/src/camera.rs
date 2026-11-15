//! Free exploration camera: orbit (left-drag), zoom (wheel), and pan/fly
//! (WASD + Q/E) over the whole terrain — unconstrained, Google-Earth style.

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;

#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
}

impl OrbitCamera {
    pub fn apply(&self, tf: &mut Transform) {
        let rot = Quat::from_euler(EulerRot::YXZ, self.yaw, self.pitch, 0.0);
        let offset = rot * Vec3::new(0.0, 0.0, self.dist);
        tf.translation = self.focus + offset;
        tf.look_at(self.focus, Vec3::Y);
    }
}

/// Orbit/pan/zoom. Inactive while the camera is in first-person (that mode drives
/// the transform itself), signalled by the `Active` flag resource.
pub fn orbit_camera_system(
    mut q: Query<(&mut OrbitCamera, &mut Transform)>,
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut motion: EventReader<MouseMotion>,
    mut wheel: EventReader<MouseWheel>,
    time: Res<Time>,
    active: Res<CameraActive>,
) {
    let mut drag = Vec2::ZERO;
    for e in motion.read() {
        drag += e.delta;
    }
    let mut scroll = 0.0;
    for e in wheel.read() {
        scroll += e.y;
    }
    if !active.orbit {
        return;
    }

    let dt = time.delta_seconds();
    for (mut orbit, mut tf) in &mut q {
        if buttons.pressed(MouseButton::Left) {
            orbit.yaw -= drag.x * 0.005;
            orbit.pitch = (orbit.pitch - drag.y * 0.005).clamp(-1.45, -0.03);
        }
        if scroll != 0.0 {
            orbit.dist = (orbit.dist * (1.0 - scroll * 0.12)).clamp(25.0, 60_000.0);
        }

        // horizontal forward/right from yaw
        let forward = Vec3::new(-orbit.yaw.sin(), 0.0, -orbit.yaw.cos());
        let right = Vec3::new(orbit.yaw.cos(), 0.0, -orbit.yaw.sin());
        let speed = orbit.dist * 0.9 * dt;
        let mut pan = Vec3::ZERO;
        if keys.pressed(KeyCode::KeyW) {
            pan += forward;
        }
        if keys.pressed(KeyCode::KeyS) {
            pan -= forward;
        }
        if keys.pressed(KeyCode::KeyD) {
            pan += right;
        }
        if keys.pressed(KeyCode::KeyA) {
            pan -= right;
        }
        if keys.pressed(KeyCode::KeyE) {
            orbit.focus.y += speed;
        }
        if keys.pressed(KeyCode::KeyQ) {
            orbit.focus.y -= speed;
        }
        if pan != Vec3::ZERO {
            orbit.focus += pan.normalize() * speed;
        }

        orbit.apply(&mut tf);
    }
}

/// Which camera controller is driving: orbit (explore/map) or first-person.
#[derive(Resource)]
pub struct CameraActive {
    pub orbit: bool,
}
impl Default for CameraActive {
    fn default() -> Self {
        CameraActive { orbit: true }
    }
}
