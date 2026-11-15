//! GPS-free hybrid navigation simulation.
//!
//! Combines the three technologies from the research concept:
//!   - **Inertial Navigation (INS)** — dead-reckoning from noisy, biased `(v, ω)`
//!     controls. Accurate briefly, but drifts without correction.
//!   - **Visual Localization** — periodic position fixes (~0.25 m, after Mars
//!     Global Localization) available only when terrain is "recognizable"
//!     (i.e. not during a dust storm / darkness outage).
//!   - **Sensor Fusion (EKF)** — an [`ekf::Ekf`] predicts from INS and corrects
//!     from visual fixes, tracking both the estimate and its uncertainty.
//!
//! The app feeds the *true* pose each tick (ground truth, which a real system
//! never has); the simulation produces the *estimated* pose + uncertainty a real
//! astronaut's system would report, so the two can be shown side by side.

pub mod ekf;

use ekf::{wrap_angle, Ekf};

/// Where the current reported position came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixSource {
    /// Last update was a visual localization fix.
    Visual,
    /// Running on INS dead-reckoning only (uncertainty rising).
    DeadReckoning,
}

/// Small deterministic RNG (mulberry32) + Box–Muller normal draws, so the sim is
/// dependency-free and reproducible.
struct Rng(u32);
impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x6D2B_79F5);
        let mut t = self.0;
        t = (t ^ (t >> 15)).wrapping_mul(1 | t);
        t ^= t.wrapping_add((t ^ (t >> 7)).wrapping_mul(61 | t));
        ((t ^ (t >> 14)) as f64) / 4_294_967_296.0
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-9);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct NavParams {
    /// Seconds between visual fixes when localization is available.
    pub fix_interval: f64,
    /// Visual fix std-dev, meters (≈0.25 m per MGL).
    pub visual_sigma: f64,
    /// Constant INS velocity bias (m/s) and gyro bias (rad/s) — the drift drivers.
    pub bias_v: f64,
    pub bias_w: f64,
    /// Per-step INS control noise (m/s, rad/s).
    pub noise_v: f64,
    pub noise_w: f64,
}
impl Default for NavParams {
    fn default() -> Self {
        NavParams {
            fix_interval: 6.0,
            visual_sigma: 0.25,
            bias_v: 0.02,
            bias_w: 0.006,
            noise_v: 0.05,
            noise_w: 0.02,
        }
    }
}

pub struct NavSim {
    pub ekf: Ekf,
    pub params: NavParams,
    pub last_fix_source: FixSource,
    /// Seconds since the last visual fix.
    pub since_fix: f64,
    rng: Rng,
    prev_true: Option<(f64, f64, f64)>,
}

impl NavSim {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        NavSim {
            ekf: Ekf::new(x, y, heading),
            params: NavParams::default(),
            last_fix_source: FixSource::Visual,
            since_fix: 0.0,
            rng: Rng(0x5EED_1234),
            prev_true: Some((x, y, heading)),
        }
    }

    /// Advance one tick given the astronaut's true pose and whether visual
    /// localization is currently available (false during a dust/darkness outage).
    pub fn step(&mut self, true_x: f64, true_y: f64, true_heading: f64, dt: f64, visual_available: bool) {
        let p = self.params;

        // Recover the true control from the change in true pose.
        let (v_true, w_true) = match self.prev_true {
            Some((px, py, pth)) if dt > 1e-6 => (
                ((true_x - px).hypot(true_y - py)) / dt,
                wrap_angle(true_heading - pth) / dt,
            ),
            _ => (0.0, 0.0),
        };

        // INS measures the control with bias + noise → the estimate drifts.
        let v_meas = v_true + p.bias_v + self.rng.normal() * p.noise_v;
        let w_meas = w_true + p.bias_w + self.rng.normal() * p.noise_w;
        // Sub-step the predict so a time-compressed frame still integrates at a
        // small dt (keeps the EKF linearization valid and drift realistic).
        let subs = (dt / 0.2).ceil().max(1.0);
        let dt_sub = dt / subs;
        for _ in 0..subs as usize {
            self.ekf.predict(v_meas, w_meas, dt_sub);
        }

        self.since_fix += dt;
        if visual_available && self.since_fix >= p.fix_interval {
            self.since_fix = 0.0;
            let zx = true_x + self.rng.normal() * p.visual_sigma;
            let zy = true_y + self.rng.normal() * p.visual_sigma;
            self.ekf.update_position(zx, zy, p.visual_sigma * p.visual_sigma);
            self.last_fix_source = FixSource::Visual;
        } else {
            self.last_fix_source = FixSource::DeadReckoning;
        }

        self.prev_true = Some((true_x, true_y, true_heading));
    }

    /// Reported position (x, y), heading, and 1-σ uncertainty radius (m).
    pub fn reported(&self) -> (f64, f64, f64, f64) {
        let (x, y) = self.ekf.pos();
        (x, y, self.ekf.heading(), self.ekf.uncertainty_radius())
    }

    /// Actual estimate error vs. ground truth (meters) — for display/debug only;
    /// a real system cannot observe this.
    pub fn true_error(&self, true_x: f64, true_y: f64) -> f64 {
        let (x, y) = self.ekf.pos();
        (x - true_x).hypot(y - true_y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Walk due east at 1 m/s and sample the sim.
    fn walk(sim: &mut NavSim, steps: usize, dt: f64, visual: bool) -> (f64, f64) {
        let mut tx = 0.0;
        for _ in 0..steps {
            tx += 1.0 * dt;
            sim.step(tx, 0.0, 0.0, dt, visual);
        }
        (sim.reported().3, sim.true_error(tx, 0.0))
    }

    #[test]
    fn drifts_without_visual() {
        let mut sim = NavSim::new(0.0, 0.0, 0.0);
        let (radius, err) = walk(&mut sim, 600, 0.1, false); // 60 s, no fixes
        assert_eq!(sim.last_fix_source, FixSource::DeadReckoning);
        assert!(radius > 1.0, "uncertainty should grow large: {radius}");
        assert!(err > 0.5, "estimate should drift from truth: {err}");
    }

    #[test]
    fn visual_keeps_it_tight() {
        let mut open = NavSim::new(0.0, 0.0, 0.0);
        let (r_open, e_open) = walk(&mut open, 600, 0.1, true);
        let mut dark = NavSim::new(0.0, 0.0, 0.0);
        let (r_dark, e_dark) = walk(&mut dark, 600, 0.1, false);
        assert!(r_open < r_dark, "visual fixes bound uncertainty");
        assert!(e_open < e_dark, "visual fixes bound drift");
        // The open-sky run must have taken at least one visual fix overall.
        assert!(open.since_fix < 60.0, "visual fixes should have occurred");
    }
}
