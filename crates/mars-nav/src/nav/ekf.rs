//! A 3-state Extended Kalman Filter for GPS-free surface navigation.
//!
//! State `x = [px, py, heading]` (meters, meters, radians). The motion model is a
//! unicycle driven by INS controls `(v, ω)` — forward speed and yaw rate — which is
//! nonlinear in `heading`, so the filter is genuinely *extended* (it linearizes the
//! model each step via the Jacobian `F`). Position fixes from visual localization
//! enter through a linear measurement `z = [px, py]`.
//!
//! Tiny fixed-size linear algebra is written out by hand so `mars-nav` stays
//! dependency-free.

pub type Vec3 = [f64; 3];
pub type Mat3 = [[f64; 3]; 3];

fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    o
}
fn mat_t(a: &Mat3) -> Mat3 {
    let mut o = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = a[j][i];
        }
    }
    o
}
fn mat_add(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = a[i][j] + b[i][j];
        }
    }
    o
}

/// Wrap an angle to (-π, π].
pub fn wrap_angle(a: f64) -> f64 {
    let mut a = a % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    } else if a <= -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

#[derive(Clone, Debug)]
pub struct Ekf {
    /// State: px, py, heading.
    pub x: Vec3,
    /// State covariance.
    pub p: Mat3,
    /// Process-noise spectral density on position (m²/s per unit speed) and heading.
    pub q_pos: f64,
    pub q_head: f64,
}

impl Ekf {
    pub fn new(px: f64, py: f64, heading: f64) -> Self {
        Ekf {
            x: [px, py, heading],
            // Start confident (we know we're at base), but not singular.
            p: [[0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 0.01]],
            q_pos: 0.05,
            q_head: 0.004,
        }
    }

    pub fn pos(&self) -> (f64, f64) {
        (self.x[0], self.x[1])
    }
    pub fn heading(&self) -> f64 {
        self.x[2]
    }

    /// 1-σ position uncertainty radius (meters): √(trace of the position block).
    pub fn uncertainty_radius(&self) -> f64 {
        (self.p[0][0] + self.p[1][1]).max(0.0).sqrt()
    }

    /// EKF predict using an INS control `(v, ω)` over `dt` seconds.
    pub fn predict(&mut self, v: f64, omega: f64, dt: f64) {
        let th = self.x[2];
        let (s, c) = th.sin_cos();
        // f(x,u)
        self.x[0] += v * c * dt;
        self.x[1] += v * s * dt;
        self.x[2] = wrap_angle(th + omega * dt);

        // Jacobian F = ∂f/∂x
        let f = [
            [1.0, 0.0, -v * s * dt],
            [0.0, 1.0, v * c * dt],
            [0.0, 0.0, 1.0],
        ];
        // Process noise grows with distance travelled (INS drift accrues with motion).
        let speed = v.abs();
        let qp = self.q_pos * dt * (0.2 + speed);
        let q = [
            [qp, 0.0, 0.0],
            [0.0, qp, 0.0],
            [0.0, 0.0, self.q_head * dt * (0.2 + speed)],
        ];
        // P = F P Fᵀ + Q
        self.p = mat_add(&mat_mul(&mat_mul(&f, &self.p), &mat_t(&f)), &q);
    }

    /// EKF update from a visual position fix `z = (zx, zy)` with isotropic noise `r` (σ²).
    pub fn update_position(&mut self, zx: f64, zy: f64, r: f64) {
        // H = [[1,0,0],[0,1,0]].  S = H P Hᵀ + R  (2×2).
        let s00 = self.p[0][0] + r;
        let s01 = self.p[0][1];
        let s10 = self.p[1][0];
        let s11 = self.p[1][1] + r;
        let det = s00 * s11 - s01 * s10;
        if det.abs() < 1e-12 {
            return;
        }
        // S⁻¹
        let (i00, i01, i10, i11) = (s11 / det, -s01 / det, -s10 / det, s00 / det);
        // P Hᵀ is the first two columns of P (3×2).
        let ph = [
            [self.p[0][0], self.p[0][1]],
            [self.p[1][0], self.p[1][1]],
            [self.p[2][0], self.p[2][1]],
        ];
        // K = P Hᵀ S⁻¹  (3×2)
        let mut k = [[0.0; 2]; 3];
        for i in 0..3 {
            k[i][0] = ph[i][0] * i00 + ph[i][1] * i10;
            k[i][1] = ph[i][0] * i01 + ph[i][1] * i11;
        }
        // innovation y = z - H x
        let yx = zx - self.x[0];
        let yy = zy - self.x[1];
        // x += K y
        self.x[0] += k[0][0] * yx + k[0][1] * yy;
        self.x[1] += k[1][0] * yx + k[1][1] * yy;
        self.x[2] = wrap_angle(self.x[2] + k[2][0] * yx + k[2][1] * yy);
        // P = (I - K H) P ; (K H) has columns 0,1 = K and column 2 = 0.
        let mut kh = [[0.0; 3]; 3];
        for i in 0..3 {
            kh[i][0] = k[i][0];
            kh[i][1] = k[i][1];
        }
        let mut imkh = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                imkh[i][j] = if i == j { 1.0 } else { 0.0 } - kh[i][j];
            }
        }
        self.p = mat_mul(&imkh, &self.p);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn predict_grows_uncertainty() {
        let mut k = Ekf::new(0.0, 0.0, 0.0);
        let r0 = k.uncertainty_radius();
        for _ in 0..100 {
            k.predict(1.0, 0.0, 0.1);
        }
        assert!(k.uncertainty_radius() > r0, "dead-reckoning should grow P");
        // moved ~10 m east
        assert!((k.x[0] - 10.0).abs() < 0.1);
    }

    #[test]
    fn visual_fix_shrinks_uncertainty() {
        let mut k = Ekf::new(0.0, 0.0, 0.0);
        for _ in 0..100 {
            k.predict(1.0, 0.0, 0.1);
        }
        let before = k.uncertainty_radius();
        k.update_position(10.0, 0.0, 0.0625); // σ = 0.25 m
        let after = k.uncertainty_radius();
        assert!(after < before, "a visual fix must reduce uncertainty");
        assert!(after > 0.0 && after.is_finite());
    }

    #[test]
    fn covariance_stays_positive() {
        let mut k = Ekf::new(0.0, 0.0, 0.5);
        for i in 0..200 {
            k.predict(1.2, 0.05, 0.1);
            if i % 20 == 0 {
                k.update_position(k.x[0] + 0.1, k.x[1] - 0.1, 0.0625);
            }
        }
        assert!(k.p[0][0] > 0.0 && k.p[1][1] > 0.0 && k.p[2][2] > 0.0);
    }
}
