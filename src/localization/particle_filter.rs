use nalgebra::{Matrix4, Vector4, Vector2};
use rand::Rng;
use rand_distr::{Normal, Distribution};
use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

// Estimation parameter of PF
const Q: f64 = 0.2; // range error
const R_V: f64 = 2.0; // velocity error
const R_YAW: f64 = 40.0 * PI / 180.0; // yaw rate error

// Simulation parameter
const Q_SIM: f64 = 0.2;
const R_SIM_V: f64 = 1.0;
const R_SIM_YAW: f64 = 30.0 * PI / 180.0;

const DT: f64 = 0.1; // time tick [s]
const SIM_TIME: f64 = 50.0; // simulation time [s]
const MAX_RANGE: f64 = 20.0; // maximum observation range

// Particle filter parameter
const NP: usize = 100; // Number of Particle
const NTH: f64 = NP as f64 / 2.0; // Number of particle for re-sampling

const SHOW_ANIMATION: bool = true;

#[derive(Debug, Clone)]
pub struct Particle {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub w: f64, // weight
}

impl Particle {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64) -> Self {
        Particle {
            x,
            y,
            yaw,
            v,
            w: 1.0 / NP as f64,
        }
    }
}

pub struct ParticleFilter {
    particles: Vec<Particle>,
    n_particle: usize,
}

impl ParticleFilter {
    pub fn new(init_x: f64, init_y: f64, init_yaw: f64, init_v: f64) -> Self {
        let mut particles = Vec::with_capacity(NP);
        let mut rng = rand::thread_rng();
        
        for _ in 0..NP {
            let x = init_x + rng.gen::<f64>() * 2.0 - 1.0;
            let y = init_y + rng.gen::<f64>() * 2.0 - 1.0;
            let yaw = init_yaw + rng.gen::<f64>() * 0.5 - 0.25;
            let v = init_v + rng.gen::<f64>() * 1.0 - 0.5;
            particles.push(Particle::new(x, y, yaw, v));
        }

        ParticleFilter {
            particles,
            n_particle: NP,
        }
    }

    pub fn predict(&mut self, u: &Vector2<f64>) {
        let mut rng = rand::thread_rng();
        let normal_v = Normal::new(0.0, R_V.powi(2)).unwrap();
        let normal_yaw = Normal::new(0.0, R_YAW.powi(2)).unwrap();

        for particle in &mut self.particles {
            let v_noise = normal_v.sample(&mut rng);
            let yaw_noise = normal_yaw.sample(&mut rng);
            
            let v_noisy = u[0] + v_noise;
            let yaw_rate_noisy = u[1] + yaw_noise;

            particle.x += v_noisy * particle.yaw.cos() * DT;
            particle.y += v_noisy * particle.yaw.sin() * DT;
            particle.yaw += yaw_rate_noisy * DT;
            particle.v = v_noisy;
        }
    }

    pub fn update(&mut self, z: &[(f64, f64, f64)]) {
        for particle in &mut self.particles {
            let mut w = 1.0;
            
            for &(d_obs, landmark_x, landmark_y) in z {
                let dx = particle.x - landmark_x;
                let dy = particle.y - landmark_y;
                let d_pred = (dx * dx + dy * dy).sqrt();
                
                let diff = d_obs - d_pred;
                w *= gauss_likelihood(diff, Q);
            }
            
            particle.w = w;
        }

        self.normalize_weights();
    }

    fn normalize_weights(&mut self) {
        let sum_w: f64 = self.particles.iter().map(|p| p.w).sum();
        
        if sum_w > 0.0 {
            for particle in &mut self.particles {
                particle.w /= sum_w;
            }
        }
    }

    pub fn resample(&mut self) {
        let n_eff = self.calc_n_eff();
        
        if n_eff < NTH {
            self.resample_particles();
        }
    }

    fn calc_n_eff(&self) -> f64 {
        let sum_w_squared: f64 = self.particles.iter().map(|p| p.w * p.w).sum();
        1.0 / sum_w_squared
    }

    fn resample_particles(&mut self) {
        let mut rng = rand::thread_rng();
        let mut new_particles = Vec::with_capacity(self.n_particle);
        
        // Calculate cumulative weights
        let mut cumulative_weights = Vec::with_capacity(self.n_particle);
        let mut cum_sum = 0.0;
        for particle in &self.particles {
            cum_sum += particle.w;
            cumulative_weights.push(cum_sum);
        }

        for _ in 0..self.n_particle {
            let r = rng.gen::<f64>();
            
            // Find particle to resample
            let mut index = 0;
            for (i, &cum_w) in cumulative_weights.iter().enumerate() {
                if r <= cum_w {
                    index = i;
                    break;
                }
            }
            
            let mut new_particle = self.particles[index].clone();
            new_particle.w = 1.0 / self.n_particle as f64;
            new_particles.push(new_particle);
        }

        self.particles = new_particles;
    }

    pub fn estimate(&self) -> Vector4<f64> {
        let mut x_est = 0.0;
        let mut y_est = 0.0;
        let mut yaw_est = 0.0;
        let mut v_est = 0.0;

        for particle in &self.particles {
            x_est += particle.w * particle.x;
            y_est += particle.w * particle.y;
            yaw_est += particle.w * particle.yaw;
            v_est += particle.w * particle.v;
        }

        Vector4::new(x_est, y_est, yaw_est, v_est)
    }

    pub fn calc_covariance(&self, x_est: &Vector4<f64>) -> Matrix4<f64> {
        let mut cov = Matrix4::zeros();
        
        for particle in &self.particles {
            let dx = Vector4::new(
                particle.x - x_est[0],
                particle.y - x_est[1],
                particle.yaw - x_est[2],
                particle.v - x_est[3],
            );
            
            cov += particle.w * dx * dx.transpose();
        }

        cov
    }
}

fn gauss_likelihood(x: f64, sigma: f64) -> f64 {
    let coeff = 1.0 / (2.0 * PI * sigma.powi(2)).sqrt();
    coeff * (-x.powi(2) / (2.0 * sigma.powi(2))).exp()
}

fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>) -> Vector4<f64> {
    let f = Matrix4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
    );

    let b = Matrix4::from_columns(&[
        Vector4::new(DT * x[2].cos(), DT * x[2].sin(), 0.0, 1.0),
        Vector4::new(0.0, 0.0, DT, 0.0),
        Vector4::zeros(),
        Vector4::zeros(),
    ]);

    f * x + b * Vector4::new(u[0], u[1], 0.0, 0.0)
}

fn calc_input() -> Vector2<f64> {
    Vector2::new(1.0, 0.1) // [v, yaw_rate]
}

fn observation(
    x_true: &Vector4<f64>,
    xd: &Vector4<f64>,
    u: &Vector2<f64>,
    rf_id: &[(f64, f64)],
) -> (Vector4<f64>, Vec<(f64, f64, f64)>, Vector4<f64>, Vector2<f64>) {
    let mut rng = rand::thread_rng();
    let normal_q = Normal::new(0.0, Q_SIM.powi(2)).unwrap();
    let normal_v = Normal::new(0.0, R_SIM_V.powi(2)).unwrap();
    let normal_yaw = Normal::new(0.0, R_SIM_YAW.powi(2)).unwrap();

    let x_true = motion_model(x_true, u);

    // Generate observations
    let mut z = Vec::new();
    for &(landmark_x, landmark_y) in rf_id {
        let dx = x_true[0] - landmark_x;
        let dy = x_true[1] - landmark_y;
        let d = (dx * dx + dy * dy).sqrt();
        
        if d <= MAX_RANGE {
            let dn = d + normal_q.sample(&mut rng);
            z.push((dn, landmark_x, landmark_y));
        }
    }

    // Add noise to input
    let ud1 = u[0] + normal_v.sample(&mut rng);
    let ud2 = u[1] + normal_yaw.sample(&mut rng);
    let ud = Vector2::new(ud1, ud2);

    let xd = motion_model(xd, &ud);

    (x_true, z, xd, ud)
}

fn main() {
    println!("Particle Filter localization start!!");

    let mut time = 0.0;

    // RFID positions [x, y]
    let rf_id = vec![
        (10.0, 0.0),
        (10.0, 10.0),
        (0.0, 15.0),
        (-5.0, 20.0),
    ];

    // Initial state
    let mut x_est = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut x_true = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut xd = Vector4::new(0.0, 0.0, 0.0, 0.0);

    let mut pf = ParticleFilter::new(0.0, 0.0, 0.0, 0.0);

    // History for plotting
    let mut h_x_est = vec![0.0];
    let mut h_y_est = vec![0.0];
    let mut h_x_true = vec![0.0];
    let mut h_y_true = vec![0.0];
    let mut h_x_dr = vec![0.0];
    let mut h_y_dr = vec![0.0];

    while time <= SIM_TIME {
        time += DT;
        let u = calc_input();

        let (x_true_new, z, xd_new, ud) = observation(&x_true, &xd, &u, &rf_id);
        x_true = x_true_new;
        xd = xd_new;

        pf.predict(&ud);
        pf.update(&z);
        pf.resample();

        x_est = pf.estimate();

        // Store history
        h_x_est.push(x_est[0]);
        h_y_est.push(x_est[1]);
        h_x_true.push(x_true[0]);
        h_y_true.push(x_true[1]);
        h_x_dr.push(xd[0]);
        h_y_dr.push(xd[1]);

        if time.rem_euclid(5.0) < DT {
            println!("Time: {:.1}, Est: ({:.2}, {:.2}), True: ({:.2}, {:.2})", 
                     time, x_est[0], x_est[1], x_true[0], x_true[1]);
        }
    }

    if SHOW_ANIMATION {
        let mut fg = Figure::new();
        fg.axes2d()
            .lines(&h_x_true, &h_y_true, &[Caption("True"), Color("blue")])
            .lines(&h_x_dr, &h_y_dr, &[Caption("Dead Reckoning"), Color("black")])
            .lines(&h_x_est, &h_y_est, &[Caption("PF"), Color("red")])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Save to file
        let output_path = "img/localization/particle_filter_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);
        
        fg.show().unwrap();
    }

    println!("Particle Filter localization completed!");
}
