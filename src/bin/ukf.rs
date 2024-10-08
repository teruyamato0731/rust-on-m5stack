extern crate nalgebra as na;

use na::{matrix, vector};
use rand_distr::{Distribution, Normal};

const M1: f64 = 150e-3;
const R_W: f64 = 50e-3;
const M2: f64 = 2.3 - 2.0 * M1 + 2.0;
const L: f64 = 0.2474; // 重心までの距離
const J1: f64 = M1 * R_W * R_W;
const J2: f64 = 0.1;
const G: f64 = 9.81;
const KT: f64 = 0.15; // m2006
const D: f64 = (M1 + M2 + J1 / R_W * R_W) * (M2 * L * L + J2) - M2 * M2 * L * L;

const DT: f64 = 0.01;
const Q: na::Matrix4<f64> = matrix![
    0.0, 0.0, 0.0, 0.0;
    0.0, 1.0, 0.0, 0.0;
    0.0, 0.0, 0.25, 0.5;
    0.0, 0.0, 0.5, 1.0;
];
const R: na::Matrix2<f64> = matrix![
    0.5, 0.5;
    0.5, 0.5;
];

// LAMBDA = α^2 * (n + κ) - n
const N: f64 = 4.0;
const ALPHA: f64 = 1e-3;
const BETA: f64 = 2.0;
const KAPPA: f64 = 3.0 - N;
const C: f64 = ALPHA * ALPHA * (N + KAPPA); // C := N + LAMBDA
const LAMBDA: f64 = C - N;

fn sigma_weight() -> (na::SVector<f64, 9>, na::SVector<f64, 9>) {
    let mut wm = na::SVector::<f64, 9>::from_element(1.0 / (2.0 * C));
    let mut wc = na::SVector::<f64, 9>::from_element(1.0 / (2.0 * C));
    wm[0] = LAMBDA / C;
    wc[0] = LAMBDA / C + 1.0 - ALPHA.powi(2) + BETA;
    (wm, wc)
}

fn sigma_points(x: &na::Vector4<f64>, p: &na::Matrix4<f64>) -> na::SMatrix<f64, 4, 9> {
    let l = (C * p).cholesky().expect("Cholesky fail").l();
    na::SMatrix::<f64, 4, 9>::from_columns(&[
        *x,
        *x + l.column(0),
        *x - l.column(0),
        *x + l.column(1),
        *x - l.column(1),
        *x + l.column(2),
        *x - l.column(2),
        *x + l.column(3),
        *x - l.column(3),
    ])
}

fn unscented_transform<const S: usize>(
    sigmas: &na::SMatrix<f64, S, 9>,
    wm: &na::SVector<f64, 9>,
    wc: &na::SVector<f64, 9>,
    cov: &na::SMatrix<f64, S, S>,
) -> (na::SVector<f64, S>, na::SMatrix<f64, S, S>) {
    let x = sigmas * wm;
    let y = sigmas - na::SMatrix::<f64, S, 9>::from_columns(&[x, x, x, x, x, x, x, x, x]);
    let mut tmp = na::SMatrix::<f64, S, S>::zeros();
    for i in 0..9 {
        tmp += wc[i] * y.column(i) * y.column(i).transpose();
    }
    let p = tmp + cov;
    (x, p)
}

// 状態遷移関数
fn fx(mut x: na::Vector4<f64>, u: na::Vector1<f64>) -> na::Vector4<f64> {
    x[3] +=
        ((M1 + M2 + J1 / R_W * R_W) / D * M2 * G * L * x[2] - M2 * L / D / R_W * KT * u[0]) * DT;
    x[2] += x[3] * DT;
    x[1] += (-M2 * M2 * G * L * L / D * x[2] + (M2 * L * L + J2) / D / R_W * KT * u[0]) * DT;
    x[0] += x[1] * DT;
    x
}

// 観測関数
fn hx(x_act: na::Vector4<f64>) -> na::Vector2<f64> {
    vector![
        x_act[1], // 駆動輪のオドメトリ
        x_act[3], // 角速度
    ]
}

fn predict(
    x: &mut na::Vector4<f64>,
    u: na::Vector1<f64>,
    p: &mut na::Matrix4<f64>,
) -> na::SMatrix<f64, 4, 9> {
    let mut sigmas = sigma_points(x, p);
    for i in 0..9 {
        sigmas.set_column(i, &fx(sigmas.column(i).into_owned(), u));
    }
    let (wm, wc) = sigma_weight();
    (*x, *p) = unscented_transform(&sigmas, &wm, &wc, &Q);
    sigmas
}

// センサ出力をシミュレーション
fn sensor(x_act: na::Vector4<f64>, rng: &mut rand::rngs::ThreadRng) -> na::Vector2<f64> {
    let mut x_obs = vector![
        x_act[1], // 駆動輪のオドメトリ
        x_act[3], // 角速度
    ];
    for i in 0..2 {
        let normal = Normal::new(0.0, R[(i, i)]).unwrap();
        x_obs[i] += normal.sample(rng);
    }
    x_obs
}

fn update(
    x_odom: &mut na::Vector4<f64>,
    x_obs: na::Vector2<f64>,
    p: &mut na::Matrix4<f64>,
    sigmas_f: &na::SMatrix<f64, 4, 9>,
) {
    let mut sigmas_h = na::SMatrix::<f64, 2, 9>::zeros();
    for i in 0..9 {
        sigmas_h.set_column(i, &hx(sigmas_f.column(i).into_owned()));
    }
    let (wm, wc) = sigma_weight();
    let (zp, pz) = unscented_transform(&sigmas_h, &wm, &wc, &R);
    let mut pxz = na::SMatrix::<f64, 4, 2>::zeros();
    for i in 0..9 {
        pxz += wc[i] * (sigmas_f.column(i) - *x_odom) * (sigmas_h.column(i) - zp).transpose();
    }
    let k = pxz * pz.try_inverse().expect("Inverse fail");
    *x_odom += k * (x_obs - zp);
    *p -= k * pz * k.transpose();
    // 対称性の維持
    *p = (*p + p.transpose()) / 2.0;
}

fn app() {
    let mut rng = rand::thread_rng();

    let mut x_act = vector![0.0, 0.0, 0.0, 0.0];
    let mut x_est = vector![0.0, 0.0, 0.0, 0.0];
    let mut p = matrix![
        10.0, 0.0, 0.0, 0.0;
        0.0, 10.0, 0.0, 0.0;
        0.0, 0.0, 10.0, 0.0;
        0.0, 0.0, 0.0, 10.0;
    ];
    loop {
        let u = vector![0.0];
        x_act = fx(x_act, u);
        let sigmas_f = predict(&mut x_est, u, &mut p);
        let x_obs = sensor(x_act, &mut rng);
        update(&mut x_est, x_obs, &mut p, &sigmas_f);

        print!(
            "x_act: ({:7.2},{:7.2},{:7.2},{:7.2}) ",
            x_act[0], x_act[1], x_act[2], x_act[3]
        );
        print!("x_obs: ({:7.2},{:7.2}) ", x_obs[0], x_obs[1]);
        print!(
            "x_est: ({:7.2},{:7.2},{:7.2},{:7.2}) ",
            x_est[0], x_est[1], x_est[2], x_est[3]
        );
        println!(
            "p: ({:7.2},{:7.2},{:7.2},{:7.2})",
            p[(0, 0)],
            p[(1, 1)],
            p[(2, 2)],
            p[(3, 3)]
        );
        // println!("p: {}", p);
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    app();
}
