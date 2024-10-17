extern crate nalgebra as na;

pub struct UnscentedKalmanFilter {
    x: na::Vector4<f32>,
    p: na::Matrix4<f32>,
    q: na::Matrix4<f32>,
    r: na::Matrix2<f32>,
    wm: na::SVector<f32, 9>,
    wc: na::SVector<f32, 9>,
    sigma_f: na::SMatrix<f32, 4, 9>,
}

impl UnscentedKalmanFilter {
    const N: f32 = 4.0;
    const ALPHA: f32 = 1e-3;
    const BETA: f32 = 2.0;
    const KAPPA: f32 = 3.0 - Self::N;
    const C: f32 = Self::ALPHA * Self::ALPHA * (Self::N + Self::KAPPA);
    const LAMBDA: f32 = Self::C - Self::N;

    pub fn new(
        x: na::Vector4<f32>,
        p: na::Matrix4<f32>,
        q: na::Matrix4<f32>,
        r: na::Matrix2<f32>,
    ) -> Self {
        let (wm, wc) = Self::sigma_weight();
        let sigma_f = Self::compute_sigma_points(x, p, 0.0, |x, _| x);
        Self {
            x,
            p,
            q,
            r,
            wm,
            wc,
            sigma_f,
        }
    }

    pub fn predict(&mut self, u: f32, fx: fn(na::Vector4<f32>, f32) -> na::Vector4<f32>) {
        self.sigma_f = Self::compute_sigma_points(self.x, self.p, u, fx);
        let (x, p) = Self::unscented_transform(&self.sigma_f, &self.wm, &self.wc, &self.q);
        self.x = x;
        self.p = p;
    }

    pub fn update(
        &mut self,
        x_obs: &na::Vector2<f32>,
        hx: fn(na::Vector4<f32>) -> na::Vector2<f32>,
    ) {
        let mut sigmas_h = na::SMatrix::<f32, 2, 9>::zeros();
        for i in 0..9 {
            sigmas_h.set_column(i, &hx(self.sigma_f.column(i).into_owned()));
        }
        let (zp, pz) = Self::unscented_transform(&sigmas_h, &self.wm, &self.wc, &self.r);
        let mut pxz = na::SMatrix::<f32, 4, 2>::zeros();
        for i in 0..9 {
            pxz += self.wc[i]
                * (self.sigma_f.column(i) - self.x)
                * (sigmas_h.column(i) - zp).transpose();
        }
        let k = pxz * pz.try_inverse().expect("Inverse fail");
        self.x += k * (x_obs - zp);
        self.p -= k * pz * k.transpose();
        // 対称性の維持
        self.p = (self.p + self.p.transpose()) / 2.0;
    }

    fn compute_sigma_points(
        x: na::Vector4<f32>,
        p: na::Matrix4<f32>,
        u: f32,
        fx: fn(na::Vector4<f32>, f32) -> na::Vector4<f32>,
    ) -> na::SMatrix<f32, 4, 9> {
        let mut sigma_f = Self::sigma_points(&x, &p);
        for i in 0..9 {
            sigma_f.set_column(i, &fx(sigma_f.column(i).into_owned(), u));
        }
        sigma_f
    }

    // 推定した状態を返す
    pub fn state(&self) -> na::Vector4<f32> {
        self.x
    }

    fn unscented_transform<const S: usize>(
        sigmas: &na::SMatrix<f32, S, 9>,
        wm: &na::SVector<f32, 9>,
        wc: &na::SVector<f32, 9>,
        cov: &na::SMatrix<f32, S, S>,
    ) -> (na::SVector<f32, S>, na::SMatrix<f32, S, S>) {
        let x = sigmas * wm;
        let y = sigmas - na::SMatrix::<f32, S, 9>::from_columns(&[x, x, x, x, x, x, x, x, x]);
        let mut tmp = na::SMatrix::<f32, S, S>::zeros();
        for i in 0..9 {
            tmp += wc[i] * y.column(i) * y.column(i).transpose();
        }
        let p = tmp + cov;
        (x, p)
    }

    fn sigma_weight() -> (na::SVector<f32, 9>, na::SVector<f32, 9>) {
        let mut wm = na::SVector::<f32, 9>::from_element(1.0 / (2.0 * Self::C));
        let mut wc = na::SVector::<f32, 9>::from_element(1.0 / (2.0 * Self::C));
        wm[0] = Self::LAMBDA / Self::C;
        wc[0] = Self::LAMBDA / Self::C + 1.0 - Self::ALPHA.powi(2) + Self::BETA;
        (wm, wc)
    }

    fn sigma_points(x: &na::Vector4<f32>, p: &na::Matrix4<f32>) -> na::SMatrix<f32, 4, 9> {
        let l = (Self::C * p).cholesky().expect("Cholesky fail").l();
        na::SMatrix::<f32, 4, 9>::from_columns(&[
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
}
