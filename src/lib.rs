extern crate nalgebra as na;

use na::{DMatrix, DVector, new_identity, Inverse, Transpose};

struct KalmanFilter {
    state: DVector<f64>,
    cov: DMatrix<f64>, // variance-covariance matrix
    a: DMatrix<f64>, // transforms old states into new states
    a_err: DVector<f64>, // transforms old states into new states
    b: DMatrix<f64>, // transforms controls into new state deltas
    c: DMatrix<f64>, // transforms a state into sensor data
}

impl KalmanFilter {
    fn new(state: &Vec<f64>,
           cov: &Vec<f64>,
           a: &Vec<f64>,
           a_err: &Vec<f64>,
           b: &Vec<f64>,
           c: &Vec<f64>) -> KalmanFilter {
        let state_count = state.len();
        let control_count = b.len() / state_count;
        let sensor_count = c.len() / state_count;
        KalmanFilter {
            state: DVector::from_slice(state_count, state),
            cov: DMatrix::from_col_vector(state_count, state_count, cov),
            a: DMatrix::from_col_vector(state_count, state_count, a),
            a_err: DVector::from_slice(state_count, a_err),
            b: DMatrix::from_col_vector(control_count, state_count, b),
            c: DMatrix::from_col_vector(state_count, sensor_count, c),
        }
    }
    fn update(&mut self, control_data_raw: &Vec<f64>, sensor_data_raw: &Vec<f64>) {
        let control_data = DVector::from_slice(control_data_raw.len(), control_data_raw);
        let sensor_data = DVector::from_slice(sensor_data_raw.len(), sensor_data_raw);

        let state_bar = &self.a * &self.state + &self.b * &control_data;
        let cov_bar = &self.a * &self.cov * &self.a.transpose()/*TODO + self.a_err*/;
        let kalman_gain = &cov_bar * &self.c.transpose() * (&self.c * &cov_bar * &self.c.transpose()).inverse().unwrap();
        self.state = state_bar.clone() + &kalman_gain * (sensor_data - &self.c * state_bar.clone());
        self.cov = (new_identity::<DMatrix<f64>>(state_bar.len()) - &kalman_gain * &self.a) * cov_bar;
    }
}

#[cfg(test)]
mod test {
    use KalmanFilter;
    #[test]
    fn it_works() {
        KalmanFilter::new(&vec![0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0]);
        assert!(true);
    }
}
