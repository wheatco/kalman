extern crate nalgebra as na;

use na::{DMatrix, DVector, new_identity, Inverse, Transpose};

struct KalmanFilter {
    state: DVector<f64>,
    cov: DMatrix<f64>, // variance-covariance matrix
    state_trans: DMatrix<f64>, // transforms old states into new states
    state_trans_error: DVector<f64>, // transforms old states into new states
    control_trans: DMatrix<f64>, // transforms controls into new state deltas
    sensor_trans: DMatrix<f64>, // transforms a state into sensor data
}

impl KalmanFilter {
    fn new(state: &Vec<f64>,
           cov: &Vec<f64>,
           state_trans: &Vec<f64>,
           state_trans_error: &Vec<f64>,
           control_trans: &Vec<f64>,
           sensor_trans: &Vec<f64>) -> KalmanFilter {
        let state_count = state.len();
        let control_count = control_trans.len() / state_count;
        let sensor_count = sensor_trans.len() / state_count;
        KalmanFilter {
            state: DVector::from_slice(state_count, state),
            cov: DMatrix::from_col_vector(state_count, state_count, cov),
            state_trans: DMatrix::from_col_vector(state_count, state_count, state_trans),
            state_trans_error: DVector::from_slice(state_count, state_trans_error),
            control_trans: DMatrix::from_col_vector(control_count, state_count, control_trans),
            sensor_trans: DMatrix::from_col_vector(state_count, sensor_count, sensor_trans),
        }
    }
    fn update(&mut self, control_data_raw: &Vec<f64>, sensor_data_raw: &Vec<f64>) {
        let control_data = DVector::from_slice(control_data_raw.len(), control_data_raw);
        let sensor_data = DVector::from_slice(sensor_data_raw.len(), sensor_data_raw);

        let state_bar = &self.state_trans * &self.state + &self.sensor_trans * &sensor_data;
        let cov_bar = &self.state_trans * &self.cov * &self.state_trans.transpose()/*TODO + self.state_trans_error*/;
        let kalman_gain = &cov_bar * &self.sensor_trans.transpose() * (&self.sensor_trans * &cov_bar * &self.sensor_trans.transpose()).inverse().unwrap();
        self.state = state_bar.clone() + &kalman_gain * (sensor_data - &self.sensor_trans * state_bar.clone());
        self.cov = (new_identity::<DMatrix<f64>>(state_bar.len()) - &kalman_gain * &self.state_trans) * cov_bar;
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
