extern crate nalgebra as na;

use na::{DMat, DVec};

struct KalmanFilter {
    state: DVec<f64>,
    cov: DMat<f64>,
    state_trans: DMat<f64>, // transforms old states into new states
    control_trans: DMat<f64>, // transforms controls into new state deltas
    sensor_trans: DMat<f64>, // transforms a state into sensor data
}

impl KalmanFilter {
    fn new(state: &Vec<f64>,
           cov: &Vec<f64>,
           state_trans: &Vec<f64>,
           control_trans: &Vec<f64>,
           sensor_trans: &Vec<f64>) -> KalmanFilter {
        let state_count = state.len();
        let control_count = control_trans.len() / state_count;
        let sensor_count = sensor_trans.len() / state_count;
        KalmanFilter {
            state: DVec::from_slice(state_count, state),
            cov: DMat::from_col_vec(state_count, state_count, cov),
            state_trans: DMat::from_col_vec(state_count, state_count, state_trans),
            control_trans: DMat::from_col_vec(control_count, state_count, control_trans),
            sensor_trans: DMat::from_col_vec(state_count, sensor_count, sensor_trans),
        }
    }
    // fn update(&mut self, control_data: DVec<f64>, sensor_data: DVec<f64>) {

    // }
}

#[cfg(test)]
mod test {
    use KalmanFilter;
    #[test]
    fn it_works() {
        KalmanFilter::new(&vec![0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0]);
        assert!(true);
    }
}
