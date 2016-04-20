extern crate nalgebra as na;

use na::{DMatrix, DVector, new_identity, Inverse, Transpose};

// struct KalmanFilter {
//     // means of world state variables, and the current covariance matrix
//     state: DVector<f64>,
//     cov: DMatrix<f64>,
//     // control + old state -> state and error covariance matrix
//     update_trans: Box<Fn(&DVector<f64>, &DVector<f64>) -> DVector<f64>>,
//     update_cov: DMatrix<f64>,
//     // state to sensor readings, and error covariance matrix
//     sensor_trans: Box<Fn(&DVector<f64>) -> DVector<f64>>,
//     sensor_cov: DMatrix<f64>
// }

fn jacobian(vals: &DVector<f64>, f: &Fn(&DVector<f64>) -> DVector<f64>) -> DMatrix<f64> {
    let input_len = vals.len();
    let output_len = f(vals).len();

    let cols: Vec<DVector<f64>> = (0..input_len).map(|i| {
        let mut v = vals.clone();
        let delta: f64 = 1e-8;
        v[i] += delta; // TODO choose a good threshold
        println!("1: {:?}", &v);
        let output1 = f(&v) - f(&vals);
        let output2 = output1.clone() / delta;
        println!("2: {:?}", &output1);
        println!("3: {:?}", &output2);
        output2
    }).collect();

    DMatrix::from_fn(output_len, input_len, |o, i| cols[i][o])
}

// impl KalmanFilter {
//     fn new(initial_state: DVector<f64>,
//            initial_cov: DMatrix<f64>,
//            update_trans: Box<Fn(DVector<f64>, DVector<f64>) -> DVector<f64>>,
//            update_cov: DMatrix<f64>,
//            sensor_trans: Box<Fn(DVector<f64>) -> DVector<f64>>,
//            sensor_cov: DMatrix<f64>) {
//         KalmanFilter {
//             state: initial_state,
//             cov: initial_cov,
//             update_trans: update_trans,
//             update_cov: update_cov,
//             sensor_trans: sensor_trans,
//             sensor_cov: sensor_cov,
//         }
//     }
//     fn update(&mut self, control_data_raw: &Vec<f64>, sensor_data_raw: &Vec<f64>) {
//         let control_data = DVector::from_slice(control_data_raw.len(), control_data_raw);
//         let sensor_data = DVector::from_slice(sensor_data_raw.len(), sensor_data_raw);

//         let state_bar = &self.update_trans(&self.control_data, &self.update_cov);
//         let cov_bar = &self.a * &self.cov * &self.a.transpose() + &self.a_err;
//         let kalman_gain = &cov_bar * &self.c.transpose() * (&self.c * &cov_bar * &self.c.transpose()).inverse().unwrap();
//         self.state = state_bar.clone() + &kalman_gain * (sensor_data - &self.c * state_bar.clone());
//         self.cov = (new_identity::<DMatrix<f64>>(state_bar.len()) - &kalman_gain * &self.a) * cov_bar;
//     }
// }

#[cfg(test)]
mod test {
    use jacobian;
    use na::{DMatrix, DVector};
    #[test]
    fn it_works() {
        // KalmanFilter::new(&vec![0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0], &vec![0.0,0.0,0.0,0.0]);
        let vals = DVector::from_slice(3, &vec![0.0,1.0,2.0]);
        let j = jacobian(&vals, &|x| x.clone() * DMatrix::from_row_vector(3, 2, &vec![1.0,2.0,3.0,4.0,5.0,6.0]));
        println!("{:?}", j);
        assert!(false);
    }
}
