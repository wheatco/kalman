extern crate nalgebra as na;

#[cfg(test)]
mod test {
    use na::{DMat};
    #[test]
    fn it_works() {
        let m1 = &DMat::from_col_vec(3,3,&vec![1,2,3,4,5,6,7,8,9]);
        let m2 = &DMat::from_col_vec(3,3,&vec![1,2,3,4,5,6,7,8,9]);
        let m3 = m1 * m2;
        println!("{:?} * {:?} = {:?}", m1, m2, m3);
        assert!(true);
    }
}
