
use libc::{c_double, c_int};

extern "C" {
    fn cblas_dgemm(
        Order: c_int,
        TransA: c_int,
        TransB: c_int,
        M: c_int,
        N: c_int,
        K: c_int,
        alpha: c_double,
        A: *const c_double,
        lda: c_int,
        B: *const c_double,
        ldb: c_int,
        beta: c_double,
        C: *mut c_double,
        ldc: c_int,
    );
}

pub fn run_blas_example() {
    let m = 2;
    let n = 2;
    let k = 2;
    let a = [1.0, 2.0, 3.0, 4.0];
    let b = [5.0, 6.0, 7.0, 8.0];
    let mut c = [0.0; 4];

    unsafe {
        cblas_dgemm(101, 111, 111, m, n, k, 1.0, a.as_ptr(), m, b.as_ptr(), k, 0.0, c.as_mut_ptr(), m);
    }

    println!("Result: {:?}", c);
}
