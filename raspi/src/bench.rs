
use std::time::Instant;
mod blas;
mod ode;

fn main() {
    println!("Benchmarking BLAS...");
    let start = Instant::now();
    for _ in 0..1000 {
        blas::run_blas_example();
    }
    println!("BLAS elapsed: {:?}", start.elapsed());

    println!("Benchmarking ODE solver...");
    let start = Instant::now();
    for _ in 0..10000 {
        ode::run_ode_example();
    }
    println!("ODE elapsed: {:?}", start.elapsed());
}


/*
## Run Benchmark
On Raspberry Pi:
```bash
./my-numerics-bench
```

On Apple Silicon:
```bash
cargo run --release --bin ben
```
*/

