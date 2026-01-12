
include!("bindings.rs");

pub fn run_ode_example() {
    unsafe {
        let result = solve_ode(0.0, 10.0, 0.1);
        println!("ODE solver result: {}", result);
    }
}
