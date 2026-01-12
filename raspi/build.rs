
fn main() {
    println!("cargo:rustc-link-lib=static=openblas");
    println!("cargo:rustc-link-lib=static=ode_solver");
    println!("cargo:rustc-link-search=native=/usr/local/openblas/lib");
    println!("cargo:rustc-link-search=native=c_lib");

    let bindings = bindgen::Builder::default()
        .header("c_lib/ode_solver.h")
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file("src/bindings.rs")
        .expect("Couldn't write bindings!");
}
