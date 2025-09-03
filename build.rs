use std::env;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    cbindgen::Builder::new()
        .with_crate(crate_dir)
        // TODO: This is stupid and easy to forgot to add things here. Can we auto detect those?
        .include_item("Tdp1Bindings")
        .with_pragma_once(true)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("./Bindings.h");
}
