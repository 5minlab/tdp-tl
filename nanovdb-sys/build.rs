fn main() {
    cc::Build::new()
        .cpp(true)
        .file("ffi/wrapper.cpp")
        .include("/opt/local/include")
        .flag_if_supported("-std=c++17")
        .compile("nanovdb_ffi");

    let bindings = bindgen::Builder::default()
        .header("ffi/wrapper.cpp")
        .clang_arg("-xc++")
        .clang_arg("-I/opt/local/include")
        .clang_arg("-std=c++17")
        .allowlist_function("grid_new")
        .allowlist_function("grid_delete")
        .allowlist_function("get_value")
        .allowlist_function("set_value")
        .allowlist_function("add_value")
        .allowlist_function("iter_init")
        .allowlist_function("iter_get0")
        .allowlist_function("iter_get")
        .allowlist_function("iter2_init")
        .allowlist_function("iter2_get0")
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(std::path::Path::new(&std::env::var("OUT_DIR").unwrap()).join("bindings.rs"))
        .unwrap();

    println!("cargo:rerun-if-changed=ffi/wrapper.cpp");
}
