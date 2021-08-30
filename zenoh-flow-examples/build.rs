use std::env;
use std::path::{Path};

fn main() {
    let pwd_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let path = Path::new(&*pwd_dir).join("lib");
    println!("cargo:rustc-env=LD_LIBRARY_PATH={}", path.to_str().unwrap());
    println!("cargo:rustc-link-search=native={}", path.to_str().unwrap());
    println!("cargo:rustc-link-lib=dylib=replay_planner");
}
