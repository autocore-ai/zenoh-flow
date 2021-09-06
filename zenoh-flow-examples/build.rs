use std::env;
use std::path::{Path, PathBuf};
fn main() {
    let pwd_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let path = Path::new(&*pwd_dir).join("lib");
    println!("cargo:rustc-env=LD_LIBRARY_PATH={}", path.to_str().unwrap());
    println!("cargo:rustc-link-search=native={}", path.to_str().unwrap());
    println!("cargo:rustc-link-lib=dylib=replay_planner");

    // cxx_build::bridge("examples/replay-planner-operator.rs")
    // .flag_if_supported("-std=c++14")
    // .compile("random-source");
    
    let path1 = PathBuf::from("examples/replay_planner/include/");
    let path2 = PathBuf::from("/home/shenmintao/adehome/AutowareAuto/install/autoware_auto_msgs/include");
    let path3 = PathBuf::from("/opt/ros/foxy/include/");
    let path4 = PathBuf::from("/home/shenmintao/adehome/AutowareAuto/install/motion_common/include/");
    let path5 = PathBuf::from("/home/shenmintao/adehome/AutowareAuto/install/time_utils/include/");
    let path6 = PathBuf::from("/home/shenmintao/adehome/AutowareAuto/install/autoware_auto_common/include/");
    
    let mut b = autocxx_build::build("examples/replay-planner-operator.rs", &[&path1, &path2,&path3,&path4,&path5,&path6], &[]).unwrap();
    b.flag_if_supported("-std=c++14").compile("replay-plannner-operator");
}
