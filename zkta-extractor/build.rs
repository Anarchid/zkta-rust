use std::env;
use std::path::PathBuf;

fn main() {
    // Engine source root — override with RECOIL_SRC env var
    let engine_src = env::var("RECOIL_SRC")
        .unwrap_or_else(|_| {
            let home = env::var("HOME").expect("HOME not set");
            format!("{}/Programs/RecoilEngine", home)
        });

    let engine_path = PathBuf::from(&engine_src);
    assert!(
        engine_path.join("rts/ExternalAI/Interface/SSkirmishAICallback.h").exists(),
        "Engine source not found at {}. Set RECOIL_SRC env var.",
        engine_src
    );

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=RECOIL_SRC");

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}", engine_src))
        .clang_arg(format!("-I{}/rts", engine_src))
        .allowlist_type("SSkirmishAICallback")
        .derive_debug(false)
        .derive_default(false)
        .generate()
        .expect("Failed to generate bindings");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_dir.join("bindings.rs"))
        .expect("Failed to write bindings");
}
