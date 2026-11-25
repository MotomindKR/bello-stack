use std::{env, fs, path::PathBuf};

use ring::digest::{SHA256, digest};
use serde::Deserialize;

#[derive(Deserialize)]
struct Release {
    model: String,
    version: String,
    filename: String,
    sha256: String,
}

fn main() {
    let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("firmware");
    println!("cargo:rerun-if-changed=firmware");
    let catalog = fs::read(directory.join("catalog.json")).expect("read firmware catalog");
    let releases: Vec<Release> = serde_json::from_slice(&catalog).expect("parse firmware catalog");
    let mut table = String::from("&[\n");
    for release in releases {
        let path = directory.join(release.filename);
        let bytes = fs::read(&path).expect("read bundled firmware");
        assert!(!bytes.is_empty() && bytes.len() <= 0x80000);
        let hash: String = digest(&SHA256, &bytes)
            .as_ref()
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        assert_eq!(
            hash, release.sha256,
            "firmware checksum mismatch for {}",
            release.model
        );
        table.push_str(&format!(
            "({:?}, {:?}, include_bytes!({path:?})),\n",
            release.model.replace('-', "_"),
            release.version
        ));
    }
    table.push(']');
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(output.join("firmware.rs"), table).unwrap();
}
