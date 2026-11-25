use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::PathBuf;

use serde::Deserialize;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Catalog {
    models: BTreeMap<String, Limits>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Limits {
    position: [f64; 2],
    velocity: [f64; 2],
    torque: [f64; 2],
    kp: [f64; 2],
    kd: [f64; 2],
    #[serde(default)]
    hardware: Option<Hardware>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Hardware {
    no_load_speed_rpm: f64,
    rated_voltage: f64,
    peak_phase_current: f64,
}

fn main() {
    let catalog_path =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("Cargo sets CARGO_MANIFEST_DIR"))
            .join("../../descriptions/motors/models.yaml");
    println!("cargo:rerun-if-changed={}", catalog_path.display());
    let text = fs::read_to_string(&catalog_path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", catalog_path.display()));
    let catalog: Catalog = serde_yaml::from_str(&text)
        .unwrap_or_else(|error| panic!("invalid {}: {error}", catalog_path.display()));
    assert!(
        !catalog.models.is_empty(),
        "RobStride model catalog is empty"
    );

    let first = catalog
        .models
        .values()
        .next()
        .expect("catalog is non-empty");
    let position = first.position;
    for (name, limits) in &catalog.models {
        assert_eq!(
            limits.position, position,
            "{name}.position differs from the shared Type-1 encoding range"
        );
    }

    let mut generated = format!(
        "pub const CONTROL_POSITION_MIN: f64 = {:?};\npub const CONTROL_POSITION_MAX: f64 = {:?};\npub const CONTROL_POSITION_SPAN: f64 = CONTROL_POSITION_MAX - CONTROL_POSITION_MIN;\n\npub const MODEL_NAMES: &[&str] = &[\n",
        position[0], position[1]
    );
    for name in catalog.models.keys() {
        generated.push_str(&format!("    {name:?},\n"));
    }
    generated.push_str("];\n\npub fn model_limits(model: &str) -> Option<ModelLimits> {\n");
    generated.push_str("    match model {\n");
    for (name, limits) in &catalog.models {
        validate(name, limits);
        let hardware = limits.hardware.as_ref().map_or_else(
            || "None".to_owned(),
            |hardware| {
                format!(
                    "Some(ModelHardware {{ no_load_speed_rpm: {:?}, rated_voltage: {:?}, peak_phase_current: {:?} }})",
                    hardware.no_load_speed_rpm,
                    hardware.rated_voltage,
                    hardware.peak_phase_current,
                )
            },
        );
        generated.push_str(&format!(
            "        {name:?} => Some(ModelLimits {{ position_min: {:?}, position_max: {:?}, velocity_min: {:?}, velocity_max: {:?}, torque_min: {:?}, torque_max: {:?}, kp_min: {:?}, kp_max: {:?}, kd_min: {:?}, kd_max: {:?}, hardware: {hardware} }}),\n",
            limits.position[0],
            limits.position[1],
            limits.velocity[0],
            limits.velocity[1],
            limits.torque[0],
            limits.torque[1],
            limits.kp[0],
            limits.kp[1],
            limits.kd[0],
            limits.kd[1],
        ));
    }
    generated.push_str("        _ => None,\n    }\n}\n");

    let output =
        PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set")).join("model_catalog.rs");
    fs::write(output, generated).expect("write generated model catalog");
}

fn validate(name: &str, limits: &Limits) {
    assert!(
        !name.trim().is_empty(),
        "RobStride model names must be non-empty"
    );
    for (field, range, non_negative) in [
        ("position", limits.position, false),
        ("velocity", limits.velocity, false),
        ("torque", limits.torque, false),
        ("kp", limits.kp, true),
        ("kd", limits.kd, true),
    ] {
        assert!(
            range.iter().all(|value| value.is_finite()) && range[0] < range[1],
            "{name}.{field} must be a finite increasing range"
        );
        assert!(
            !non_negative || range[0] >= 0.0,
            "{name}.{field} must be non-negative"
        );
    }
    assert_eq!(
        limits.kp[0], 0.0,
        "{name}.kp must support an exactly zero safety command"
    );
    assert_eq!(
        limits.kd[0], 0.0,
        "{name}.kd must support disabled or scaled-to-zero damping"
    );
    if let Some(hardware) = &limits.hardware {
        for (field, value) in [
            ("no_load_speed_rpm", hardware.no_load_speed_rpm),
            ("rated_voltage", hardware.rated_voltage),
            ("peak_phase_current", hardware.peak_phase_current),
        ] {
            assert!(
                value.is_finite() && value > 0.0,
                "{name}.hardware.{field} must be finite and positive"
            );
        }
    }
}
