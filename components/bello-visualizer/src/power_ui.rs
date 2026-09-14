use crate::runtime::DisplaySnapshot;
use bello_hardware_client::{PowerFlags, PowerReading, VoltageCurrent};
use kiss3d::egui::{self, Color32};

fn rail(label: &str, value: &VoltageCurrent) -> String {
    format!(
        "{label}: {:.2} V / {:+.2} A",
        value.voltage_v, value.current_a
    )
}

fn flags(ui: &mut egui::Ui, label: &str, value: &PowerFlags, color: Color32) {
    let names = if value.active_names.is_empty() {
        "none".to_owned()
    } else {
        value.active_names.join(", ")
    };
    ui.colored_label(
        if value.bits == 0 {
            Color32::GRAY
        } else {
            color
        },
        format!("{label}: {names} (0x{:08x})", value.bits),
    );
}

pub(crate) fn power_ui(ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
    ui.label("Board telemetry only. No power switching or emergency clearing.");
    if snapshot.power_boards.is_empty() {
        ui.label("No power-board telemetry available. Configure a board in hardwared; the viewer never opens its UART.");
    }
    egui::ScrollArea::vertical().show(ui, |ui| {
        for board in snapshot.power_boards.values() {
            ui.heading(&board.board);
            if let Some(status) = &board.status {
                ui.label(format!("{:?}: {}", status.state(), status.detail()));
            }
            ui.label(format!(
                "Serial generation {} | invalid frames {} | unknown frames {}",
                board.connection_generation, board.invalid_frames, board.unknown_frames
            ));
            if board.samples.is_empty() {
                ui.colored_label(
                    Color32::YELLOW,
                    "Waiting for valid readings (values unavailable)",
                );
            }
            for sample in &board.samples {
                let stale = !snapshot.hardware_connected || board.is_stale(sample);
                ui.separator();
                ui.colored_label(
                    if stale {
                        Color32::YELLOW
                    } else {
                        Color32::LIGHT_GREEN
                    },
                    format!(
                        "{} | {} | age {:.2} s | seq {}",
                        sample.reading.name(),
                        if stale {
                            "STALE - last known value"
                        } else {
                            "fresh"
                        },
                        board.age(sample).as_secs_f64(),
                        sample.sequence
                    ),
                );
                match &sample.reading {
                    PowerReading::Battery {
                        voltage_v,
                        current_a,
                        soc_percent,
                    } => {
                        ui.label(format!("Battery: {voltage_v:.2} V / {current_a:+.2} A"));
                        if let Some(soc) = soc_percent {
                            ui.add(
                                egui::ProgressBar::new((*soc / 100.0) as f32)
                                    .text(format!("Board-reported charge: {soc:.1}%")),
                            );
                        } else {
                            ui.label("Charge percentage: unavailable on this profile");
                        }
                    }
                    PowerReading::Gpi { bits, second_bits } => {
                        ui.label(format!(
                            "Raw GPIO levels: 0x{bits:08x}; second word: {}",
                            second_bits
                                .map(|b| format!("0x{b:08x}"))
                                .unwrap_or_else(|| "unavailable".into())
                        ));
                    }
                    PowerReading::Iwc {
                        info,
                        warning,
                        critical,
                    } => {
                        flags(ui, "Info / emergency / charging", info, Color32::YELLOW);
                        flags(ui, "Warnings", warning, Color32::YELLOW);
                        flags(ui, "Critical", critical, Color32::LIGHT_RED);
                    }
                    PowerReading::LegacyFault { alarm, fault } => {
                        flags(ui, "Alarm", alarm, Color32::YELLOW);
                        flags(ui, "Fault", fault, Color32::LIGHT_RED);
                    }
                    PowerReading::Environment {
                        temperature_c,
                        pressure_pa,
                    } => {
                        ui.label(format!("Board: {temperature_c:.2} C | {pressure_pa:.0} Pa"));
                    }
                    PowerReading::LocalRails {
                        rail_20v0,
                        rail_5v0,
                        rail_3v3,
                    } => {
                        for (name, v) in
                            [("20 V", rail_20v0), ("5 V", rail_5v0), ("3.3 V", rail_3v3)]
                        {
                            ui.label(rail(name, v));
                        }
                    }
                    PowerReading::MotorRail(v) => {
                        ui.label(rail("Motor supply", v));
                    }
                    PowerReading::TickCount(v) => {
                        ui.label(format!("MCU tick: {v} (raw counter)"));
                    }
                    PowerReading::Encoder {
                        left_rpm,
                        right_rpm,
                    } => {
                        ui.label(format!(
                            "Legacy wheel encoders: {left_rpm:.2} / {right_rpm:.2} RPM"
                        ));
                    }
                }
            }
        }
    });
}
