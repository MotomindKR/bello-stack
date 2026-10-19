use crate::power_ui::power_ui;
use std::time::Duration;

use bello_hardware_client::{DeviceState, GroupState};
use kiss3d::egui::{self, Color32, RichText};

use crate::runtime::{
    ControlStatus, DiagnosticEvent, DiagnosticSeverity, DisplayDeviceStatus, DisplaySnapshot,
};

const DISPLAY_ONLY_STALE_AFTER: Duration = Duration::from_millis(200);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum DiagnosticsTab {
    #[default]
    Overview,
    Devices,
    Motors,
    Groups,
    Imus,
    Power,
    Events,
}

#[derive(Default)]
pub struct DiagnosticsUi {
    tab: DiagnosticsTab,
    event_search: String,
    show_info: bool,
    show_warnings: bool,
    show_errors: bool,
    paused_events: Option<Vec<DiagnosticEvent>>,
    cleared_through: u64,
}

impl DiagnosticsUi {
    pub fn new() -> Self {
        Self {
            show_info: true,
            show_warnings: true,
            show_errors: true,
            ..Self::default()
        }
    }

    pub fn show(&mut self, context: &egui::Context, snapshot: &DisplaySnapshot) {
        egui::Window::new("Bello diagnostics")
            .default_pos([8.0, 8.0])
            .default_size([900.0, 580.0])
            .resizable(true)
            .show(context, |ui| {
                self.connection_header(ui, snapshot);
                ui.separator();
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Overview, "Overview");
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Devices, "Devices");
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Motors, "Motors");
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Groups, "Groups");
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Imus, "IMUs");
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Power, "Power");
                    ui.selectable_value(&mut self.tab, DiagnosticsTab::Events, "Events");
                });
                ui.separator();
                match self.tab {
                    DiagnosticsTab::Overview => overview_ui(ui, snapshot),
                    DiagnosticsTab::Devices => devices_ui(ui, snapshot),
                    DiagnosticsTab::Motors => motors_ui(ui, snapshot),
                    DiagnosticsTab::Groups => groups_ui(ui, snapshot),
                    DiagnosticsTab::Imus => imus_ui(ui, snapshot),
                    DiagnosticsTab::Power => power_ui(ui, snapshot),
                    DiagnosticsTab::Events => self.events_ui(ui, snapshot),
                }
            });
    }

    fn connection_header(&self, ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
        ui.horizontal_wrapped(|ui| {
            let (label, color) = if snapshot.hardware_connected {
                ("hardware connected", Color32::LIGHT_GREEN)
            } else {
                ("hardware disconnected", Color32::LIGHT_RED)
            };
            ui.colored_label(color, RichText::new(label).strong());
            ui.separator();
            if snapshot.pose_available {
                ui.colored_label(Color32::LIGHT_GREEN, "root pose available");
            } else {
                ui.colored_label(Color32::YELLOW, "root pose frozen");
            }
            ui.separator();
            ui.label(format!("display snapshot {}", snapshot.generation));
            if snapshot.telemetry_dropped_total > 0 {
                ui.separator();
                ui.colored_label(
                    Color32::LIGHT_RED,
                    format!(
                        "{} telemetry events dropped",
                        snapshot.telemetry_dropped_total
                    ),
                );
            }
        });
        if let Some(warning) = &snapshot.hardware_warning {
            ui.colored_label(Color32::LIGHT_RED, warning);
        }
        if let Some(warning) = &snapshot.pose_warning {
            ui.colored_label(Color32::YELLOW, warning);
        }
    }

    fn events_ui(&mut self, ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Filter:");
            ui.text_edit_singleline(&mut self.event_search);
            ui.checkbox(&mut self.show_info, "Info");
            ui.checkbox(&mut self.show_warnings, "Warnings");
            ui.checkbox(&mut self.show_errors, "Errors");
            let paused = self.paused_events.is_some();
            if ui.button(if paused { "Resume" } else { "Pause" }).clicked() {
                self.paused_events = if paused {
                    None
                } else {
                    Some(snapshot.events.clone())
                };
            }
            if ui.button("Clear view").clicked() {
                self.cleared_through = snapshot.events.last().map_or(0, |event| event.id);
                if let Some(events) = &mut self.paused_events {
                    events.clear();
                }
            }
        });
        ui.weak(format!(
            "{} retained · {} evicted locally · newest first",
            snapshot.events.len(),
            snapshot.events_evicted
        ));
        ui.separator();

        let events = self.paused_events.as_deref().unwrap_or(&snapshot.events);
        let search = self.event_search.to_lowercase();
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::Grid::new("diagnostic_events")
                .striped(true)
                .num_columns(5)
                .show(ui, |ui| {
                    header(ui, "time (s)");
                    header(ui, "level");
                    header(ui, "category");
                    header(ui, "component");
                    header(ui, "detail");
                    ui.end_row();
                    for event in events.iter().rev().filter(|event| {
                        event.id > self.cleared_through
                            && self.severity_visible(event.severity)
                            && (search.is_empty()
                                || event.component.to_lowercase().contains(&search)
                                || event.detail.to_lowercase().contains(&search)
                                || event.category.contains(&search))
                    }) {
                        ui.monospace(source_time(event.monotonic_ns));
                        ui.colored_label(
                            severity_color(event.severity),
                            severity_name(event.severity),
                        );
                        ui.label(event.category);
                        ui.monospace(&event.component);
                        let detail = if event.repeat_count > 1 {
                            format!("{} ×{}", event.detail, event.repeat_count)
                        } else {
                            event.detail.clone()
                        };
                        ui.label(detail);
                        ui.end_row();
                    }
                });
        });
    }

    fn severity_visible(&self, severity: DiagnosticSeverity) -> bool {
        match severity {
            DiagnosticSeverity::Info => self.show_info,
            DiagnosticSeverity::Warning => self.show_warnings,
            DiagnosticSeverity::Error => self.show_errors,
        }
    }
}

fn overview_ui(ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
    let device_faults = snapshot
        .buses
        .values()
        .filter_map(|device| device.status.as_ref())
        .chain(
            snapshot
                .serial_devices
                .values()
                .filter_map(|device| device.status.as_ref()),
        )
        .chain(
            snapshot
                .motors
                .values()
                .filter_map(|device| device.status.as_ref()),
        )
        .chain(
            snapshot
                .imus
                .values()
                .filter_map(|device| device.status.as_ref()),
        )
        .filter(|status| status.state == DeviceState::Fault)
        .count();
    let group_faults = snapshot
        .groups
        .values()
        .filter(|group| group.state == GroupState::Fault)
        .count();
    let stale_motors = snapshot
        .motors
        .values()
        .filter(|motor| {
            !motor.feedback_available
                || !motor.feedback_current_connection
                || motor.age > motor.stale_after
        })
        .count();
    let stale_imus = snapshot
        .imus
        .values()
        .filter(|imu| {
            !imu.sample_available
                || !imu.sample_current_connection
                || imu.sample_age > DISPLAY_ONLY_STALE_AFTER
        })
        .count();
    let maximum_temperature = snapshot
        .motors
        .values()
        .filter(|motor| motor.feedback_available && motor.temperature.is_finite())
        .map(|motor| motor.temperature)
        .reduce(f64::max);

    ui.horizontal_wrapped(|ui| {
        summary(ui, "CAN buses", snapshot.buses.len(), Color32::LIGHT_BLUE);
        summary(
            ui,
            "Serial",
            snapshot.serial_devices.len(),
            Color32::LIGHT_BLUE,
        );
        summary(ui, "Groups", snapshot.groups.len(), Color32::LIGHT_BLUE);
        summary(ui, "Motors", snapshot.motors.len(), Color32::LIGHT_BLUE);
        summary(ui, "IMUs", snapshot.imus.len(), Color32::LIGHT_BLUE);
        summary(
            ui,
            "Faults",
            device_faults + group_faults,
            if device_faults + group_faults == 0 {
                Color32::LIGHT_GREEN
            } else {
                Color32::LIGHT_RED
            },
        );
        summary(
            ui,
            "Stale inputs",
            stale_motors + stale_imus,
            if stale_motors + stale_imus == 0 {
                Color32::LIGHT_GREEN
            } else {
                Color32::YELLOW
            },
        );
    });

    ui.add_space(8.0);
    ui.heading("Live summary");
    egui::Grid::new("diagnostic_overview")
        .striped(true)
        .show(ui, |ui| {
            ui.label("Highest motor temperature");
            ui.monospace(
                maximum_temperature.map_or_else(|| "—".into(), |value| format!("{value:.1} °C")),
            );
            ui.end_row();
            ui.label("Telemetry gaps");
            ui.monospace(snapshot.telemetry_dropped_total.to_string());
            ui.end_row();
            ui.label("Diagnostic events retained");
            ui.monospace(snapshot.events.len().to_string());
            ui.end_row();
            ui.label("Diagnostic events evicted");
            ui.monospace(snapshot.events_evicted.to_string());
            ui.end_row();
        });

    ui.add_space(8.0);
    ui.heading("Groups");
    for group in snapshot.groups.values() {
        ui.horizontal_wrapped(|ui| {
            ui.monospace(&group.group);
            group_state_cell(ui, group.state);
            if !group.detail.is_empty() {
                ui.colored_label(Color32::LIGHT_RED, &group.detail);
            }
        });
    }
}

fn devices_ui(ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
    egui::ScrollArea::both().show(ui, |ui| {
        ui.heading("CAN buses");
        egui::Grid::new("diagnostic_buses")
            .striped(true)
            .show(ui, |ui| {
                for title in [
                    "name",
                    "transport",
                    "host ID",
                    "state",
                    "detail",
                    "transitions",
                ] {
                    header(ui, title);
                }
                ui.end_row();
                for bus in snapshot.buses.values() {
                    ui.monospace(&bus.name);
                    ui.monospace(&bus.transport);
                    ui.monospace(bus.host_id.to_string());
                    status_cells(ui, bus.status.as_ref());
                    ui.end_row();
                }
            });

        ui.add_space(10.0);
        ui.heading("Serial devices");
        egui::Grid::new("diagnostic_serial")
            .striped(true)
            .show(ui, |ui| {
                for title in ["name", "port", "baud", "state", "detail", "transitions"] {
                    header(ui, title);
                }
                ui.end_row();
                for serial in snapshot.serial_devices.values() {
                    ui.monospace(&serial.name);
                    ui.monospace(&serial.port);
                    ui.monospace(serial.baud_rate.to_string());
                    status_cells(ui, serial.status.as_ref());
                    ui.end_row();
                }
            });

        ui.add_space(10.0);
        ui.heading("Lifecycle telemetry");
        egui::Grid::new("diagnostic_health")
            .striped(true)
            .show(ui, |ui| {
                for title in [
                    "component",
                    "time (s)",
                    "state",
                    "detail",
                    "transitions",
                    "dropped",
                ] {
                    header(ui, title);
                }
                ui.end_row();
                for health in snapshot.health.values() {
                    ui.monospace(&health.component);
                    ui.monospace(source_time(Some(health.monotonic_ns)));
                    status_cells(ui, health.status.as_ref());
                    ui.monospace(health.dropped_events.to_string());
                    ui.end_row();
                }
            });
    });
}

fn motors_ui(ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
    ui.weak("Temperature thresholds are enforced by hardwared but are not published; values are shown without invented limits.");
    ui.weak("Feedback-age color uses the group's configured control staleness limit; monitor-only groups use 200 ms.");
    egui::ScrollArea::both().show(ui, |ui| {
        egui::Grid::new("diagnostic_motors")
            .striped(true)
            .show(ui, |ui| {
                for title in [
                    "motor",
                    "coordinate",
                    "group",
                    "bus:ID",
                    "model",
                    "dir",
                    "state",
                    "detail",
                    "transitions",
                    "stream",
                    "q",
                    "dq",
                    "torque",
                    "temp",
                    "fault bits",
                    "mode",
                    "age",
                    "time (s)",
                ] {
                    header(ui, title);
                }
                ui.end_row();
                for motor in snapshot.motors.values() {
                    ui.monospace(&motor.motor);
                    ui.monospace(&motor.coordinate);
                    ui.monospace(&motor.group);
                    ui.monospace(format!("{}:{}", motor.bus, motor.id));
                    ui.monospace(&motor.model);
                    ui.monospace(format!("{:+}", motor.direction));
                    status_cells(ui, motor.status.as_ref());
                    stream_cell(
                        ui,
                        motor.feedback_available,
                        motor.feedback_current_connection,
                    );
                    if motor.feedback_available {
                        ui.monospace(format!("{:.5}", motor.position));
                        ui.monospace(format!("{:.5}", motor.velocity));
                        ui.monospace(format!("{:.4}", motor.effort));
                        ui.monospace(format!("{:.1} °C", motor.temperature));
                        ui.colored_label(
                            if motor.fault_bits == 0 {
                                Color32::GRAY
                            } else {
                                Color32::LIGHT_RED
                            },
                            format!("0x{:04x}", motor.fault_bits),
                        );
                        ui.monospace(format!("0x{:02x}", motor.mode_status));
                        age_cell(ui, motor.age, true, motor.stale_after);
                        ui.monospace(source_time(motor.monotonic_ns));
                    } else {
                        for _ in 0..8 {
                            ui.weak("—");
                        }
                    }
                    ui.end_row();
                }
            });
    });
}

fn groups_ui(ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
    ui.weak("Group telemetry age uses a display-only 200 ms warning threshold.");
    egui::ScrollArea::both().show(ui, |ui| {
        egui::Grid::new("diagnostic_groups")
            .striped(true)
            .show(ui, |ui| {
                for title in [
                    "group",
                    "daemon state",
                    "visualizer",
                    "stream",
                    "rate",
                    "fault cause",
                    "fault detail",
                    "commands",
                    "tick",
                    "jitter",
                    "age",
                    "time (s)",
                ] {
                    header(ui, title);
                }
                ui.end_row();
                for group in snapshot.groups.values() {
                    ui.monospace(&group.group);
                    group_state_cell(ui, group.state);
                    ui.monospace(format!(
                        "{:?}",
                        snapshot
                            .control
                            .get(&group.group)
                            .unwrap_or(&ControlStatus::Monitoring)
                    ));
                    stream_cell(
                        ui,
                        group.telemetry_available,
                        group.telemetry_current_connection,
                    );
                    ui.monospace(format!("{:.1} Hz", group.control_rate_hz));
                    ui.monospace(format!("{:?}", group.cause));
                    if group.detail.is_empty() {
                        ui.weak("—");
                    } else {
                        ui.label(&group.detail);
                    }
                    if group.telemetry_available {
                        ui.monospace(group.command_count.to_string());
                        ui.monospace(group.control_tick.to_string());
                        ui.monospace(format!("{:+} µs", group.jitter_us));
                        age_cell(
                            ui,
                            group.age.unwrap_or(Duration::MAX),
                            group.age.is_some(),
                            DISPLAY_ONLY_STALE_AFTER,
                        );
                        ui.monospace(source_time(group.monotonic_ns));
                    } else {
                        for _ in 0..5 {
                            ui.weak("—");
                        }
                    }
                    ui.end_row();
                }
            });
    });
}

fn imus_ui(ui: &mut egui::Ui, snapshot: &DisplaySnapshot) {
    ui.weak("IMU sample age uses a display-only 200 ms warning threshold.");
    egui::ScrollArea::both().show(ui, |ui| {
        egui::Grid::new("diagnostic_imus")
            .striped(true)
            .show(ui, |ui| {
                for title in [
                    "IMU",
                    "serial",
                    "state",
                    "detail",
                    "transitions",
                    "stream",
                    "orientation xyzw",
                    "angular velocity xyz",
                    "linear acceleration xyz",
                    "device time",
                    "age",
                    "time (s)",
                ] {
                    header(ui, title);
                }
                ui.end_row();
                for imu in snapshot.imus.values() {
                    ui.monospace(&imu.imu);
                    ui.monospace(&imu.serial_device);
                    status_cells(ui, imu.status.as_ref());
                    stream_cell(ui, imu.sample_available, imu.sample_current_connection);
                    if imu.sample_available {
                        ui.monospace(format!(
                            "[{:.5}, {:.5}, {:.5}, {:.5}]",
                            imu.orientation.x,
                            imu.orientation.y,
                            imu.orientation.z,
                            imu.orientation.w
                        ));
                        ui.monospace(vector(imu.angular_velocity));
                        ui.monospace(vector(imu.linear_acceleration));
                        ui.monospace(duration(imu.device_timestamp));
                        age_cell(ui, imu.sample_age, true, DISPLAY_ONLY_STALE_AFTER);
                        ui.monospace(source_time(imu.monotonic_ns));
                    } else {
                        for _ in 0..6 {
                            ui.weak("—");
                        }
                    }
                    ui.end_row();
                }
            });
    });
}

fn summary(ui: &mut egui::Ui, label: &str, value: usize, color: Color32) {
    ui.group(|ui| {
        ui.label(label);
        ui.colored_label(color, RichText::new(value.to_string()).size(20.0).strong());
    });
}

fn header(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).strong());
}

fn status_cells(ui: &mut egui::Ui, status: Option<&DisplayDeviceStatus>) {
    match status {
        Some(status) => {
            ui.colored_label(
                device_state_color(status.state),
                format!("{:?}", status.state),
            );
            ui.label(&status.detail);
            ui.monospace(status.transition_count.to_string());
        }
        None => {
            ui.weak("unknown");
            ui.weak("—");
            ui.weak("—");
        }
    }
}

fn group_state_cell(ui: &mut egui::Ui, state: GroupState) {
    ui.colored_label(group_state_color(state), format!("{state:?}"));
}

fn stream_cell(ui: &mut egui::Ui, available: bool, current_connection: bool) {
    if current_connection {
        ui.colored_label(Color32::LIGHT_GREEN, "live");
    } else if available {
        ui.colored_label(Color32::YELLOW, "frozen");
    } else {
        ui.weak("waiting");
    }
}

fn age_cell(ui: &mut egui::Ui, age: Duration, available: bool, stale_after: Duration) {
    if !available {
        ui.weak("—");
    } else {
        ui.colored_label(
            if age > stale_after {
                Color32::LIGHT_RED
            } else {
                Color32::LIGHT_GREEN
            },
            duration(age),
        );
    }
}

fn device_state_color(state: DeviceState) -> Color32 {
    match state {
        DeviceState::Online => Color32::LIGHT_GREEN,
        DeviceState::Fault => Color32::LIGHT_RED,
        DeviceState::Starting
        | DeviceState::Connecting
        | DeviceState::Degraded
        | DeviceState::Reconnecting => Color32::YELLOW,
        DeviceState::Shutdown | DeviceState::Unspecified | DeviceState::Unknown(_) => Color32::GRAY,
        _ => Color32::GRAY,
    }
}

fn group_state_color(state: GroupState) -> Color32 {
    match state {
        GroupState::Running => Color32::LIGHT_GREEN,
        GroupState::Fault => Color32::LIGHT_RED,
        GroupState::Idle => Color32::LIGHT_BLUE,
        GroupState::Damped => Color32::YELLOW,
        GroupState::Shutdown | GroupState::Unspecified | GroupState::Unknown(_) => Color32::GRAY,
        _ => Color32::GRAY,
    }
}

const fn severity_color(severity: DiagnosticSeverity) -> Color32 {
    match severity {
        DiagnosticSeverity::Info => Color32::LIGHT_BLUE,
        DiagnosticSeverity::Warning => Color32::YELLOW,
        DiagnosticSeverity::Error => Color32::LIGHT_RED,
    }
}

const fn severity_name(severity: DiagnosticSeverity) -> &'static str {
    match severity {
        DiagnosticSeverity::Info => "info",
        DiagnosticSeverity::Warning => "warning",
        DiagnosticSeverity::Error => "error",
    }
}

fn duration(value: Duration) -> String {
    if value.as_secs() > 0 {
        format!("{:.3} s", value.as_secs_f64())
    } else {
        format!("{} ms", value.as_millis())
    }
}

fn source_time(value: Option<u64>) -> String {
    value.map_or_else(
        || "local".into(),
        |value| format!("{:.6}", value as f64 / 1e9),
    )
}

fn vector(value: bello_hardware_client::Vector3) -> String {
    format!("[{:.5}, {:.5}, {:.5}]", value.x, value.y, value.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_diagnostic_units_and_unknown_times() {
        assert_eq!(duration(Duration::from_millis(12)), "12 ms");
        assert_eq!(duration(Duration::from_millis(1_250)), "1.250 s");
        assert_eq!(source_time(None), "local");
        assert_eq!(source_time(Some(1_500_000_000)), "1.500000");
    }

    #[test]
    fn fault_and_stale_states_have_high_visibility_colors() {
        assert_eq!(device_state_color(DeviceState::Fault), Color32::LIGHT_RED);
        assert_eq!(group_state_color(GroupState::Fault), Color32::LIGHT_RED);
        assert_eq!(severity_color(DiagnosticSeverity::Warning), Color32::YELLOW);
    }
}
