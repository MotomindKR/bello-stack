use kiss3d::egui::{self, Color32, FontId, RichText, Stroke, Vec2};

use crate::runtime::DisplayRemote;

pub fn gamepad_ui(ui: &mut egui::Ui, remote: Option<&DisplayRemote>) {
    ui.heading("Gamepad");
    let Some(remote) = remote else {
        ui.colored_label(Color32::YELLOW, "Remote command service is starting");
        return;
    };

    ui.label(format!("Streaming on {}", remote.listen));
    if let Some(error) = &remote.service_error {
        ui.colored_label(Color32::LIGHT_RED, format!("Service stopped: {error}"));
        return;
    }
    if remote.controller_connected {
        ui.colored_label(
            Color32::LIGHT_GREEN,
            format!("Connected: {}", remote.controller_name),
        );
    } else {
        ui.colored_label(Color32::YELLOW, "Disconnected; publishing zero intent");
    }

    let input_color = if remote.controller_connected {
        Color32::from_rgb(90, 170, 255)
    } else {
        Color32::GRAY
    };
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        stick(
            ui,
            "Left stick",
            remote.raw_left_stick_x,
            remote.raw_left_stick_y,
            "lateral / forward",
            input_color,
        );
        stick(
            ui,
            "Right stick",
            remote.raw_right_stick_x,
            remote.raw_right_stick_y,
            "yaw / not mapped",
            input_color,
        );
    });

    ui.separator();
    ui.strong("Published command");
    signed_bar(ui, "Forward", remote.normalized_forward);
    signed_bar(ui, "Lateral", remote.normalized_lateral);
    signed_bar(ui, "Yaw", remote.normalized_yaw);
    if remote.controller_connected && !remote.intent_armed {
        ui.colored_label(
            Color32::YELLOW,
            "Return mapped axes to neutral to arm command output",
        );
    } else if remote.intent_armed {
        ui.colored_label(Color32::LIGHT_GREEN, "Command output armed");
    }

    ui.add_space(6.0);
    ui.horizontal(|ui| {
        ui.label("R1");
        let text = if remote.abort_active {
            RichText::new("ABORT ACTIVE").strong().color(Color32::WHITE)
        } else {
            RichText::new("released").color(Color32::GRAY)
        };
        let fill = if remote.abort_active {
            Color32::DARK_RED
        } else {
            Color32::from_gray(35)
        };
        ui.add(egui::Button::new(text).fill(fill));
    });
}

fn stick(ui: &mut egui::Ui, label: &str, x: f64, y: f64, mapping: &str, color: Color32) {
    let size = egui::vec2(180.0, 170.0);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.text(
        rect.center_top() + egui::vec2(0.0, 2.0),
        egui::Align2::CENTER_TOP,
        label,
        FontId::proportional(15.0),
        Color32::WHITE,
    );

    let center = rect.center() + egui::vec2(0.0, -2.0);
    let radius = 53.0;
    painter.circle_filled(center, radius, Color32::from_black_alpha(75));
    painter.circle_stroke(center, radius, Stroke::new(1.5, Color32::from_gray(95)));
    painter.line_segment(
        [
            center - Vec2::new(radius, 0.0),
            center + Vec2::new(radius, 0.0),
        ],
        Stroke::new(1.0, Color32::from_gray(65)),
    );
    painter.line_segment(
        [
            center - Vec2::new(0.0, radius),
            center + Vec2::new(0.0, radius),
        ],
        Stroke::new(1.0, Color32::from_gray(65)),
    );
    let position =
        center + egui::vec2(x.clamp(-1.0, 1.0) as f32, -y.clamp(-1.0, 1.0) as f32) * (radius - 8.0);
    painter.line_segment([center, position], Stroke::new(3.0, color));
    painter.circle_filled(position, 8.0, color);
    painter.text(
        rect.center_bottom() - egui::vec2(0.0, 1.0),
        egui::Align2::CENTER_BOTTOM,
        format!("{mapping}  {x:+.2}, {y:+.2}"),
        FontId::monospace(11.0),
        Color32::LIGHT_GRAY,
    );
}

fn signed_bar(ui: &mut egui::Ui, label: &str, value: f64) {
    ui.horizontal(|ui| {
        ui.label(format!("{label:>7}"));
        let desired = egui::vec2((ui.available_width() - 55.0).max(120.0), 16.0);
        let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, Color32::from_black_alpha(75));
        let center_x = rect.center().x;
        painter.line_segment(
            [
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            ],
            Stroke::new(1.0, Color32::from_gray(110)),
        );
        let value = value.clamp(-1.0, 1.0) as f32;
        let end_x = center_x + value * rect.width() * 0.5;
        let fill =
            egui::Rect::from_x_y_ranges(center_x.min(end_x)..=center_x.max(end_x), rect.y_range());
        let color = if value == 0.0 {
            Color32::from_gray(80)
        } else {
            Color32::from_rgb(75, 190, 135)
        };
        painter.rect_filled(fill, 2.0, color);
        ui.monospace(format!("{value:+.3}"));
    });
}

#[cfg(test)]
mod tests {
    use std::net::{IpAddr, Ipv4Addr, SocketAddr};

    use super::*;

    #[test]
    fn connected_gamepad_snapshot_renders() {
        let remote = DisplayRemote {
            listen: SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 50054),
            service_error: None,
            controller_connected: true,
            controller_name: "test gamepad".into(),
            intent_armed: true,
            raw_left_stick_x: 0.25,
            raw_left_stick_y: -0.5,
            raw_right_stick_x: -0.75,
            raw_right_stick_y: 0.0,
            normalized_forward: 0.45,
            normalized_lateral: 0.17,
            normalized_yaw: 0.72,
            abort_active: false,
        };
        let context = egui::Context::default();
        let output = context.run_ui(egui::RawInput::default(), |ui| {
            gamepad_ui(ui, Some(&remote));
        });

        assert!(!output.shapes.is_empty());
    }
}
