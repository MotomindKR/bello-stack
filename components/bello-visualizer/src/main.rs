use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::time::Duration;

use bello_hardware_client::GroupState;
use bello_visualizer::config::RootConfig;
use bello_visualizer::diagnostics::DiagnosticsUi;
use bello_visualizer::editor::{EditorMode, GroupEditor};
use bello_visualizer::gamepad::gamepad_ui;
use bello_visualizer::runtime::{ControlStatus, DisplaySnapshot, RuntimeHandle};
use bello_visualizer::scene::ModelScene;
use bello_visualizer::transmission::{CoordinateFeedback, FeedbackReconstructor, Transmission};
use bello_visualizer::{RobotDescription, VisualizerConfig};
use clap::Parser;
use kiss3d::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq)]
enum VisualizerTab {
    Controls,
    Gamepad,
    Environment,
}

#[derive(Parser)]
#[command(about = "Monitor and safely command Bello hardware against its URDF description")]
struct Arguments {
    #[arg(long, value_name = "YAML")]
    config: PathBuf,
    /// Override the hardwared endpoint from the YAML.
    #[arg(long)]
    hardware_endpoint: Option<String>,
    /// Override the external PoseService endpoint from the YAML.
    #[arg(long)]
    pose_endpoint: Option<String>,
}

#[kiss3d::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "bello_visualizer=info".into()),
        )
        .init();
    if let Err(error) = run().await {
        eprintln!("bello-visualizer: {error:#}");
        std::process::exit(1);
    }
}

async fn run() -> anyhow::Result<()> {
    let arguments = Arguments::parse();
    let mut config = VisualizerConfig::load(&arguments.config)?;
    if let Some(endpoint) = arguments.hardware_endpoint {
        config.hardware.endpoint = endpoint;
    }
    if let Some(endpoint) = arguments.pose_endpoint {
        match &mut config.root {
            RootConfig::ExternalPose {
                endpoint: configured,
                ..
            } => *configured = endpoint,
            RootConfig::Fixed { .. } => {
                anyhow::bail!("--pose-endpoint requires root.source: external_pose")
            }
        }
    }
    config.validate()?;
    let description = RobotDescription::load(&config.urdf)?;
    let profile_configs = config.control_profiles.clone();
    let runtime = RuntimeHandle::spawn(config, description.clone())?;
    let mut editors = BTreeMap::<String, GroupEditor>::new();

    let mut window = Window::new(&format!("Bello visualizer — {}", description.name())).await;
    window.set_background_color(Color::new(0.035, 0.045, 0.06, 1.0));
    window.set_ambient(0.4);
    window.set_ambient_color(Color::new(0.85, 0.9, 1.0, 1.0));
    let mut camera = OrbitCamera3d::new(Vec3::new(1.7, -1.7, 1.2), Vec3::new(0.0, 0.0, 0.65));
    // URDF models are Z-up. Kiss3d's orbit camera defaults to Y-up.
    camera.set_up_axis_dir(Vec3::Z);
    let mut scene = SceneNode3d::empty();
    scene
        .add_light(
            Light::point(20.0)
                .with_color(Color::new(1.0, 0.94, 0.85, 1.0))
                .with_intensity(3.2),
        )
        .set_position(Vec3::new(2.0, -2.0, 3.0));
    // A cool, non-shadow-casting fill keeps the far side readable without
    // flattening the key light's shape cues.
    scene
        .add_light(
            Light::point(20.0)
                .with_color(Color::new(0.72, 0.84, 1.0, 1.0))
                .with_intensity(0.9)
                .with_casts_shadows(false),
        )
        .set_position(Vec3::new(-2.0, 1.5, 2.2));
    let mut ground = scene.add_cube(12.0, 12.0, 0.02);
    ground
        .set_position(Vec3::new(0.0, 0.0, -0.01))
        .set_color(Color::new(0.28, 0.30, 0.34, 1.0))
        .set_metallic(0.0)
        .set_roughness(0.9);
    let mut model_scene = ModelScene::build(&mut scene, &description)?;

    let mut selected = BTreeSet::<String>::new();
    let mut diagnostics = DiagnosticsUi::new();
    let mut editor_mode = EditorMode::Actuator;
    let mut commanding = false;
    let mut reconstructors = BTreeMap::<String, FeedbackReconstructor>::new();
    let mut mapping_warnings = BTreeMap::<String, String>::new();
    let mut visualizer_tab = VisualizerTab::Controls;
    let mut show_ground = true;
    let mut sky_color = [0.035, 0.045, 0.06];
    let mut sky_path = String::new();
    let mut sky_rotation_degrees = 0.0_f32;
    let mut sky_intensity = 1.0_f32;
    let mut sky_status = "Using solid color".to_string();

    while window.render_3d(&mut scene, &mut camera).await {
        let snapshot = runtime.latest();
        for (group, profile) in &profile_configs {
            if editors.contains_key(group) {
                continue;
            }
            let coordinates = snapshot
                .motors
                .values()
                .filter(|motor| motor.group == *group)
                .map(|motor| (motor.coordinate.clone(), motor.limits))
                .collect::<Vec<_>>();
            if coordinates.is_empty() {
                continue;
            }
            let resolved = profile
                .resolve_limits(&description, coordinates)
                .map_err(|error| {
                    anyhow::anyhow!("control profile {group:?} cannot use hardware limits: {error}")
                })?;
            editors.insert(
                group.clone(),
                GroupEditor::new(&description, resolved).map_err(|error| {
                    anyhow::anyhow!("control profile {group:?} has invalid transmission: {error}")
                })?,
            );
        }
        model_scene.set_root_transform(snapshot.root_transform);
        update_model_pose(
            &description,
            &snapshot,
            &mut reconstructors,
            &mut mapping_warnings,
            &mut model_scene,
        );
        for (group, editor) in &mut editors {
            if !matches!(snapshot.control.get(group), Some(ControlStatus::Running)) {
                editor.follow_feedback(group, &snapshot);
            }
        }

        let mut start_clicked = false;
        let mut stop_clicked = false;
        let mut changed_groups = BTreeSet::new();
        let mut ground_changed = false;
        let mut sky_color_changed = false;
        let mut sky_load_requested = false;
        let mut sky_clear_requested = false;
        let mut sky_orientation_changed = false;
        window.draw_ui(|context| {
            diagnostics.show(context, &snapshot);
            orientation_indicator(context, &camera);
            kiss3d::egui::Window::new("Bello controls")
                .default_pos([920.0, 8.0])
                .default_width(420.0)
                .show(context, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(
                            &mut visualizer_tab,
                            VisualizerTab::Controls,
                            "Controls",
                        );
                        ui.selectable_value(&mut visualizer_tab, VisualizerTab::Gamepad, "Gamepad");
                        ui.selectable_value(
                            &mut visualizer_tab,
                            VisualizerTab::Environment,
                            "Environment",
                        );
                    });
                    ui.separator();

                    if visualizer_tab == VisualizerTab::Gamepad {
                        gamepad_ui(ui, snapshot.remote.as_ref());
                        return;
                    }

                    if visualizer_tab == VisualizerTab::Environment {
                        ui.heading("Scene environment");
                        ui.label("Sky");
                        ui.horizontal(|ui| {
                            ui.label("Solid color");
                            sky_color_changed |= ui.color_edit_button_rgb(&mut sky_color).changed();
                        });
                        ui.weak("Used when no sky image is loaded.");
                        ui.add_space(6.0);
                        ui.horizontal(|ui| {
                            ui.label("Image");
                            ui.text_edit_singleline(&mut sky_path);
                        });
                        ui.horizontal(|ui| {
                            if ui.button("Choose…").clicked()
                                && let Some(path) = rfd::FileDialog::new()
                                    .add_filter(
                                        "Sky images",
                                        &["hdr", "exr", "png", "jpg", "jpeg", "webp"],
                                    )
                                    .pick_file()
                            {
                                sky_path = path.to_string_lossy().into_owned();
                                sky_load_requested = true;
                            }
                            if ui.button("Load").clicked() {
                                sky_load_requested = true;
                            }
                            if ui.button("Use solid color").clicked() {
                                sky_clear_requested = true;
                            }
                        });
                        sky_orientation_changed |= ui
                            .add(
                                kiss3d::egui::Slider::new(
                                    &mut sky_rotation_degrees,
                                    -180.0..=180.0,
                                )
                                .text("rotation°"),
                            )
                            .changed();
                        sky_orientation_changed |= ui
                            .add(
                                kiss3d::egui::Slider::new(&mut sky_intensity, 0.0..=8.0)
                                    .text("intensity"),
                            )
                            .changed();
                        ui.weak("Equirectangular HDR, EXR, or standard image.");
                        ui.label(&sky_status);
                        ui.separator();
                        ui.label("Ground");
                        ground_changed |= ui.checkbox(&mut show_ground, "Show ground").changed();
                        ui.weak("The ground surface is aligned with world Z = 0.");
                        return;
                    }

                    let controls_busy = snapshot.control.values().any(control_status_busy);
                    ui.heading("Bello hardware");
                    status_ui(ui, &snapshot, &mapping_warnings);
                    ui.separator();
                    ui.label("Control groups");
                    for group in editors.keys() {
                        let mut enabled = selected.contains(group);
                        let status = snapshot
                            .control
                            .get(group)
                            .cloned()
                            .unwrap_or(ControlStatus::Monitoring);
                        let fault = snapshot
                            .groups
                            .get(group)
                            .filter(|state| state.state == GroupState::Fault);
                        ui.horizontal(|ui| {
                            ui.add_enabled(
                                !commanding && !controls_busy,
                                kiss3d::egui::Checkbox::new(&mut enabled, group),
                            );
                            ui.label(format!("{status:?}"));
                        });
                        if let Some(fault) = fault {
                            ui.horizontal(|ui| {
                                ui.colored_label(
                                    kiss3d::egui::Color32::LIGHT_RED,
                                    format!("{:?}", fault.cause),
                                )
                                .on_hover_text(&fault.detail);
                                let busy = control_status_busy(&status);
                                if ui
                                    .add_enabled(
                                        !busy,
                                        kiss3d::egui::Button::new("Clear Fault")
                                            .fill(kiss3d::egui::Color32::DARK_RED),
                                    )
                                    .on_hover_text(&fault.detail)
                                    .clicked()
                                    && let Err(error) = runtime.recover_groups(vec![group.clone()])
                                {
                                    tracing::warn!(group, "could not request recovery: {error}");
                                }
                            });
                        }
                        if enabled {
                            selected.insert(group.clone());
                        } else {
                            selected.remove(group);
                        }
                    }
                    let selected_busy = selected
                        .iter()
                        .any(|group| snapshot.control.get(group).is_some_and(control_status_busy));
                    ui.horizontal(|ui| {
                        let mut enabled = commanding;
                        let can_start = !selected.is_empty() && !selected_busy;
                        if ui
                            .add_enabled(
                                commanding || can_start,
                                kiss3d::egui::Checkbox::new(&mut enabled, "Commanding"),
                            )
                            .changed()
                        {
                            start_clicked = enabled;
                            stop_clicked = !enabled;
                        }
                    });

                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label("Active editor:");
                        ui.selectable_value(&mut editor_mode, EditorMode::Model, "Model joints");
                        ui.selectable_value(
                            &mut editor_mode,
                            EditorMode::Actuator,
                            "Actuator coordinates",
                        );
                    });
                    kiss3d::egui::ScrollArea::vertical().show(ui, |ui| {
                        for (group, editor) in &mut editors {
                            ui.collapsing(group, |ui| {
                                let editable = commanding
                                    && selected.contains(group)
                                    && matches!(
                                        snapshot.control.get(group),
                                        Some(ControlStatus::Running)
                                    );
                                if editor_ui(ui, editor, editor_mode, editable) {
                                    changed_groups.insert(group.clone());
                                }
                            });
                        }
                    });
                });
        });

        if ground_changed {
            ground.set_visible(show_ground);
        }
        if sky_color_changed {
            window.set_background_color(Color::new(sky_color[0], sky_color[1], sky_color[2], 1.0));
        }
        if sky_clear_requested {
            window.clear_skybox();
            sky_status = "Using solid color".to_string();
        } else if sky_load_requested {
            let path = sky_path.trim();
            if path.is_empty() {
                sky_status = "Choose a sky image first".to_string();
            } else if window.set_skybox_from_file(std::path::Path::new(path)) {
                window.set_skybox_orientation(sky_rotation_degrees.to_radians(), sky_intensity);
                sky_status = format!("Loaded {path}");
            } else {
                sky_status = format!("Could not load {path}");
            }
        } else if sky_orientation_changed && window.has_skybox() {
            window.set_skybox_orientation(sky_rotation_degrees.to_radians(), sky_intensity);
        }

        if stop_clicked {
            let _ = runtime.stop_all();
            commanding = false;
        } else if start_clicked && !commanding {
            for group in &selected {
                let editor = editors
                    .get_mut(group)
                    .expect("selected groups come from the editor map");
                editor.reset_for_new_session();
                editor.seed_from_feedback(group, &snapshot);
            }
            runtime
                .start_groups(selected.iter().cloned().collect())
                .map_err(anyhow::Error::msg)?;
            commanding = true;
        } else if commanding {
            for group in changed_groups {
                if selected.contains(&group) {
                    runtime
                        .set_targets(&group, editors[&group].actuator.clone())
                        .map_err(anyhow::Error::msg)?;
                }
            }
        }
    }

    if !runtime.shutdown(Duration::from_secs(3)) {
        tracing::warn!("background shutdown exceeded three seconds");
    }
    Ok(())
}

fn orientation_indicator(context: &kiss3d::egui::Context, camera: &OrbitCamera3d) {
    let view_rotation = camera.view_transform().rotation;
    kiss3d::egui::Area::new(kiss3d::egui::Id::new("world_orientation_indicator"))
        .anchor(kiss3d::egui::Align2::LEFT_BOTTOM, [12.0, -12.0])
        .interactable(false)
        .show(context, |ui| {
            let size = kiss3d::egui::vec2(108.0, 108.0);
            let (rect, _) = ui.allocate_exact_size(size, kiss3d::egui::Sense::hover());
            ui.painter()
                .rect_filled(rect, 7.0, kiss3d::egui::Color32::from_black_alpha(165));
            ui.painter().text(
                rect.left_top() + kiss3d::egui::vec2(8.0, 7.0),
                kiss3d::egui::Align2::LEFT_TOP,
                "WORLD",
                kiss3d::egui::FontId::proportional(11.0),
                kiss3d::egui::Color32::LIGHT_GRAY,
            );
            let origin = rect.center() + kiss3d::egui::vec2(0.0, 9.0);
            let axes = [
                ("X", Vec3::X, kiss3d::egui::Color32::from_rgb(235, 75, 75)),
                ("Y", Vec3::Y, kiss3d::egui::Color32::from_rgb(80, 220, 110)),
                ("Z", Vec3::Z, kiss3d::egui::Color32::from_rgb(80, 145, 245)),
            ];
            for (label, axis, color) in axes {
                let direction = project_world_axis(view_rotation, axis);
                let end = origin + kiss3d::egui::vec2(direction.x, direction.y) * 34.0;
                ui.painter()
                    .line_segment([origin, end], kiss3d::egui::Stroke::new(3.0, color));
                ui.painter().circle_filled(end, 3.5, color);
                let label_offset = if direction.length_squared() > 0.01 {
                    direction.normalize() * 9.0
                } else {
                    Vec2::new(0.0, -9.0)
                };
                ui.painter().text(
                    end + kiss3d::egui::vec2(label_offset.x, label_offset.y),
                    kiss3d::egui::Align2::CENTER_CENTER,
                    label,
                    kiss3d::egui::FontId::proportional(13.0),
                    color,
                );
            }
        });
}

fn project_world_axis(view_rotation: Quat, axis: Vec3) -> Vec2 {
    let view_axis = view_rotation * axis;
    // Egui's screen Y points down, opposite camera-space Y.
    Vec2::new(view_axis.x, -view_axis.y)
}

fn control_status_busy(status: &ControlStatus) -> bool {
    matches!(
        status,
        ControlStatus::Starting
            | ControlStatus::Running
            | ControlStatus::Recovering
            | ControlStatus::Stopping
    )
}

fn status_ui(
    ui: &mut kiss3d::egui::Ui,
    snapshot: &DisplaySnapshot,
    mapping_warnings: &BTreeMap<String, String>,
) {
    if snapshot.hardware_connected {
        ui.colored_label(
            kiss3d::egui::Color32::LIGHT_GREEN,
            "Hardware telemetry connected",
        );
    } else {
        ui.colored_label(
            kiss3d::egui::Color32::YELLOW,
            "Hardware telemetry disconnected",
        );
    }
    if let Some(warning) = &snapshot.hardware_warning {
        ui.colored_label(kiss3d::egui::Color32::YELLOW, warning);
    }
    if !snapshot.pose_available {
        ui.colored_label(
            kiss3d::egui::Color32::YELLOW,
            snapshot
                .pose_warning
                .as_deref()
                .unwrap_or("pose unavailable"),
        );
    }
    for (group, warning) in mapping_warnings {
        ui.colored_label(kiss3d::egui::Color32::YELLOW, format!("{group}: {warning}"));
    }
}

fn editor_ui(
    ui: &mut kiss3d::egui::Ui,
    editor: &mut GroupEditor,
    mode: EditorMode,
    editable: bool,
) -> bool {
    let mut changed = false;
    if !editable {
        ui.weak("Following live feedback; controls unlock while this group is running.");
    }
    match mode {
        EditorMode::Actuator => {
            for (name, target) in &mut editor.actuator {
                let bounds = &editor.profile.coordinates[name];
                ui.group(|ui| {
                    ui.strong(name);
                    changed |= jog_position(
                        ui,
                        &mut target.position,
                        bounds.position,
                        editor.profile.position_slew_rate,
                        editable,
                    );
                    changed |= ui
                        .add_enabled(
                            editable,
                            kiss3d::egui::Slider::new(
                                &mut target.velocity,
                                bounds.velocity.min..=bounds.velocity.max,
                            )
                            .text("velocity"),
                        )
                        .changed();
                    changed |= ui
                        .add_enabled(
                            editable,
                            kiss3d::egui::Slider::new(
                                &mut target.torque_feedforward,
                                bounds.torque_feedforward.min..=bounds.torque_feedforward.max,
                            )
                            .text("torque ff"),
                        )
                        .changed();
                    changed |= ui
                        .add_enabled(
                            editable,
                            kiss3d::egui::Slider::new(
                                &mut target.kp,
                                bounds.kp.min..=bounds.kp.max,
                            )
                            .text("kp"),
                        )
                        .changed();
                    changed |= ui
                        .add_enabled(
                            editable,
                            kiss3d::egui::Slider::new(
                                &mut target.kd,
                                bounds.kd.min..=bounds.kd.max,
                            )
                            .text("kd"),
                        )
                        .changed();
                });
            }
            if changed {
                let _ = editor.sync_model_from_actuator();
            }
            ui.weak("Converted model values (read-only)");
            readonly_model(ui, editor);
        }
        EditorMode::Model => {
            for index in 0..editor.transmission.joint_names().len() {
                ui.group(|ui| {
                    ui.strong(&editor.transmission.joint_names()[index]);
                    changed |= jog_position(
                        ui,
                        &mut editor.model_position[index],
                        editor.joint_position_bounds[index],
                        editor.profile.position_slew_rate,
                        editable,
                    );
                    changed |= ui
                        .add_enabled(
                            editable,
                            kiss3d::egui::Slider::new(
                                &mut editor.model_velocity[index],
                                editor.joint_velocity_bounds[index].min
                                    ..=editor.joint_velocity_bounds[index].max,
                            )
                            .text("velocity"),
                        )
                        .changed();
                    changed |= ui
                        .add_enabled(
                            editable,
                            kiss3d::egui::Slider::new(
                                &mut editor.model_effort[index],
                                editor.joint_effort_bounds[index].min
                                    ..=editor.joint_effort_bounds[index].max,
                            )
                            .text("effort ff"),
                        )
                        .changed();
                });
            }
            if changed {
                let _ = editor.sync_actuator_from_model();
            }
            ui.weak("Converted actuator values and group control gains (read-only)");
            for (name, target) in &editor.actuator {
                ui.monospace(format!(
                    "{name}: q={:.4} v={:.4} τ={:.3} kp={:.2} kd={:.2}",
                    target.position,
                    target.velocity,
                    target.torque_feedforward,
                    target.kp,
                    target.kd
                ));
            }
        }
    }
    changed
}

fn jog_position(
    ui: &mut kiss3d::egui::Ui,
    position: &mut f64,
    bounds: bello_visualizer::config::BoundedDefault,
    rate: f64,
    editable: bool,
) -> bool {
    let mut direction = 0.0;
    ui.horizontal(|ui| {
        ui.label("position");
        if ui
            .add_enabled(editable, kiss3d::egui::Button::new("◀ hold"))
            .is_pointer_button_down_on()
        {
            direction -= 1.0;
        }
        ui.add_enabled(
            false,
            kiss3d::egui::Slider::new(position, bounds.min..=bounds.max).show_value(true),
        );
        if ui
            .add_enabled(editable, kiss3d::egui::Button::new("hold ▶"))
            .is_pointer_button_down_on()
        {
            direction += 1.0;
        }
    });
    if direction == 0.0 {
        return false;
    }
    let dt = ui.input(|input| input.stable_dt).min(0.1) as f64;
    let next = (*position + direction * rate * dt).clamp(bounds.min, bounds.max);
    let changed = next != *position;
    *position = next;
    changed
}

fn readonly_model(ui: &mut kiss3d::egui::Ui, editor: &GroupEditor) {
    for (index, name) in editor.transmission.joint_names().iter().enumerate() {
        ui.monospace(format!(
            "{name}: q={:.4} v={:.4} τ={:.3}",
            editor.model_position[index], editor.model_velocity[index], editor.model_effort[index]
        ));
    }
}

fn update_model_pose(
    description: &RobotDescription,
    snapshot: &DisplaySnapshot,
    reconstructors: &mut BTreeMap<String, FeedbackReconstructor>,
    warnings: &mut BTreeMap<String, String>,
    scene: &mut ModelScene,
) {
    let groups = snapshot
        .motors
        .values()
        .map(|motor| motor.group.clone())
        .collect::<BTreeSet<_>>();
    let mut positions = BTreeMap::new();
    for group in groups {
        let group_feedback = snapshot
            .motors
            .values()
            .filter(|motor| motor.group == group)
            .map(|motor| {
                (
                    motor.coordinate.clone(),
                    CoordinateFeedback {
                        position: motor.position,
                        velocity: motor.velocity,
                        effort: motor.effort,
                        age: if motor.feedback_available {
                            motor.age
                        } else {
                            Duration::MAX
                        },
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();
        if !reconstructors.contains_key(&group) {
            match Transmission::from_description(description, group_feedback.keys()) {
                Ok(transmission) => {
                    reconstructors.insert(
                        group.clone(),
                        FeedbackReconstructor::new(transmission, Duration::from_millis(200)),
                    );
                    warnings.remove(&group);
                }
                Err(error) => {
                    warnings.insert(group.clone(), error.to_string());
                    continue;
                }
            }
        }
        let reconstructor = reconstructors
            .get_mut(&group)
            .expect("reconstructor was inserted above");
        let joint_names = reconstructor.transmission().joint_names().to_vec();
        let reconstructed = reconstructor.update(&group_feedback);
        for (name, position) in joint_names.into_iter().zip(&reconstructed.position) {
            positions.insert(name, *position);
        }
        if reconstructed.available {
            warnings.remove(&group);
        } else {
            warnings.insert(
                group,
                "one or more coordinates are missing or stale; holding last pose".into(),
            );
        }
    }
    scene.set_joint_positions(&positions);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world_axis_indicator_projects_camera_space_with_screen_y_down() {
        assert_eq!(project_world_axis(Quat::IDENTITY, Vec3::X), Vec2::X);
        assert_eq!(project_world_axis(Quat::IDENTITY, Vec3::Y), -Vec2::Y);
        assert_eq!(project_world_axis(Quat::IDENTITY, Vec3::Z), Vec2::ZERO);
    }
}
