//! GPU-independent model, mapping, configuration, and runtime logic for the Bello visualizer.

pub mod config;
pub mod diagnostics;
pub mod editor;
pub mod gamepad;
pub mod remote;
pub mod runtime;
pub mod scene;
pub mod transform;
pub mod transmission;

pub use bello_robot_description::RobotDescription;
pub use config::VisualizerConfig;
pub use transform::Transform;
pub use transmission::Transmission;
