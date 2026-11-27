//! The fields of a hardwared deployment that its simulator peer consumes.
use anyhow::{Context, Result};
use bello_hardware_api::{camera::config::Transport, deployment::BusTransportConfig};
use robstride_emulator::config::CanInterface;
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

#[derive(Debug, Deserialize)]
pub(crate) struct Deployment {
    pub description: PathBuf,
    #[serde(default)]
    pub bus_transports: BTreeMap<String, BusTransportConfig>,
    #[serde(default)]
    pub cameras: BTreeMap<String, Transport>,
}

impl Deployment {
    pub fn interfaces(&self) -> Result<BTreeMap<String, CanInterface>> {
        self.bus_transports
            .iter()
            .map(|(name, bus)| {
                let interface = match bus {
                    BusTransportConfig::Sim(bus) => CanInterface::Sim(*bus),
                    BusTransportConfig::Socketcan { interface } => {
                        CanInterface::Socketcan(interface.clone())
                    }
                    BusTransportConfig::Gsusb { .. } => anyhow::bail!(
                        "cannot emulate gsusb bus {name}; use sim or virtual SocketCAN"
                    ),
                };
                Ok((name.clone(), interface))
            })
            .collect()
    }

    pub fn camera_outputs(
        &self,
        additional: &BTreeMap<String, Vec<String>>,
        samples: i32,
    ) -> Result<Vec<crate::cameras::Config>> {
        let mut sockets: BTreeMap<&Path, Vec<String>> = BTreeMap::new();
        for (name, transport) in &self.cameras {
            if let Transport::Mujoco { socket } = transport {
                sockets.entry(socket).or_default().push(name.clone());
            }
        }
        for (anchor, names) in additional {
            let Some(Transport::Mujoco { socket }) = self.cameras.get(anchor) else {
                anyhow::bail!(
                    "additional streams require a camera bound to a mujoco socket: {anchor}"
                );
            };
            sockets.entry(socket).or_default().extend_from_slice(names);
        }
        sockets
            .into_iter()
            .map(|(socket, names)| {
                crate::cameras::Config::from_streams(
                    socket.to_owned(),
                    names,
                    samples,
                    &self.description,
                )
            })
            .collect()
    }

    pub fn load(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)
            .with_context(|| format!("opening hardwared deployment {}", path.display()))?;
        let mut deployment: Self = serde_yaml::from_reader(file)
            .with_context(|| format!("parsing hardwared deployment {}", path.display()))?;
        let description = path.with_file_name(&deployment.description);
        deployment.description = description
            .canonicalize()
            .with_context(|| format!("resolving robot description {}", description.display()))?;
        Ok(deployment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_bindings_group_renderers_and_allow_extra_views() -> Result<()> {
        let mut deployment = Deployment::load(
            &Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../bello-hardwared/config/bello-mini-torso-sim.yaml"),
        )?;
        let socket = PathBuf::from("/tmp/grippers.sock");
        for name in ["left_gripper", "right_gripper"] {
            deployment.cameras.insert(
                name.into(),
                Transport::Mujoco {
                    socket: socket.clone(),
                },
            );
        }
        let extra = BTreeMap::from([("left_gripper".into(), vec!["birds_eye".into()])]);
        let outputs = deployment.camera_outputs(&extra, 2)?;
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0].socket, socket);
        assert_eq!(outputs[0].offscreen_samples, 2);
        let names: Vec<_> = outputs[0]
            .cameras
            .iter()
            .map(|camera| camera.name.as_str())
            .collect();
        assert_eq!(names, ["left_gripper", "right_gripper", "birds_eye"]);

        deployment.cameras.insert(
            "right_gripper".into(),
            Transport::Mujoco {
                socket: "/tmp/right.sock".into(),
            },
        );
        assert_eq!(deployment.camera_outputs(&extra, 2)?.len(), 2);
        deployment.cameras.insert(
            "left_gripper".into(),
            Transport::V4l2 {
                device: "/dev/video0".into(),
            },
        );
        assert!(deployment.camera_outputs(&extra, 2).is_err());
        let outputs = deployment.camera_outputs(&BTreeMap::new(), 2)?;
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0].cameras[0].name, "right_gripper");
        Ok(())
    }
}
