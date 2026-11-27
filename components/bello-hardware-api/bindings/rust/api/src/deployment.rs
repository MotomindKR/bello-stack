//! CAN transport declarations shared by hardwared and its simulator peer.
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum BusTransportConfig {
    Socketcan { interface: String },
    Gsusb { serial: String, channel: u8 },
    Sim(bello_sim_can::SimBus),
}

impl BusTransportConfig {
    pub fn label(&self) -> String {
        match self {
            Self::Socketcan { interface } => format!("socketcan:{interface}"),
            Self::Sim(bus) => format!("sim:{}:{}", bus.daemon_port, bus.simulator_port),
            Self::Gsusb { serial, channel } => format!("gsusb:{serial}:{channel}"),
        }
    }
}
