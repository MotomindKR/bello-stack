use std::io;
use std::net::{Ipv4Addr, UdpSocket};

use bello_sim_can::{SimBus, SimSocket};

pub fn reserve_bus() -> io::Result<(SimBus, SimSocket, UdpSocket)> {
    let daemon = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0))?;
    let simulator = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0))?;
    let bus = SimBus {
        daemon_port: daemon.local_addr()?.port(),
        simulator_port: simulator.local_addr()?.port(),
    };
    drop(daemon);
    Ok((bus, bus.open_daemon()?, simulator))
}
