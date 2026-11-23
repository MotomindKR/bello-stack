use std::io;
use std::net::UdpSocket;

pub fn available_ports() -> io::Result<String> {
    let daemon = UdpSocket::bind(("127.0.0.1", 0))?;
    let simulator = UdpSocket::bind(("127.0.0.1", 0))?;
    Ok(format!(
        "daemon_port: {}, simulator_port: {}",
        daemon.local_addr()?.port(),
        simulator.local_addr()?.port()
    ))
}
