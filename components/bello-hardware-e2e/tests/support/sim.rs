use std::io;
use std::net::UdpSocket;

pub fn available_ports<const BUSES: usize>() -> io::Result<[String; BUSES]> {
    let mut ports = std::array::from_fn(|_| String::new());
    let mut sockets = Vec::with_capacity(2 * BUSES);
    for bus in &mut ports {
        let daemon = UdpSocket::bind(("127.0.0.1", 0))?;
        let simulator = UdpSocket::bind(("127.0.0.1", 0))?;
        *bus = format!(
            "daemon_port: {}, simulator_port: {}",
            daemon.local_addr()?.port(),
            simulator.local_addr()?.port()
        );
        sockets.extend([daemon, simulator]);
    }
    Ok(ports)
}
