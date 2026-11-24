use anyhow::{Result, ensure};
use bello_sim::runtime::{Config, run};
mod support;
use robstride_emulator::config::CanInterface;
use robstride_protocol::{
    ControlCommand, MotorParams, active_report_frame, control_frame, enable_frame,
    param_write_u8_frame, parse_feedback,
};
use std::io::ErrorKind;
use std::path::Path;
use std::time::Duration;

#[test]
fn motor_follows_position_commands_over_sim_can() -> Result<()> {
    let profile = Path::new(env!("CARGO_MANIFEST_DIR")).join("config/emulator_milly.yaml");
    let mut config = Config::load(&profile)?;
    let (bus, client, simulator_port) = support::reserve_bus()?;
    client.set_read_timeout(Some(Duration::from_millis(10)))?;
    config
        .bus_interfaces
        .insert("can0".into(), CanInterface::Sim(bus));
    config.pose.listen.set_port(0);
    drop(simulator_port);

    let params = MotorParams::for_model("RS_06", 1).unwrap();
    let physics = std::thread::spawn(move || run(config, Some(Duration::from_secs(1))));
    let mut target = 0.3;
    let mut reversed = false;
    while !physics.is_finished() {
        for request in [
            active_report_frame(0, 1, true),
            param_write_u8_frame(0, 1, 0x7005, 0),
            enable_frame(0, 1),
            control_frame(
                params,
                ControlCommand {
                    motor_id: 1,
                    position: target,
                    kp: 40.,
                    kd: 3.,
                    ..Default::default()
                },
            ),
        ] {
            client.send(request)?;
        }
        match client.recv() {
            Ok(frame) => {
                if let Some(feedback) = parse_feedback(&frame, params) {
                    ensure!(feedback.motor_id == 1);
                    if feedback.position > 0.15 {
                        target = -0.3;
                    } else if target < 0. && feedback.position < -0.15 {
                        reversed = true;
                        break;
                    }
                }
            }
            Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {}
            Err(e) => return Err(e.into()),
        }
    }
    physics.join().unwrap()?;
    ensure!(
        reversed,
        "simulated joint did not follow both position targets"
    );
    Ok(())
}
