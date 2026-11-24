use anyhow::Result;
use bello_sim_serial::Server;
use nix::sys::termios::{SetArg, cfmakeraw, tcgetattr, tcsetattr};
use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{self, ErrorKind, Read, Write};
use std::os::unix::{fs::OpenOptionsExt, net::UnixStream};
use std::path::Path;

pub(crate) struct Uart<E> {
    encoder: E,
    endpoint: Endpoint<E>,
}

enum Endpoint<E> {
    Device(Port),
    Pty {
        server: Server,
        initial_encoder: E,
        session: Option<Port>,
    },
}

struct Port {
    file: File,
    peer: Peer,
}

enum Peer {
    Device { pending: VecDeque<u8> },
    Pty(UnixStream),
}

impl<E: Clone> Uart<E> {
    pub(crate) fn open(path: &Path, encoder: E) -> Result<Self> {
        let endpoint = if let Some(name) = path.to_str().and_then(|s| s.strip_prefix("pty:")) {
            Endpoint::Pty {
                server: Server::bind(name)?,
                initial_encoder: encoder.clone(),
                session: None,
            }
        } else {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .custom_flags(nix::libc::O_NOCTTY | nix::libc::O_NONBLOCK)
                .open(path)?;
            let mut settings = tcgetattr(&file)?;
            cfmakeraw(&mut settings);
            tcsetattr(&file, SetArg::TCSANOW, &settings)?;
            Endpoint::Device(Port {
                file,
                peer: Peer::Device {
                    pending: VecDeque::new(),
                },
            })
        };
        Ok(Self { encoder, endpoint })
    }

    pub(crate) fn update(
        &mut self,
        encode: impl FnOnce(&mut E, &[u8]) -> Result<Vec<u8>>,
    ) -> Result<()> {
        match &mut self.endpoint {
            Endpoint::Device(port) => port.exchange(&mut self.encoder, encode),
            Endpoint::Pty {
                server,
                initial_encoder,
                session,
            } => {
                let port = match session {
                    Some(peer) => peer,
                    None => {
                        let Some((file, control)) = server.accept()? else {
                            return Ok(());
                        };
                        self.encoder.clone_from(initial_encoder);
                        session.insert(Port {
                            file,
                            peer: Peer::Pty(control),
                        })
                    }
                };
                match port.exchange(&mut self.encoder, encode) {
                    Err(e) if e.is::<io::Error>() => {
                        *session = None;
                        Ok(())
                    }
                    result => result,
                }
            }
        }
    }
}

impl Port {
    fn exchange<E>(
        &mut self,
        encoder: &mut E,
        encode: impl FnOnce(&mut E, &[u8]) -> Result<Vec<u8>>,
    ) -> Result<()> {
        if let Peer::Pty(control) = &mut self.peer {
            match control.read(&mut [0]) {
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) if e.kind() == ErrorKind::Interrupted => return Ok(()),
                Err(e) => return Err(e.into()),
                Ok(_) => return Err(io::Error::from(ErrorKind::ConnectionAborted).into()),
            }
        }
        let mut buffer = [0; 4096];
        let count = match self.file.read(&mut buffer) {
            Ok(0) if matches!(self.peer, Peer::Pty(_)) => {
                return Err(io::Error::from(ErrorKind::UnexpectedEof).into());
            }
            Ok(n) => n,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::Interrupted) => 0,
            Err(e) => return Err(e.into()),
        };
        let bytes = encode(encoder, &buffer[..count])?;
        let Peer::Device { pending } = &mut self.peer else {
            self.file.write_all(&bytes)?;
            return Ok(());
        };
        anyhow::ensure!(
            pending.len() + bytes.len() <= 16384,
            "emulated UART peer is not consuming output"
        );
        pending.extend(bytes);
        while !pending.is_empty() {
            let (bytes, _) = pending.as_slices();
            match self.file.write(bytes) {
                Ok(0) => return Err(io::Error::from(ErrorKind::WriteZero).into()),
                Ok(n) => {
                    pending.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }
}
