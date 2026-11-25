use robstride_protocol::Frame;

const SIM_FIRMWARE_PREFIX: &[u8] = b"BELLO-SIM-FW/1\n";
const MAX_SIM_FIRMWARE_BYTES: usize = SIM_FIRMWARE_PREFIX.len() + "255.255.255.255".len();
const VERSION_QUERY: [u8; 8] = [0, 0xc4, 0, 0, 0, 0, 0, 0];

#[derive(Clone, Debug)]
pub(crate) struct Maintenance {
    uuid: [u8; 8],
    version: [u8; 4],
    ota: Ota,
}

#[derive(Clone, Debug)]
enum Ota {
    Idle,
    Started,
    Receiving { size: usize, packets: Vec<[u8; 8]> },
    Complete { count: usize },
}

enum Ack {
    Accepted,
    Rejected,
    Retry(u16),
}

impl Maintenance {
    pub(crate) fn new(model: &str, interface: &str, id: u8) -> Option<Self> {
        // Firmware families follow motor-setup-tool's motor model catalog.
        let family = match model {
            "RS_00" => 0,
            "RS_01" => 1,
            "RS_02" => 2,
            "RS_03" => 3,
            "RS_04" => 4,
            "RS_05" => 5,
            "RS_06" => 6,
            "RS_10P" => 10,
            _ => return None,
        };
        // FNV-1a makes simulated identities reproducible across process restarts.
        let uuid = interface
            .bytes()
            .chain([family, id])
            .fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
                (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
            })
            .to_le_bytes();
        Some(Self {
            uuid,
            version: [0, family, 0, 1],
            ota: Ota::Idle,
        })
    }

    pub(crate) fn updating(&self) -> bool {
        matches!(self.ota, Ota::Started | Ota::Receiving { .. })
    }

    pub(crate) fn reply(
        &mut self,
        host: u8,
        motor: u8,
        stopped: bool,
        frame: Frame,
    ) -> Option<Frame> {
        let kind = (frame.id >> 24) as u8;
        let field = (frame.id >> 8) as u16;
        let (kind, status, data): (u8, u8, [u8; 8]) = match kind {
            0 if field == u16::from(host) && frame.data == [0; 8] => (0, 0, self.uuid),
            4 if field == u16::from(host) && frame.data == VERSION_QUERY => {
                let [a, b, c, d] = self.version;
                (2, 0, [0, 0xc4, 0x56, a, b, c, d, 0])
            }
            11..=14 => {
                let ack = match (kind, stopped, field) {
                    (11 | 12, _, field) if field != u16::from(host) => return None,
                    (_, false, _) => Ack::Rejected,
                    (11, true, _) => self.begin(frame.data),
                    (12, true, _) => self.info(frame.data),
                    (13, true, index) => self.data(index, frame.data),
                    (14, true, 0) => self.end(frame.data),
                    _ => Ack::Rejected,
                };
                let (status, data) = match ack {
                    Ack::Accepted => (0, [0; 8]),
                    Ack::Rejected => (1, [0; 8]),
                    Ack::Retry(index) => {
                        let mut data = [0; 8];
                        data[..2].copy_from_slice(&index.to_le_bytes());
                        (0x0f, data)
                    }
                };
                (kind, status, data)
            }
            _ => return None,
        };
        Some(Frame {
            id: (u32::from(kind) << 24)
                | (u32::from(status) << 16)
                | (u32::from(motor) << 8)
                | u32::from(host),
            data,
        })
    }

    fn begin(&mut self, uuid: [u8; 8]) -> Ack {
        if uuid != self.uuid {
            return Ack::Rejected;
        }
        // START also restarts a transfer abandoned by a previous updater.
        self.ota = Ota::Started;
        Ack::Accepted
    }

    fn info(&mut self, data: [u8; 8]) -> Ack {
        let size = u32::from_le_bytes(data[..4].try_into().unwrap()) as usize;
        let count = u32::from_le_bytes(data[4..].try_into().unwrap()) as usize;
        if size == 0 || size > MAX_SIM_FIRMWARE_BYTES || count != size.div_ceil(8) {
            return Ack::Rejected;
        }
        match self.ota {
            Ota::Started => {
                self.ota = Ota::Receiving {
                    size,
                    packets: Vec::with_capacity(count),
                };
                Ack::Accepted
            }
            Ota::Receiving { size: existing, .. } if existing == size => Ack::Accepted,
            _ => Ack::Rejected,
        }
    }

    fn data(&mut self, index: u16, data: [u8; 8]) -> Ack {
        let Ota::Receiving { size, packets } = &mut self.ota else {
            return Ack::Rejected;
        };
        let index = usize::from(index);
        match packets.get(index) {
            Some(previous) if *previous == data => Ack::Accepted,
            Some(_) => Ack::Rejected,
            None if index > packets.len() => Ack::Retry(packets.len() as u16),
            None if index < size.div_ceil(8) => {
                packets.push(data);
                Ack::Accepted
            }
            None => Ack::Rejected,
        }
    }

    fn end(&mut self, data: [u8; 8]) -> Ack {
        let count = u32::from_le_bytes(data[..4].try_into().unwrap()) as usize;
        if data[4..] != [0; 4] {
            return Ack::Rejected;
        }
        if matches!(self.ota, Ota::Complete { count: completed } if count == completed) {
            return Ack::Accepted;
        }
        let Ota::Receiving { size, packets } = &self.ota else {
            return Ack::Rejected;
        };
        if count != size.div_ceil(8) || count != packets.len() {
            return Ack::Rejected;
        }
        let bytes = packets.as_flattened();
        if bytes[*size..].iter().any(|&byte| byte != 0xff) {
            return Ack::Rejected;
        }
        let Some(version) = simulated_version(&bytes[..*size], self.version[1]) else {
            return Ack::Rejected;
        };
        self.version = version;
        self.ota = Ota::Complete { count };
        Ack::Accepted
    }
}

fn simulated_version(bytes: &[u8], family: u8) -> Option<[u8; 4]> {
    let text = std::str::from_utf8(bytes.strip_prefix(SIM_FIRMWARE_PREFIX)?).ok()?;
    let mut parts = text.split('.');
    let mut version = [0; 4];
    for byte in &mut version {
        let part = parts.next()?;
        if !part.bytes().all(|digit| digit.is_ascii_digit())
            || (part.len() > 1 && part.starts_with('0'))
        {
            return None;
        }
        *byte = part.parse().ok()?;
    }
    (parts.next().is_none() && version[..2] == [0, family]).then_some(version)
}
