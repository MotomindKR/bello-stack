"""Resolve hand inventory against the arm deployment's actual bus bindings."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


@dataclass(frozen=True)
class Endpoint:
    interface: str
    can_id: int
    palm_frame: str


def load_endpoints(deployment: Path) -> dict[str, Endpoint]:
    settings = yaml.safe_load(deployment.read_text())
    hardware_path = (deployment.parent / settings["description"]).resolve()
    hardware = yaml.safe_load(hardware_path.read_text())
    inventory = yaml.safe_load((hardware_path.parent / "hands.yaml").read_text())
    if (hardware_path.parent / inventory["hardware"]).resolve() != hardware_path:
        raise ValueError("Hand inventory belongs to a different hardware description")

    model = ET.parse(hardware_path.parent / hardware["robot_description"])
    links = {link.attrib["name"] for link in model.findall("link")}
    buses = {bus["name"]: bus for bus in hardware["logical_buses"]}
    endpoints = {}
    addresses = set()
    for side, hand in inventory["hands"].items():
        if side not in ("left", "right") or hand["model"] != "linker_o6":
            raise ValueError(f"Unsupported hand {side!r}: {hand['model']!r}")
        bus = hand["bus"]
        if bus not in buses or buses[bus].get("bitrate", 1_000_000) != 1_000_000:
            raise ValueError(f"{side}: expected a declared 1 Mbit/s bus")
        transport = settings["bus_transports"].get(bus, {})
        interface = transport.get("interface")
        if transport.get("type") != "socketcan" or not isinstance(interface, str) or not interface:
            raise ValueError(f"{side}: {bus} needs a SocketCAN interface")
        can_id = hand["can_id"]
        if type(can_id) is not int or not 0 <= can_id <= 0x7FF:
            raise ValueError(f"{side}: expected an 11-bit CAN ID")
        if (interface, can_id) in addresses:
            raise ValueError(f"Duplicate hand address on {interface}: {can_id:#x}")
        addresses.add((interface, can_id))
        frame = hand["palm_frame"]
        if frame not in links:
            raise ValueError(f"{side}: unknown palm frame {frame!r}")
        endpoints[side] = Endpoint(interface, can_id, frame)
    if not endpoints:
        raise ValueError("Empty hand inventory")
    return endpoints
