import argparse
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path

import yaml

from .config import load_endpoints
from .linker_o6 import AXES, O6Hand


def main():
    parser = argparse.ArgumentParser(description="Linux SocketCAN hand control")
    parser.add_argument("--config", required=True, type=Path, help="arm deployment YAML")
    commands = parser.add_subparsers(required=True, dest="action")
    commands.add_parser("check", help="resolve hand inventory without opening CAN")
    for name in ("status", "set"):
        cmd = commands.add_parser(name)
        cmd.add_argument("side", choices=("left", "right"))
        if name == "set":
            cmd.add_argument("positions", nargs=6, type=int, help=", ".join(AXES))
    args = parser.parse_args()
    try:
        endpoints = load_endpoints(args.config)
        if args.action == "check":
            result = {side: asdict(endpoint) for side, endpoint in endpoints.items()}
        else:
            if sys.platform != "linux":
                parser.error("SocketCAN hand control requires Linux")
            with O6Hand(endpoints[args.side]) as hand:
                if args.action == "status":
                    positions, faults = hand.status()
                    result = {
                        "axes": AXES,
                        "positions": asdict(positions),
                        "faults": asdict(faults),
                    }
                else:
                    hand.set_positions(tuple(args.positions))
                    result = {"side": args.side, "target_queued": args.positions}
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError, yaml.YAMLError) as error:
        parser.exit(1, f"bello-hand: {error}\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
