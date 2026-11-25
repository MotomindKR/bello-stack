import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

from bello_hand.config import load_endpoints
from bello_hand.linker_o6 import O6Hand

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "components/bello-hardwared/config/milly-7dof-dual-o6.yaml"
VIRTUAL = CONFIG.with_name("milly-7dof-dual-o6-vcan.yaml")


class InventoryTests(unittest.TestCase):
    def test_physical_and_virtual_use_the_same_hands_and_palm_frames(self):
        for deployment, prefix in [(CONFIG, "can"), (VIRTUAL, "vcan")]:
            hands = load_endpoints(deployment)
            self.assertEqual(set(hands), {"left", "right"})
            for index, (side, can_id) in enumerate([("left", 0x28), ("right", 0x27)]):
                hand = hands[side]
                self.assertEqual((hand.interface, hand.can_id), (f"{prefix}{index}", can_id))
                self.assertEqual(hand.palm_frame, f"{side}_palm_eef")

    def test_reject_incompatible_transport(self):
        with tempfile.TemporaryDirectory() as tmp:
            deploy = yaml.safe_load(CONFIG.read_text())
            deploy["description"] = str((CONFIG.parent / deploy["description"]).resolve())
            deploy["bus_transports"]["can0"] = {"type": "gsusb"}
            path = Path(tmp) / "deployment.yaml"
            path.write_text(yaml.safe_dump(deploy))
            with self.assertRaisesRegex(ValueError, "SocketCAN"):
                load_endpoints(path)


FRAME = struct.Struct("=IB3x8s")


def send(sock, can_id, data):
    sock.send(FRAME.pack(can_id, len(data), data))


def receive(sock):
    can_id, n, data = FRAME.unpack(sock.recv(FRAME.size))
    return can_id, data[:n]


@unittest.skipUnless(os.getenv("BELLO_HAND_VXCAN") == "1", "requires isolated vxcan pairs")
class SocketCANTests(unittest.TestCase):
    def setUp(self):
        self.hands = load_endpoints(VIRTUAL)
        self.pool = ThreadPoolExecutor(1)
        self.addCleanup(self.pool.shutdown)
        self.peers = []
        for name in ("o6peer0", "o6peer1"):
            peer = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.addCleanup(peer.close)
            peer.bind((name,))
            peer.settimeout(1.5)
            self.peers.append(peer)

    def test_cli_target_uses_only_selected_bus_and_only_after_fault_check(self):
        for side, peer_index, can_id in [("left", 0, 0x28), ("right", 1, 0x27)]:
            peer = self.peers[peer_index]

            def device():
                self.assertEqual(receive(peer), (can_id, b"\x35"))
                send(peer, can_id, bytes([0x35] + [0] * 6))
                self.assertEqual(receive(peer), (can_id, bytes(range(1, 8))))

            result = self.pool.submit(device)
            args = [sys.executable, "-m", "bello_hand", "--config", str(VIRTUAL), "set", side]
            cli = subprocess.run(args + list("234567"), capture_output=True, text=True, timeout=3)
            result.result()
            self.assertEqual(cli.returncode, 0, cli.stderr)
            self.assertEqual(json.loads(cli.stdout)["target_queued"], list(range(2, 8)))
            other = self.peers[1 - peer_index]
            other.settimeout(0.03)
            with self.assertRaises(TimeoutError):
                receive(other)
            other.settimeout(1.5)

    def test_feedback_ignores_arm_frames_local_echo_and_malformed_replies(self):
        can_id = 0x28
        peer = self.peers[0]
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as local:
            local.bind(("vcan0",))

            def device():
                self.assertEqual(receive(peer), (can_id, b"\x01"))
                send(peer, socket.CAN_EFF_FLAG | can_id, bytes([1] * 7))
                send(peer, can_id + 1, bytes([1] * 7))
                send(peer, can_id, bytes([1, 255]))
                send(peer, can_id, bytes([0x35] * 7))
                send(local, can_id, bytes([1] * 7))
                self.assertEqual(receive(peer), (can_id, bytes([1] * 7)))
                send(peer, can_id, bytes([1, 2, 3, 4, 5, 6, 7]))
                self.assertEqual(receive(peer), (can_id, b"\x35"))
                send(peer, can_id, bytes([0x35] + [0] * 6))

            result = self.pool.submit(device)
            with O6Hand(self.hands["left"]) as hand:
                with self.assertRaises(OSError), O6Hand(self.hands["left"]):
                    self.fail("Two owners acquired one hand")
                positions, faults = hand.status()
            result.result()
        self.assertEqual(positions.values, (2, 3, 4, 5, 6, 7))
        self.assertEqual(faults.values, (0,) * 6)
        self.assertLess(positions.received_monotonic_ns, faults.received_monotonic_ns)

    def test_timeout_reopen_and_fault_do_not_send_target(self):
        peer = self.peers[0]
        with O6Hand(self.hands["left"]) as hand:
            with self.assertRaises(TimeoutError):
                hand.set_positions((8,) * 6)
            self.assertEqual(receive(peer), (0x28, b"\x35"))
            with self.assertRaises(RuntimeError):
                hand.set_positions((8,) * 6)

        def faulted_device():
            self.assertEqual(receive(peer), (0x28, b"\x35"))
            send(peer, 0x28, bytes([0x35, 0, 0, 0, 0, 1, 0]))

        with O6Hand(self.hands["left"]) as hand:
            peer.settimeout(0.03)
            with self.assertRaises(TimeoutError):
                receive(peer)
            with self.assertRaises(ValueError):
                hand.set_positions((256,) * 6)
            peer.settimeout(1.5)
            result = self.pool.submit(faulted_device)
            with self.assertRaisesRegex(ValueError, "active faults"):
                hand.set_positions((10,) * 6)
            result.result()
            peer.settimeout(0.03)
            with self.assertRaises(TimeoutError):
                receive(peer)
