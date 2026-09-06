import runpy
import unittest
from pathlib import Path


tool = runpy.run_path(str(Path(__file__).with_name("robstride-zero.py")))


class SaveResponseTests(unittest.TestCase):
    def setUp(self):
        self.connection = object.__new__(tool["RobstrideConnection"])
        self.connection.motor_id = 51
        self.connection.host_id = 0

    def test_accepts_captured_mini_save_broadcast(self):
        self.assertTrue(
            self.connection._is_save_response(
                0x000033FE, bytes.fromhex("a99093c79c90b013")
            )
        )

    def test_accepts_documented_feedback_reply(self):
        self.assertTrue(self.connection._is_save_response(0x02003300, bytes(8)))

    def test_rejects_unrelated_or_truncated_frames(self):
        for can_id, payload in [
            (0x000034FE, bytes(8)),
            (0x00003300, bytes(8)),
            (0x18003300, bytes(8)),
            (0x02003400, bytes(8)),
            (0x02003301, bytes(8)),
            (0x000033FE, bytes(7)),
            (0x02003300, bytes(7)),
        ]:
            with self.subTest(can_id=hex(can_id), size=len(payload)):
                self.assertFalse(self.connection._is_save_response(can_id, payload))


if __name__ == "__main__":
    unittest.main()
