import binascii
import re
import struct
import unittest
from pathlib import Path


WAL = struct.Struct("<IIIIIHHBB6hI")
OFFLINE_HEADER = struct.Struct("<4s32sI")
BLOCK_HEADER = struct.Struct("<IIBHH")
WAL_MAGIC = 0x334C4157


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def make_wal(sequence: int) -> bytes:
    fields = (WAL_MAGIC, 7, sequence, 1_750_000_000 + sequence,
              sequence * 1000, 2300, 500, 1, 0,
              *([sequence] * 6))
    prefix = struct.pack("<IIIIIHHBB6h", *fields)
    return prefix + struct.pack("<I", crc32(prefix))


def parse_wal(data: bytes):
    records = []
    for offset in range(0, len(data), WAL.size):
        chunk = data[offset:offset + WAL.size]
        if len(chunk) != WAL.size:
            break
        values = WAL.unpack(chunk)
        if values[0] != WAL_MAGIC or crc32(chunk[:-4]) != values[-1]:
            break
        records.append(values[2])
    return records


class StorageProtocolTests(unittest.TestCase):
    def test_layouts_match_firmware_contract(self):
        self.assertEqual(WAL.size, 42)
        self.assertEqual(OFFLINE_HEADER.size, 40)
        self.assertEqual(BLOCK_HEADER.size, 13)
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_wal_crc_detects_every_single_bit_error(self):
        record = make_wal(1)
        self.assertEqual(parse_wal(record), [1])
        for byte_index in range(len(record)):
            for bit in range(8):
                damaged = bytearray(record)
                damaged[byte_index] ^= 1 << bit
                self.assertEqual(parse_wal(damaged), [])

    def test_partial_tail_preserves_valid_prefix(self):
        complete = b"".join(make_wal(i) for i in range(1, 6))
        for tail_bytes in range(WAL.size):
            image = complete + make_wal(6)[:tail_bytes]
            self.assertEqual(parse_wal(image), [1, 2, 3, 4, 5])

    def test_checkpoint_crash_boundaries_never_omit_identity(self):
        original = b"".join(make_wal(i) for i in range(1, 16))
        prefix_ids = set(range(1, 11))  # verified LS02 copy or confirmed live ACKs
        tail = original[10 * WAL.size:]

        # State seen at boot: (wal, recovery, next). The firmware's recovery
        # chooses the old WAL until the new WAL has reached its commit point.
        states = [
            (original, None, tail),   # before first rename
            (None, original, tail),   # between renames
            (tail, original, None),   # committed, recovery cleanup pending
            (tail, None, None),       # clean commit
        ]
        for wal, recovery, next_file in states:
            if recovery is not None and wal is None:
                recovered = recovery
            elif wal is not None:
                recovered = wal
            else:
                recovered = next_file or b""
            owned = prefix_ids | set(parse_wal(recovered))
            self.assertEqual(owned, set(range(1, 16)))

    def test_source_contains_no_automatic_format_or_destructive_clear(self):
        root = Path(__file__).resolve().parents[1]
        sources = "\n".join(path.read_text() for path in root.glob("*.cpp"))
        sources += "\n" + (root / "Line_Sight.ino").read_text()
        self.assertNotIn("LittleFS.begin(true)", sources)
        self.assertIsNone(re.search(r"\bclearRejected\w*\s*\(", sources))
        self.assertIsNone(re.search(r"action\s*==\s*\"clear_rejected\"", sources))
        self.assertIsNone(re.search(r"action\s*==\s*\"factory_reset\"", sources))


if __name__ == "__main__":
    unittest.main()
