"""Check the actual compiled shader roster and SPIR-V entry-point structure.

The pinned GLSL compiler performs shader compilation. These CPU checks catch a
wrong stage, missing artifact, truncated output, or invalid instruction stream;
they do not replace Vulkan pipeline/device execution.
"""

from pathlib import Path
import shlex
import struct
import sys
import unittest

STAGES = {
    "chrono_sensor_vkrt.rgen.spv": 5313,
    "chrono_sensor_vkrt.rmiss.spv": 5317,
    "chrono_sensor_vkrt.rchit.spv": 5316,
    "chrono_sensor_vkrt_shadow.rmiss.spv": 5317,
    "chrono_sensor_vkrt_shadow.rahit.spv": 5315,
}


def entry_points(payload):
    if len(payload) < 20 or len(payload) % 4:
        raise ValueError("Truncated or unaligned SPIR-V")
    words = struct.unpack("<" + "I" * (len(payload) // 4), payload)
    if words[0] != 0x07230203 or words[1] != 0x00010500 or words[3] == 0 or words[4] != 0:
        raise ValueError("Expected a SPIR-V 1.5 module for the retained Vulkan 1.2 command")
    entries = []
    cursor = 5
    while cursor < len(words):
        count, opcode = words[cursor] >> 16, words[cursor] & 0xFFFF
        if not count or cursor + count > len(words):
            raise ValueError("Invalid SPIR-V instruction length")
        if opcode == 15:  # OpEntryPoint: execution model, id, null-terminated name.
            if count < 4:
                raise ValueError("Truncated entry point")
            encoded = struct.pack("<" + "I" * (count - 3), *words[cursor + 3:cursor + count])
            if b"\0" not in encoded:
                raise ValueError("Unterminated entry-point name")
            entries.append((words[cursor + 1], encoded.split(b"\0", 1)[0].decode("utf-8")))
        cursor += count
    return entries


class SensorSpirv(unittest.TestCase):
    def test_all_five_compiled_shaders_have_the_original_execution_stage(self):
        self.assertEqual({path.name for path in SHADERS}, set(STAGES))
        self.assertEqual(len(SHADERS), len(STAGES))
        for path in SHADERS:
            with self.subTest(shader=path.name):
                self.assertEqual(entry_points(path.read_bytes()), [(STAGES[path.name], "main")])

    def test_truncated_and_zero_length_instructions_are_rejected(self):
        header = struct.pack("<5I", 0x07230203, 0x00010500, 0, 10, 0)
        for payload in (b"", header + b"x", header + struct.pack("<I", 0),
                        header + struct.pack("<I", (3 << 16) | 15)):
            with self.subTest(bytes=len(payload)):
                with self.assertRaises(ValueError):
                    entry_points(payload)


if __name__ == "__main__":
    SHADERS = [Path(path) for argument in sys.argv[1:] for path in shlex.split(argument)]
    del sys.argv[1:]
    unittest.main()
