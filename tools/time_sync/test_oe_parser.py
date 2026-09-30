import os
import struct
import tempfile
import unittest

import oe_parser


class OeParserTest(unittest.TestCase):
    def test_decodes_mannheim_pressure_and_exg_sensors(self):
        parse_info = b"embedded parse info"
        header_size = struct.calcsize("<HQIIQB") + len(parse_info)
        header = struct.pack("<HQIIQB", 0x0003, 1_000_000, header_size, len(parse_info), 123, 0)
        expected = {
            1: 101.1,
            3: 102.2,
            9: 9.9,
            10: 10.1,
            11: 11.1,
            12: 12.1,
        }
        records = b"".join(
            struct.pack("<BBQf", sensor_id, 4, 2_000_000 + sensor_id, value)
            for sensor_id, value in expected.items()
        )

        with tempfile.NamedTemporaryFile(suffix=".oe", delete=False) as file:
            filename = file.name
            file.write(header + parse_info + records)

        try:
            parsed, metadata = oe_parser.parse_oe_file(filename)
            dataframe = oe_parser.build_dataframe_from_parsed(parsed)
        finally:
            os.unlink(filename)

        self.assertFalse(metadata["truncated"])
        self.assertEqual(metadata["total_samples"], len(expected))
        self.assertEqual(
            set(dataframe.columns),
            {labels[0] for name, labels in oe_parser.LABELS.items() if name in {
                "pressure_right", "pressure_left", "exg_eartips", "exg_3_4", "exg_5_6", "exg_7_8"
            }},
        )
        for sensor_name in ("pressure_right", "pressure_left", "exg_eartips", "exg_3_4", "exg_5_6", "exg_7_8"):
            sensor_id = oe_parser.SENSOR_SID[sensor_name]
            self.assertAlmostEqual(parsed[sensor_name]["values"][0, 0], expected[sensor_id], places=4)


if __name__ == "__main__":
    unittest.main()
