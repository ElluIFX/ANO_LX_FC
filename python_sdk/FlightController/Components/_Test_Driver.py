import unittest

from ._Driver_Components import calculate_crc8
from .LDRadar_Resolver import resolve_radar_data

TEST_DATA = b"\x54\x2C\x68\x08\xAB\x7E\xE0\x00\xE4\xDC\x00\xE2\xD9\x00\xE5\xD5\x00\xE3\xD3\x00\xE4\xD0\x00\xE9\xCD\x00\xE4\xCA\x00\xE2\xC7\x00\xE9\xC5\x00\xE5\xC2\x00\xE5\xC0\x00\xE5\xBE\x82\x3A\x1A\x50"

TEST_RESULT = (
    "--- Radar Package < TS = 06714 ms > ---\n"
    "Range: 324.27° -> 334.70° 5.98rpm\n"
    "#00 Point: deg = 324.27, dist =  224, conf = 228\n"
    "#01 Point: deg = 325.22, dist =  220, conf = 226\n"
    "#02 Point: deg = 326.17, dist =  217, conf = 229\n"
    "#03 Point: deg = 327.11, dist =  213, conf = 227\n"
    "#04 Point: deg = 328.06, dist =  211, conf = 228\n"
    "#05 Point: deg = 329.01, dist =  208, conf = 233\n"
    "#06 Point: deg = 329.96, dist =  205, conf = 228\n"
    "#07 Point: deg = 330.91, dist =  202, conf = 226\n"
    "#08 Point: deg = 331.86, dist =  199, conf = 233\n"
    "#09 Point: deg = 332.80, dist =  197, conf = 229\n"
    "#10 Point: deg = 333.75, dist =  194, conf = 229\n"
    "#11 Point: deg = 334.70, dist =  192, conf = 229\n"
    "--- End of Info ---"
)


class TestRadarResolver(unittest.TestCase):
    def test_crc8(self):
        self.assertEqual(calculate_crc8(TEST_DATA[:-1]), TEST_DATA[-1])

    def test_resolving(self):
        package = resolve_radar_data(TEST_DATA)
        self.assertEqual(str(package), TEST_RESULT)


if __name__ == "__main__":
    unittest.main()
