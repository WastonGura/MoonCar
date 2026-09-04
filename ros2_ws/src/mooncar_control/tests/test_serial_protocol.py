import binascii
import os
from pathlib import Path
import pty
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from serial_protocol import COMMAND, TELEMETRY, Decoder, SerialLink, frame


class ProtocolTests(unittest.TestCase):
    def test_stream_corruption_and_fragmentation(self):
        self.assertEqual(binascii.crc_hqx(b'123456789', 0xffff), 0x29b1)
        packet = frame(COMMAND, 65535, b'abc')
        decoder = Decoder()
        bad = bytearray(packet)
        bad[-1] ^= 1
        self.assertEqual(decoder.feed(b'noise' + bad + packet[:7]), [])
        self.assertEqual(decoder.feed(packet[7:]), [(COMMAND, 65535, b'abc')])
        self.assertGreater(decoder.errors, 0)
        self.assertLess(len(decoder.buffer), 74)
        with self.assertRaises(ValueError):
            frame(TELEMETRY, 1, bytes(65))

    def test_real_c_core_over_virtual_uart(self):
        root = Path(__file__).resolve().parents[4]
        core = root / 'firmware/stm32/core'
        tests = root / 'firmware/stm32/tests'
        with tempfile.TemporaryDirectory() as temp:
            binary = str(Path(temp) / 'emulator')
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(core), str(core/'mooncar_core.c'),
                            str(tests/'emulator.c'), '-lm', '-o', binary], check=True)
            master, slave = pty.openpty()
            link = SerialLink(os.ttyname(slave))
            process = subprocess.Popen([binary, str(master)], pass_fds=(master,))
            try:
                def pump(seconds, linear=0.0, angular=0.0, armed=False):
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        link.read()
                        link.send(linear, angular, armed)
                        time.sleep(0.02)
                    link.read()

                pump(0.3)
                self.assertTrue(link.healthy())
                self.assertEqual(link.telemetry[:4], (0, 0, 0, 0))
                pump(0.7, linear=0.2, armed=True)
                self.assertTrue(all(t > 0 for t in link.telemetry[:4]))
                before = link.telemetry[:4]
                pump(0.3)
                before = link.telemetry[:4]
                pump(0.7, angular=0.5, armed=True)
                delta = [a-b for a, b in zip(link.telemetry[:4], before)]
                self.assertLess(delta[0], 0)
                self.assertGreater(delta[1], 0)
                time.sleep(0.4)
                link.read()
                self.assertEqual(link.telemetry[-1] & 2, 2)
                self.assertFalse(link.healthy())
                pump(0.3)
                self.assertTrue(link.healthy())
                process.terminate()
                process.wait(timeout=3)
                time.sleep(0.6)
                self.assertFalse(link.healthy())
            finally:
                link.close()
                process.terminate()
                process.wait(timeout=3)
                os.close(master)
                os.close(slave)


if __name__ == '__main__':
    unittest.main()
