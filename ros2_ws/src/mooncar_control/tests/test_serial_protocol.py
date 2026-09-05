import binascii
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from serial_protocol import COMMAND, TELEMETRY, TELEMETRY_DATA, COMMAND_DATA, Decoder, SerialLink, frame


class ProtocolTests(unittest.TestCase):
    def test_ros_bridge_arm_timeout_and_encoder_wrap(self):
        import math
        import rclpy
        from geometry_msgs.msg import Twist
        from std_srvs.srv import SetBool
        from serial_bridge import SerialBridge
        master, slave = pty.openpty()
        os.set_blocking(master, False)
        rclpy.init(args=['--ros-args', '-p', 'port:=' + os.ttyname(slave),
                        '-p', 'encoder_ticks_per_rev:=1000'])
        node = None
        def command_packet():
            decoder = Decoder()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if select.select([master], [], [], 0.05)[0]:
                    for kind, sequence, payload in decoder.feed(os.read(master, 4096)):
                        if kind == COMMAND and sequence == node.link.sequence:
                            return COMMAND_DATA.unpack(payload)
            self.fail('Latest command was not delivered through the PTY')

        try:
            node = SerialBridge()
            self.assertFalse(node.armed)
            for tick in (2147483646, -2147483647):
                os.write(master, frame(TELEMETRY, 1, TELEMETRY_DATA.pack(*([tick]*4), 12000, 0)))
                self.assertTrue(select.select([slave], [], [], 1.0)[0])
                node.update()
            self.assertAlmostEqual(node.positions[0], 3*2*math.pi/1000)
            response = node.on_arm(SetBool.Request(data=True), SetBool.Response())
            self.assertTrue(response.success)
            command = Twist()
            command.linear.x = 0.2
            node.on_command(command)
            node.update()
            self.assertEqual(command_packet(), (200, 0, 1))
            node.command_time -= 1
            node.update()
            self.assertEqual(command_packet(), (0, 0, 0))
        finally:
            if node is not None:
                node.link.close()
                node.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)

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
