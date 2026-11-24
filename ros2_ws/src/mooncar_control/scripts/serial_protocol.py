"""UART v1: A5 5A, version, type, sequence u16, length u16, payload, CRC16."""
import binascii
import math
import struct
import time

import serial

COMMAND, TELEMETRY = 1, 2
HEADER = struct.Struct('<2sBBHH')
COMMAND_DATA = struct.Struct('<hhB')
TELEMETRY_DATA = struct.Struct('<4iHH')


def frame(kind, sequence, payload):
    if len(payload) > 64:
        raise ValueError('UART payload exceeds 64 bytes')
    data = HEADER.pack(b'\xa5\x5a', 1, kind, sequence & 0xffff, len(payload)) + payload
    return data + struct.pack('<H', binascii.crc_hqx(data, 0xffff))


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= HEADER.size:
            if self.buffer[:2] != b'\xa5\x5a':
                del self.buffer[0]
                continue
            _, version, kind, seq, size = HEADER.unpack_from(self.buffer)
            if version != 1 or size > 64 or kind not in (COMMAND, TELEMETRY):
                del self.buffer[0]
                self.errors += 1
                continue
            total = HEADER.size + size + 2
            if len(self.buffer) < total:
                break
            packet = self.buffer[:total]
            if binascii.crc_hqx(packet[:-2], 0xffff) != struct.unpack('<H', packet[-2:])[0]:
                del self.buffer[0]
                self.errors += 1
                continue
            messages.append((kind, seq, bytes(packet[HEADER.size:-2])))
            del self.buffer[:total]
        return messages


class SerialLink:
    def __init__(self, port, baudrate=115200):
        self.port = serial.Serial(port, baudrate=baudrate, timeout=0,
                                  write_timeout=0.05, exclusive=True)
        self.decoder = Decoder()
        self.sequence = 0
        self.last_received = 0.0
        self.telemetry = None
        self.last_ack = None
        self.send(0.0, 0.0, False)

    def read(self):
        updates = []
        for kind, ack, payload in self.decoder.feed(self.port.read(4096)):
            if kind == TELEMETRY and len(payload) == TELEMETRY_DATA.size:
                self.telemetry = TELEMETRY_DATA.unpack(payload)
                self.last_received = time.monotonic()
                self.last_ack = ack
                updates.append(self.telemetry)
        return updates

    def healthy(self):
        return (self.telemetry is not None and time.monotonic() - self.last_received < 0.5
                and self.telemetry[-1] == 0)

    def send(self, linear, angular, armed):
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear = angular = 0.0
            armed = False
        armed = bool(armed and self.healthy())
        linear = max(-0.35, min(0.35, linear)) if armed else 0.0
        angular = max(-0.8, min(0.8, angular)) if armed else 0.0
        self.sequence = (self.sequence + 1) & 0xffff
        packet = frame(COMMAND, self.sequence, COMMAND_DATA.pack(
            round(linear*1000), round(angular*1000), int(armed)))
        if self.port.write(packet) != len(packet):
            raise serial.SerialTimeoutException('Incomplete command frame write')

    def close(self):
        try:
            self.send(0.0, 0.0, False)
        finally:
            self.port.close()
