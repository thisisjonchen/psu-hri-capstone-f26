#!/usr/bin/env python

import logging
import struct
import time
from threading import Lock

import serial

class MultiWii(object):
    RAW_IMU = 102
    ATTITUDE = 108
    ANALOG = 110
    SET_RAW_RC = 200
    INTEGRATED_IMU = 0x7F01
    INTEGRATED_IMU_STRUCT = struct.Struct('<BBIIII7f9h')
    ANALOG_STRUCT = struct.Struct('<B2HhH')

    def __init__(self, serPort):
        self.receive_buffer = b''
        self.received_at = 0.
        self.logger = logging.getLogger(self.__class__.__name__)

        self.ser = serial.Serial(serPort,
                                 baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=1,
                                 xonxoff=False,
                                 rtscts=False,
                                 dsrdtr=False,
                                 writeTimeout=2,
                             )
        self.serial_port_write_lock = Lock()
        self.serial_port_read_lock = Lock()

    def create_packet(self, code, data):
        if code == self.SET_RAW_RC:
            payload = struct.pack('<8H', *data)
        elif code == self.INTEGRATED_IMU:
            # MSP v2 request wrapped in an MSP v1 frame.
            payload = struct.pack('<BHHII', 0, code, 8, *data)
            payload += struct.pack('B', self.crc8(payload))
            code = 255
        elif code == self.ANALOG and not data:
            payload = b''
        else:
            raise ValueError('Unsupported MSP request: %s' % code)
        checksum = len(payload) ^ code
        for byte in bytearray(payload):
            checksum ^= byte
        packet = b'$M<' + struct.pack('<BB', len(payload), code) + payload + struct.pack('B', checksum)
        # Keep the existing battery request terminator.
        return packet + b'\n' if code == self.ANALOG else packet

    def send_commands(self, commands):
        with self.serial_port_write_lock:
            self.ser.write(b''.join(self.create_packet(code, data) for code, data in commands))

    def receiveDataPacket(self):
        packet = self._receive_packet(True)
        return packet[1] if packet is not None else None

    def receive_available(self):
        """Yield complete replies without waiting for the flight controller."""
        while True:
            packet = self._receive_packet(False)
            if packet is None:
                return
            yield packet

    def _receive_packet(self, wait):
        with self.serial_port_read_lock:
            while True:
                if self.receive_buffer and not self.receive_buffer.startswith(b'$M'):
                    offset = self.receive_buffer.find(b'$M')
                    if offset < 0:
                        self.receive_buffer = b'$' if self.receive_buffer.endswith(b'$') else b''
                    else:
                        self.receive_buffer = self.receive_buffer[offset:]
                    if self.receive_buffer != b'$':
                        self.logger.warning('Discarding bytes before an MSP reply')
                needed = 5
                if len(self.receive_buffer) >= 5:
                    needed = ord(self.receive_buffer[3:4]) + 6
                    if len(self.receive_buffer) >= needed:
                        packet = self.receive_buffer[:needed]
                        self.receive_buffer = self.receive_buffer[needed:]
                        header, data, checksum = packet[:5], packet[5:-1], packet[-1:]
                        code = ord(header[4:5])
                        crc = len(data) ^ code
                        for byte in bytearray(data):
                            crc ^= byte
                        if header[:3] != b'$M>' or ord(checksum) != crc:
                            return code, None
                        if code == 255:
                            code = self.INTEGRATED_IMU
                            if (len(data) < 6 or self.crc8(data[:-1]) != ord(data[-1:])):
                                return code, None
                            flags, command, size = struct.unpack('<BHH', data[:5])
                            if flags or command != code or size != len(data)-6:
                                return code, None
                            data = data[5:-1]
                        return code, self._decode_packet(code, data, self.received_at)
                count = needed - len(self.receive_buffer) if wait else self.ser.inWaiting()
                if count <= 0:
                    return None
                chunk = self.ser.read(count)
                if not chunk:
                    return None
                self.receive_buffer += chunk
                self.received_at = time.time()

    def _decode_packet(self, code, data, received_at):
        if code == self.INTEGRATED_IMU:
            if len(data) != self.INTEGRATED_IMU_STRUCT.size:
                return None
            v = self.INTEGRATED_IMU_STRUCT.unpack(data)
            if v[0] != 1 or v[1] not in (0, 1):
                return None
            raw = dict(zip(('ax', 'ay', 'az', 'gx', 'gy', 'gz'), v[13:19]))
            raw.update(cmd=self.RAW_IMU, timestamp=received_at)
            attitude = dict(cmd=self.ATTITUDE, angx=v[19]/10., angy=v[20]/10.,
                            heading=v[21], timestamp=received_at)
            return dict(cmd=code, valid=bool(v[1]), request=v[2], sequence=v[3], end_us=v[4], duration_us=v[5],
                        rotation=v[6:10], velocity=v[10:13], timestamp=received_at,
                        legacy=[attitude, raw])
        if code == self.ANALOG and len(data) == self.ANALOG_STRUCT.size:
            voltage, power, rssi, amperage, _ = self.ANALOG_STRUCT.unpack(data)
            return dict(vbat=voltage, intPowerMeterSum=power, rssi=rssi,
                        amperage=amperage, timestamp=received_at)
        if code == self.SET_RAW_RC:
            return 'Set Raw RC'
        return None

    @staticmethod
    def crc8(data):
        crc = 0
        for value in bytearray(data):
            crc ^= value
            for _ in range(8):
                crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
        return crc
