#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas and Stefanie Tellex"
__copyright__ = "Copyright 2014 Altax.net, 2017"

__license__ = "GPL"
__version__ = "1.5"

import logging
import serial, time, struct
import numpy as np
from threading import Lock

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def __str__(self):
        print(repr(self))
    def __repr__(self):
        return "PID(%f, %f, %f)" % (self.kp, self.ki, self.kd)

class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    RAW_IMU_STRUCT = struct.Struct('<hhhhhhhhh')
    POS_EST = 123
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ATTITUDE_STRUCT = struct.Struct('<hhh')
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    PID_STRUCT = struct.Struct('<BBBBBBBBBBBBBBBBBBBBBBBBBBBBBB')    
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    MULTIPLE_MSP = 230
    DEBUG = 254
    EEPROM_WRITE = 250

    SEND_ZERO_STRUCT1 = struct.Struct('<2B%dh' % 0)

    SEND_EIGHT_STRUCT1 = struct.Struct('<2B%dh' % 8)
    codeS = struct.Struct('<B')
    footerS = struct.Struct('B')
    emptyString = ""
    headerString = "$M<"

    """Class initialization"""
    def __init__(self, serPort):

        """Global variables of data"""
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'elapsed':0,'timestamp':0}
        self.posest = {'x':0,'y':0,'z':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}

        self.pid = {'roll':None, 'pitch': None, 'yaw':None, 'alt':None ,'pos':None, 'posr':None, 'navr':None, 'level':None, 'mag':None, 'vel':None}

        self.ident = {"version":"", "multitype":"", "msp_version":"", "capability":""}
        self.status = {}
        self.analog = {}
        self.multiple_msp_commands = ()
        self.receive_buffer = ""
        self.received_at = 0.
        self.boxids = []
        self.box = []
        self.elapsed = 0
        self.PRINT = 1

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

    def create_raw_rc_packet(self,channels):
        """
        Create a MSP packet for RAW_RC command with given channel values.
        channels: List of 8 integers, each between 1000 and 2000 representing RC channel values.
        """
        header = b'$M<'  # MSP header
        code = 200  # MSP code for RAW_RC
        data_length = 16  # Length of data for 8 channels, each 2 bytes

        # Pack the channels data into a byte string
        data = struct.pack('<8H', *channels)

        # Calculate the checksum: XOR of size, code, and all bytes in data
        checksum = data_length ^ code
        for byte in data:
            checksum ^= ord(byte)

        checksum &= 0xFF

        # Construct the complete packet
        packet = header + struct.pack('<BB', data_length, code) + data + struct.pack('<B', checksum)
        return packet

    def create_packet(self, data_length, code, data):
        if code == MultiWii.SET_RAW_RC:
            return self.create_raw_rc_packet(data)
        dl = len(data)
        if code == MultiWii.MULTIPLE_MSP:
            self.multiple_msp_commands = tuple(data)
            data_length = dl
            s1 = struct.Struct('<2B%dB' % dl)
        elif dl == 0:
            s1 = MultiWii.SEND_ZERO_STRUCT1
        elif dl == 8:
            s1 = MultiWii.SEND_EIGHT_STRUCT1
        else:
            s1 = struct.Struct('<2B%dh' % dl)
        data_string = s1.pack(data_length, code, *data)
        checksum = np.bitwise_xor.accumulate(np.frombuffer(data_string, dtype=np.uint8))[-1]
        return MultiWii.emptyString.join((MultiWii.headerString, data_string,
                                         MultiWii.footerS.pack(checksum), "\n"))

    """Function for sending a command to the board."""
    def send_raw_command(self, data_length, code, data):
        with self.serial_port_write_lock:
            packet = self.create_packet(data_length, code, data)
            self.ser.write(packet)
            self.logger.debug("Raw command sent: %r", packet)



    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition
      s_struct((uint8_t*)&att,6);
      break;

    """
    # def arm(self):
    #     timer = 0
    #     start = time.time()
    #     while timer < 0.5:
    #         data = [1500, 1500, 2000, 1000, 1500, 1500, 1500, 1500]
    #         self.sendCMD(16,MultiWii.SET_RAW_RC,data)
    #         time.sleep(0.05)
    #         timer = timer + (time.time() - start)
    #         start =  time.time()

    # def disarm(self):
    #     timer = 0
    #     start = time.time()
    #     while timer < 0.5:
    #         data = [1500,1500,1000,990, 1500, 1500, 1500, 1500]
    #         self.sendCMD(16,MultiWii.SET_RAW_RC,data)
    #         time.sleep(0.05)
    #         timer = timer + (time.time() - start)
    #         start =  time.time()

    """Function to receive a data packet from the board"""
    def getData(self, cmd):
        self.send_raw_command(0,cmd,[])
        return self.receiveDataPacket()

    def send_commands(self, cmds):
        with self.serial_port_write_lock:
            packet = MultiWii.emptyString.join(self.create_packet(0, c, args) for c, args in cmds)
            self.ser.write(packet)

    """ Sends a request for N commands from the board. """
    def getDataBulk(self, cmds):
        self.send_commands(cmds)
        result = []
        for c, args in cmds:
            result.append(self.receiveDataPacket())
        return result

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
            start = time.time()
            while True:
                if self.receive_buffer and not self.receive_buffer.startswith('$M'):
                    offset = self.receive_buffer.find('$M')
                    if offset < 0:
                        self.receive_buffer = '$' if self.receive_buffer.endswith('$') else ''
                    else:
                        self.receive_buffer = self.receive_buffer[offset:]
                    if self.receive_buffer != '$':
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
                        if header[:3] != '$M>' or ord(checksum) != crc:
                            return code, None
                        reply = self._decode_packet(code, data, self.received_at,
                                                    time.time() - start)
                        return code, reply
                count = needed - len(self.receive_buffer) if wait else self.ser.inWaiting()
                if count <= 0:
                    return None
                chunk = self.ser.read(count)
                if not chunk:
                    return None
                self.receive_buffer += chunk
                self.received_at = time.time()

    def _decode_packet(self, code, data, readTime, elapsed):
        datalength = len(data)
        if code == MultiWii.MULTIPLE_MSP:
            return self._decode_imu_batch(data, readTime, elapsed)
        elif code in (MultiWii.ATTITUDE, MultiWii.RAW_IMU):
            size = MultiWii.ATTITUDE_STRUCT.size if code == MultiWii.ATTITUDE else MultiWii.RAW_IMU_STRUCT.size
            if len(data) != size:
                return None
            return self._decode_imu_reply(code, data, readTime, elapsed)
        elif code == MultiWii.BOXIDS:
            temp = struct.unpack('<'+'b'*datalength,data)
            self.boxids = temp
            return self.boxids
        elif code == MultiWii.SET_BOX:
            print("data", data)
            print("len", len(data))
        elif code == MultiWii.BOX:
            assert datalength % 2 == 0
            temp = struct.unpack('<'+'H'*(datalength/2), data)
            self.box = temp
            return self.box
        elif code == MultiWii.ANALOG:
            temp = struct.unpack('<'+'B2HhH', data)
            self.analog['vbat'] = temp[0]
            self.analog['intPowerMeterSum'] = temp[1]
            self.analog['rssi'] = temp[2]
            self.analog['amperage'] = temp[3]
            self.analog['timestamp']= readTime
            return self.analog
        elif code == MultiWii.BOXNAMES:
            print("datalength", datalength)
            assert datalength % 2 == 0
            temp = struct.unpack('<'+'s' * datalength, data)
            temp = "".join(temp)[:-1].split(";")
            self.boxnames = temp
            return self.boxnames
        elif code == MultiWii.STATUS:
            print(data)
            temp = struct.unpack('<'+'HHHIb',data)
            self.status['cycleTime'] = temp[0]
            self.status['i2c_errors_count'] = temp[1]
            self.status['sensor'] = temp[2]
            self.status['flag'] = temp[3]
            self.status['global_conf.currentSet'] = temp[4]
            self.status['timestamp']= readTime
            return self.status
        elif code == MultiWii.ACC_CALIBRATION:
            print("data", data)
            print("len", len(data))

        elif code == MultiWii.IDENT:
            temp = struct.unpack('<'+'BBBI',data)
            self.ident["cmd"] = code
            self.ident["version"] = temp[0]
            self.ident["multitype"] = temp[1]
            self.ident["msp_version"] = temp[2]
            self.ident["capability"] = temp[3]
            self.ident['timestamp']= readTime

            return self.ident
        elif code == MultiWii.RC:
            temp = struct.unpack('<'+'hhhhhhhhhhhh',data)
            self.rcChannels['cmd'] = code
            self.rcChannels['roll']=temp[0]
            self.rcChannels['pitch']=temp[1]
            self.rcChannels['yaw']=temp[2]
            self.rcChannels['throttle']=temp[3]
            self.rcChannels['aux1'] = temp[4]
            self.rcChannels['aux2'] = temp[5]
            self.rcChannels['aux3'] = temp[6]
            self.rcChannels['aux4'] = temp[7]
            self.rcChannels['aux5'] = temp[8]
            self.rcChannels['aux6'] = temp[9]
            self.rcChannels['aux7'] = temp[10]
            self.rcChannels['aux8'] = temp[11]
            self.rcChannels['elapsed']= elapsed
            self.rcChannels['timestamp']= readTime
            return self.rcChannels

        elif code == MultiWii.PID:
            temp = MultiWii.PID_STRUCT.unpack(data)
            self.pid['roll']   = PID(temp[0], temp[1], temp[2])
            self.pid['pitch']  = PID(temp[3], temp[4], temp[5])
            self.pid['yaw']    = PID(temp[6], temp[7], temp[8])
            self.pid['alt']    = PID(temp[9], temp[10], temp[11])
            self.pid['pos']    = PID(temp[12], temp[13], temp[14])
            self.pid['posr']   = PID(temp[15], temp[16], temp[17])
            self.pid['navr']   = PID(temp[18], temp[19], temp[20])
            self.pid['level']  = PID(temp[21], temp[22], temp[23])
            self.pid['mag']    = PID(temp[24], temp[25], temp[26])
            self.pid['vel']    = PID(temp[27], temp[28], temp[29])
            print(self.pid)

        elif code == MultiWii.POS_EST:
            temp = struct.unpack('<'+'hhh',data)
            self.posest["cmd"] = code
            self.posest['x']=float(temp[0])
            self.posest['y']=float(temp[1])
            self.posest['z']=float(temp[2])
            self.posest['elapsed']=elapsed
            self.posest['timestamp']=time.time()
            return self.posest
        elif code == MultiWii.MOTOR:
            temp = struct.unpack('<'+'hhhhhhhh',data)
            self.motor['cmd'] = code
            self.motor['m1']=float(temp[0])
            self.motor['m2']=float(temp[1])
            self.motor['m3']=float(temp[2])
            self.motor['m4']=float(temp[3])
            self.motor['m5']=float(temp[4])
            self.motor['m6']=float(temp[5])
            self.motor['m7']=float(temp[6])
            self.motor['m8']=float(temp[7])
            self.motor['elapsed']= elapsed
            self.motor['timestamp']= readTime
            return self.motor
        elif code == MultiWii.SET_RAW_RC:
            return "Set Raw RC"
        else:
            print("No return error!: %d" % code)
            raise

    def _decode_imu_reply(self, code, data, read_time, elapsed):
        if code == MultiWii.ATTITUDE:
            roll, pitch, heading = MultiWii.ATTITUDE_STRUCT.unpack(data)
            self.attitude.update(cmd=code, angx=roll / 10.0, angy=pitch / 10.0,
                                 heading=heading, elapsed=elapsed, timestamp=read_time)
            return dict(self.attitude)
        values = MultiWii.RAW_IMU_STRUCT.unpack(data)
        self.rawIMU.update(zip(('ax', 'ay', 'az', 'gx', 'gy', 'gz'), values[:6]))
        self.rawIMU.update(cmd=code, timestamp=read_time)
        return dict(self.rawIMU)

    def _decode_imu_batch(self, data, read_time, elapsed):
        # Combined replies contain lengths and payloads in request order, without IDs.
        sizes = {MultiWii.ATTITUDE: MultiWii.ATTITUDE_STRUCT.size,
                 MultiWii.RAW_IMU: MultiWii.RAW_IMU_STRUCT.size}
        payloads, offset = [], 0
        for code in self.multiple_msp_commands:
            if offset >= len(data):
                return None
            size = ord(data[offset:offset + 1])
            offset += 1
            if size != sizes.get(code) or offset + size > len(data):
                return None
            payloads.append((code, data[offset:offset + size]))
            offset += size
        if not payloads or offset != len(data):
            return None
        return [self._decode_imu_reply(code, payload, read_time, elapsed)
                for code, payload in payloads]

    def close(self):
        self.ser.close()




    """
    Calibrate the IMU and write outputs to a config file.
    """
    def calibrate(self, fname):
        self.send_raw_command(0, MultiWii.ACC_CALIBRATION, [])
        print(self.receiveDataPacket())

        # ignore the first 200 because it takes a while to settle.
        for i in range(200):
            raw_imu = self.getData(MultiWii.RAW_IMU)
            time.sleep(0.01)
            print(raw_imu)


        raw_imu_totals = {}
        samples = 0.0
        for i in range(1000):
            raw_imu = self.getData(MultiWii.RAW_IMU)
            print(raw_imu)
            if raw_imu != None:
                for key, value in raw_imu.items():
                    raw_imu_totals.setdefault(key, 0.0)
                    raw_imu_totals[key] += value
                samples += 1
                time.sleep(0.01)

        for key, value in raw_imu_totals.items():
            raw_imu_totals[key] = raw_imu_totals[key] / samples

        print(raw_imu_totals)

        import yaml
        f = open(fname, "w")
        yaml.dump(raw_imu_totals, f)
        f.close()

    def setBoxValues(self, values=(0,7,0,0)):
        self.send_raw_command(len(values) * 2, MultiWii.SET_BOX, values)

    def eepromWrite(self):
        self.send_raw_command(0, MultiWii.EEPROM_WRITE, [])
