import serial
import time
from dataclasses import dataclass


@dataclass
class CanFrame:
    can_id: int
    data: bytes


class USBCanA:

    def __init__(self, port="/dev/ttyUSB0", baudrate=2000000, timeout=0.1):

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )

        time.sleep(0.1)

    def close(self):

        if self.ser.is_open:
            self.ser.close()

    def build_packet(self, can_id, data):

        pkt = bytearray()

        pkt.append(0xAA)

        pkt.append(0xC0 | len(data))

        pkt.append(can_id & 0xFF)
        pkt.append((can_id >> 8) & 0xFF)

        pkt.extend(data)

        pkt.append(0x55)

        return bytes(pkt)

    def send_frame(self, can_id, data):

        pkt = self.build_packet(can_id, data)

        self.ser.write(pkt)

    def read_frame(self):

        while True:

            b = self.ser.read(1)

            if not b:
                return None

            if b[0] == 0xAA:
                break

        type_b = self.ser.read(1)[0]

        dlc = type_b & 0x0F

        id_l = self.ser.read(1)[0]
        id_h = self.ser.read(1)[0]

        can_id = id_l | (id_h << 8)

        data = self.ser.read(dlc)

        end = self.ser.read(1)

        if end != b'\x55':
            return None

        return CanFrame(can_id, data)

    def read_frame_by_id(self, node_id):

        while True:

            frame = self.read_frame()

            if frame is None:
                return None

            if frame.can_id == node_id:
                return frame