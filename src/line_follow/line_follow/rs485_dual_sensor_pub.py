import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial


PORT = "/dev/ttyUSB5"
BAUD = 115200

SENSOR_IDS = [1, 2]

START_REG = 0x20   # 16 kênh analog
REG_QTY = 8        # 8 thanh ghi = 16 byte
RESP_LEN = 21

SER_TIMEOUT = 0.5
REQUEST_GAP = 0.03
RETRY = 3
PUBLISH_RATE = 20.0  # Hz


def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


def build_req(slave_id: int, start_reg: int, qty: int) -> bytes:
    frame = bytes([
        slave_id, 0x03,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF
    ])
    return frame + crc16_modbus(frame)


def check_crc(frame: bytes) -> bool:
    return len(frame) >= 5 and frame[-2:] == crc16_modbus(frame[:-2])


def read_exact_with_timeout(ser: serial.Serial, nbytes: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    data = b""
    while len(data) < nbytes and time.monotonic() < deadline:
        chunk = ser.read(nbytes - len(data))
        if chunk:
            data += chunk
        else:
            time.sleep(0.001)
    return data


class DualSensorPublisher(Node):
    def __init__(self):
        super().__init__("rs485_dual_sensor_pub")

        self.pub1 = self.create_publisher(UInt16MultiArray, "/sensor1/analog16", 10)
        self.pub2 = self.create_publisher(UInt16MultiArray, "/sensor2/analog16", 10)

        self.ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.1
        )
        # Try enable RS485 auto-DE/RE if available on this pyserial build
        try:
            self.ser.rs485_mode = serial.rs485.RS485Settings()
        except Exception:
            # not supported on this platform/version — ignore
            pass

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)
        self.get_logger().info(f"Opened {PORT} @ {BAUD}")

    def read_16ch_once(self, sid: int):
        req = build_req(sid, START_REG, REG_QTY)

        _ = self.ser.read_all()   # dọn byte rác cũ
        self.ser.write(req)
        self.ser.flush()
        time.sleep(0.02)  # allow DE->RE switch on RS485 adapters

        # read header (slave, func, bytecount)
        header = read_exact_with_timeout(self.ser, 3, SER_TIMEOUT)
        if len(header) < 3:
            raise TimeoutError(f"ID {sid}: short header, got {len(header)} bytes")

        # if bus returns leading 0xFF bytes (idle/no device), treat as no response
        if header == b'\xff\xff\xff' or all(b == 0xFF for b in header):
            raise TimeoutError(f"ID {sid}: bus idle / no device (0xFF)")

        resp_slave, func, bytecount = header[0], header[1], header[2]

        # read data + CRC
        rest = read_exact_with_timeout(self.ser, bytecount + 2, SER_TIMEOUT)
        if len(rest) < (bytecount + 2):
            raise TimeoutError(f"ID {sid}: short data ({len(rest)}/{bytecount+2})")

        resp = header + rest

        if not check_crc(resp):
            # show the raw frame for debugging (but avoid huge spam)
            raise ValueError(f"ID {sid}: CRC sai: {resp.hex(' ')}")

        if resp[0] != sid or resp[1] != 0x03 or resp[2] != (REG_QTY * 2):
            raise ValueError(f"ID {sid}: khung khong hop le: {resp.hex(' ')}")

        return list(resp[3:3 + (REG_QTY * 2)])

    def read_16ch(self, sid: int):
        last_err = None
        for _ in range(RETRY):
            try:
                return self.read_16ch_once(sid)
            except Exception as e:
                last_err = e
                time.sleep(REQUEST_GAP)
        raise last_err

    def timer_callback(self):
        for sid, pub in zip(SENSOR_IDS, [self.pub1, self.pub2]):
            try:
                ch = self.read_16ch(sid)
                msg = UInt16MultiArray()
                msg.data = [int(v) for v in ch]
                pub.publish(msg)
            except Exception as e:
                self.get_logger().warning(str(e))

            time.sleep(REQUEST_GAP)

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
