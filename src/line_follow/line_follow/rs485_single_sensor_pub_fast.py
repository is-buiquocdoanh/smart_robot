import struct
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial


PORT = "/dev/magnetic"
BAUD = 115200

START_REG = 0x20
REG_QTY = 8
RESP_LEN = 21

SER_TIMEOUT = 0.5
REQUEST_GAP = 0.03
RETRY = 1
PUBLISH_RATE = 20.0


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


def build_req(slave_id, start_reg, qty):
    frame = bytes([
        slave_id, 0x03,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF
    ])
    return frame + crc16_modbus(frame)


def check_crc(frame):
    return len(frame) >= 5 and frame[-2:] == crc16_modbus(frame[:-2])


def read_exact_with_timeout(ser, nbytes, timeout):
    deadline = time.monotonic() + timeout
    data = b""
    while len(data) < nbytes and time.monotonic() < deadline:
        chunk = ser.read(nbytes - len(data))
        if chunk:
            data += chunk
        else:
            time.sleep(0.001)
    return data


def read_16ch_once(ser, sid, ser_timeout):
    req = build_req(sid, START_REG, REG_QTY)

    # clear stale bytes
    _ = ser.read_all()

    ser.write(req)
    ser.flush()

    resp = read_exact_with_timeout(ser, RESP_LEN, ser_timeout)

    if len(resp) != RESP_LEN:
        raise TimeoutError(f"Not enough bytes: expected {RESP_LEN}, got {len(resp)}: {resp.hex(' ')}")

    if not check_crc(resp):
        raise ValueError(f"CRC error: {resp.hex(' ')}")

    if resp[0] != sid or resp[1] != 0x03 or resp[2] != 16:
        raise ValueError(f"Invalid frame: {resp.hex(' ')}")

    return list(resp[3:19])


class SingleSensorFast(Node):
    def __init__(self):
        super().__init__("rs485_single_sensor_pub_fast")

        # parameters
        self.declare_parameter("port", PORT)
        self.declare_parameter("baud", BAUD)
        self.declare_parameter("sensor_id", 1)
        self.declare_parameter("ser_timeout", SER_TIMEOUT)
        self.declare_parameter("request_gap", REQUEST_GAP)
        self.declare_parameter("retry", RETRY)
        self.declare_parameter("publish_rate", PUBLISH_RATE)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        self.sensor_id = int(self.get_parameter("sensor_id").get_parameter_value().integer_value)
        self.ser_timeout = float(self.get_parameter("ser_timeout").get_parameter_value().double_value)
        self.request_gap = float(self.get_parameter("request_gap").get_parameter_value().double_value)
        self.retry = int(self.get_parameter("retry").get_parameter_value().integer_value)
        self.publish_rate = float(self.get_parameter("publish_rate").get_parameter_value().double_value)

        topic = f"/sensor{self.sensor_id}/analog16"
        self.pub = self.create_publisher(UInt16MultiArray, topic, 10)

        # latest sensor values (protected by lock)
        self._lock = threading.Lock()
        self._latest = [0] * 16
        self._last_valid = 0.0

        # open serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.05,
        )

        # reader thread
        self._stop = False
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        # publisher timer at desired rate
        self._pub_timer = self.create_timer(1.0 / float(self.publish_rate), self._publish_loop)

        self.get_logger().info(f"rs485_single_sensor_pub_fast opened {port} @ {baud}, sensor_id={self.sensor_id}")

    def _reader_loop(self):
        while not self._stop and rclpy.ok():
            try:
                vals = read_16ch_once(self.ser, self.sensor_id, self.ser_timeout)
                with self._lock:
                    # vals is list of 16 bytes
                    self._latest = [int(v) for v in vals]
                    self._last_valid = time.monotonic()
            except Exception as e:
                self.get_logger().debug(f"reader error: {e}")
                # on error, wait a bit then retry
                time.sleep(self.request_gap)

    def _publish_loop(self):
        msg = UInt16MultiArray()
        with self._lock:
            msg.data = list(self._latest)
        self.pub.publish(msg)

    def destroy_node(self):
        # stop reader thread and close serial
        self._stop = True
        if hasattr(self, "_reader") and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SingleSensorFast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
