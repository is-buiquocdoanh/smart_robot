import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

import serial
import time


def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_read_frame(slave_id: int, reg_addr: int, quantity: int = 1) -> bytes:
    frame = bytes([
        slave_id,
        0x04,
        (reg_addr >> 8) & 0xFF,
        reg_addr & 0xFF,
        (quantity >> 8) & 0xFF,
        quantity & 0xFF
    ])
    return frame + modbus_crc(frame)


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        # Parameters
        self.declare_parameter('port', '/dev/battery')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('read_retries', 4)
        self.declare_parameter('read_timeout', 0.5)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value
        self.retries = self.get_parameter('read_retries').value
        self.timeout = self.get_parameter('read_timeout').value

        # Serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1
        )

        # if adapter supports RS485 auto DE/RE, try enabling
        try:
            self.ser.rs485_mode = serial.rs485.RS485Settings()
        except Exception:
            # not all platforms / pyserial versions support this; ignore
            pass

        # Publisher
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)

        # Timer (đọc mỗi 0.5s)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Battery node started")

    def read_register(self, reg_addr: int):
        tx = build_read_frame(self.slave_id, reg_addr, 1)

        def read_exact(n, timeout):
            buf = b''
            end = time.time() + timeout
            while len(buf) < n and time.time() < end:
                chunk = self.ser.read(n - len(buf))
                if chunk:
                    buf += chunk
                else:
                    time.sleep(0.005)
            return buf

        for attempt in range(1, max(1, self.retries) + 1):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(tx)
                self.ser.flush()
                # give device time to respond / allow RS485 DE/RE to settle
                time.sleep(0.08)

                # read header: id(1), func(1), bytecount/exception(1)
                self.ser.timeout = self.timeout
                header = read_exact(3, self.timeout)

                # drop leading 0xFF bytes (common bus idle/noise) by sliding window
                # stop after timeout if still all 0xFF
                start_time = time.time()
                while header and header[0] == 0xFF and (time.time() - start_time) < self.timeout:
                    more = read_exact(1, self.timeout - (time.time() - start_time))
                    if not more:
                        break
                    header = header[1:] + more

                if len(header) < 3:
                    self.get_logger().debug(f"read_register attempt {attempt}: short header ({len(header)})")
                    if attempt == self.retries:
                        raise TimeoutError("Không nhận đủ dữ liệu (header)")
                    time.sleep(0.02)
                    continue

                # all 0xFF commonly means no device/line idle
                if header == b'\xff\xff\xff':
                    self.get_logger().debug(f"read_register attempt {attempt}: bus returned all 0xFF")
                    if attempt == self.retries:
                        raise TimeoutError("Thiết bị không phản hồi (0xFF)")
                    time.sleep(0.02)
                    continue

                resp_slave, func, third = header[0], header[1], header[2]

                # exception response: header already contains exception code in 'third'
                if func & 0x80:
                    rest = read_exact(2, self.timeout)  # CRC (2)
                    if len(rest) < 2:
                        raise TimeoutError("Không nhận đủ dữ liệu (exception CRC)")
                    full = header + rest
                    # CRC check
                    if full[-2:] != modbus_crc(full[:-2]):
                        self.get_logger().warn(f"CRC sai (exception) RX={full.hex(' ')} TX={tx.hex(' ')}")
                        if attempt == self.retries:
                            raise ValueError("CRC sai (exception)")
                        time.sleep(0.02)
                        continue
                    raise ValueError(f"Modbus exception code={third:02x}")

                # normal response: 'third' is bytecount
                bytecount = third
                rest = read_exact(bytecount + 2, self.timeout)  # data + CRC
                if len(rest) < (bytecount + 2):
                    self.get_logger().debug(f"read_register attempt {attempt}: short data ({len(rest)}/{bytecount+2})")
                    if attempt == self.retries:
                        raise TimeoutError("Không nhận đủ dữ liệu (data)")
                    time.sleep(0.02)
                    continue

                rx = header + rest
                # CRC verify
                if rx[-2:] != modbus_crc(rx[:-2]):
                    self.get_logger().warn(f"CRC mismatch RX={rx.hex(' ')} TX={tx.hex(' ')}")
                    if attempt == self.retries:
                        raise ValueError("CRC sai")
                    time.sleep(0.02)
                    continue

                # validate slave and function
                if resp_slave != self.slave_id or func != 0x04:
                    raise ValueError(f"Unexpected response: slave={resp_slave:02x} func={func:02x}")

                # parse value (quantity=1 -> 2 bytes)
                if bytecount < 2:
                    raise ValueError("Bytecount too small")
                value = (rx[3] << 8) | rx[4]
                return value

            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"Serial error reading register {reg_addr:#04x}: {e}")
                time.sleep(0.1)
                if attempt == self.retries:
                    raise
                continue

        # if we get here, no successful read
        raise TimeoutError("Không nhận phản hồi sau nhiều lần thử")

    def timer_callback(self):
        try:
            percent = self.read_register(0x0000)
            voltage_raw = self.read_register(0x0001)

            voltage = voltage_raw / 10.0

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.percentage = percent / 100.0   # ROS dùng 0.0 -> 1.0
            msg.voltage = voltage
            msg.present = True

            # Optional
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

            self.pub.publish(msg)

            self.get_logger().info(f"Pin: {percent}%, Voltage: {voltage:.1f}V")

        except TimeoutError as e:
            # transient / no device response — log at debug to avoid spam
            self.get_logger().debug(f"Lỗi đọc pin (timeout): {e}")
        except ValueError as e:
            # CRC or modbus exception — log warning once per timer
            self.get_logger().warn(f"Lỗi đọc pin: {e}")
        except Exception as e:
            self.get_logger().error(f"Lỗi đọc pin unexpected: {e}")

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser and getattr(self.ser, "is_open", False):
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()