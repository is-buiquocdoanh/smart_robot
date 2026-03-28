import math
from typing import Optional, Tuple

from odom import Odometry, OdomState
from usb_can_a import USBCanA, CanFrame


PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2_000_000
RESPONSE_TIMEOUT = 0.3

LEFT_ID = 0x01
RIGHT_ID = 0x02

WHEEL_RADIUS = 0.05   # m
WHEEL_BASE = 0.30     # m
ENCODER_CPR = 10000   # count / vòng

MAX_VX = 1.0          # m/s
MAX_OMEGA = 2.0       # rad/s
MAX_RPM = 3000


def hex_bytes(s: str) -> bytes:
    return bytes.fromhex(s)


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(value, vmax))


def int16_to_hi_lo(value: int) -> Tuple[int, int]:
    v = value & 0xFFFF
    return (v >> 8) & 0xFF, v & 0xFF


def print_tx_rx(label: str, tx: bytes, rx: Optional[CanFrame]) -> None:
    print(f"\n[{label}]")
    print("TX:", " ".join(f"{b:02X}" for b in tx))
    if rx is None:
        print("RX: <timeout/no frame>")
    else:
        print("RX:", rx)


def transact(dev: USBCanA, node_id: int, data: bytes, label: str = "") -> Optional[CanFrame]:
    tx, rx = dev.transact(node_id, data, response_timeout=RESPONSE_TIMEOUT)
    if label:
        print_tx_rx(label, tx, rx)
    return rx


def read_status(dev: USBCanA, node_id: int) -> Optional[CanFrame]:
    return transact(dev, node_id, hex_bytes("00 2A E3 00 00 E4 00 00"), f"READ STATUS ID=0x{node_id:02X}")


def set_pc_control(dev: USBCanA, node_id: int) -> Optional[CanFrame]:
    return transact(dev, node_id, hex_bytes("00 1A 36 00 02 FF 00 00"), f"SET PC CONTROL ID=0x{node_id:02X}")


def set_speed_mode(dev: USBCanA, node_id: int) -> Optional[CanFrame]:
    return transact(dev, node_id, hex_bytes("00 1A 02 00 C4 0A 01 03"), f"SET SPEED MODE ID=0x{node_id:02X}")


def run_speed_rpm(dev: USBCanA, node_id: int, rpm: int) -> Optional[CanFrame]:
    if not -32768 <= rpm <= 32767:
        raise ValueError("rpm must fit int16")
    hi, lo = int16_to_hi_lo(rpm)
    data = bytes([0x00, 0x1A, 0x10, hi, lo, 0x00, 0x00, 0x01])
    return transact(dev, node_id, data, f"RUN SPEED {rpm} RPM ID=0x{node_id:02X}")


def stop_motor(dev: USBCanA, node_id: int) -> Optional[CanFrame]:
    return transact(dev, node_id, hex_bytes("00 1A 00 00 00 FF 00 00"), f"STOP MOTOR ID=0x{node_id:02X}")


def read_encoder(dev: USBCanA, node_id: int) -> Optional[int]:
    rx = transact(dev, node_id, hex_bytes("00 2A E8 00 00 E9 00 00"), f"READ ENCODER ID=0x{node_id:02X}")
    if rx is None:
        return None

    if len(rx.data) != 8 or rx.data[1] != 0x2B:
        return None

    high16 = (rx.data[3] << 8) | rx.data[4]
    low16 = (rx.data[6] << 8) | rx.data[7]
    pos32 = (high16 << 16) | low16

    if pos32 & 0x80000000:
        pos32 -= 0x100000000

    return pos32


def initialize_driver(dev: USBCanA, node_id: int) -> None:
    print(f"\n=== KHOI TAO DRIVER ID=0x{node_id:02X} ===")
    read_status(dev, node_id)
    set_pc_control(dev, node_id)
    set_speed_mode(dev, node_id)


def linear_to_rpm(linear_speed: float) -> float:
    wheel_omega = linear_speed / WHEEL_RADIUS
    return wheel_omega * 60.0 / (2.0 * math.pi)


def twist_to_rpm(vx: float, omega: float) -> Tuple[int, int]:
    v_left = vx - omega * WHEEL_BASE / 2.0
    v_right = vx + omega * WHEEL_BASE / 2.0

    rpm_left = int(round(linear_to_rpm(v_left)))
    rpm_right = int(round(linear_to_rpm(v_right)))

    rpm_left = int(clamp(rpm_left, -MAX_RPM, MAX_RPM))
    rpm_right = int(clamp(rpm_right, -MAX_RPM, MAX_RPM))
    return rpm_left, rpm_right


def main() -> None:
    print("[INFO] Bat dau chuong trinh motor_control")

    odom = Odometry(
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
        encoder_cpr=ENCODER_CPR,
    )

    with USBCanA(port=PORT, baudrate=SERIAL_BAUD, timeout=0.2) as dev:
        initialize_driver(dev, LEFT_ID)
        initialize_driver(dev, RIGHT_ID)

        print("\nNhap lenh theo dang: vx omega")
        print("Vi du: 0.30 0.00")
        print("Nhan Enter rong de dung.\n")

        while True:
            line = input("Nhap vx omega: ").strip()
            if line == "":
                break

            parts = line.split()
            if len(parts) != 2:
                print("[WARN] Hay nhap dung 2 so: vx omega")
                continue

            try:
                vx = float(parts[0])
                omega = float(parts[1])
            except ValueError:
                print("[WARN] Gia tri khong hop le")
                continue

            vx = clamp(vx, -MAX_VX, MAX_VX)
            omega = clamp(omega, -MAX_OMEGA, MAX_OMEGA)

            rpm_left, rpm_right = twist_to_rpm(vx, omega)

            run_speed_rpm(dev, LEFT_ID, rpm_left)
            run_speed_rpm(dev, RIGHT_ID, rpm_right)

            left_count = read_encoder(dev, LEFT_ID)
            right_count = read_encoder(dev, RIGHT_ID)

            if left_count is None or right_count is None:
                print("[WARN] Khong doc duoc encoder")
                continue

            state: OdomState = odom.update(left_count, right_count, 0.02)

            print(
                f"[CTRL] vx={vx:.3f} m/s, omega={omega:.3f} rad/s | "
                f"rpmL={rpm_left}, rpmR={rpm_right}"
            )
            print(
                f"[ODOM] x={state.x:.3f} m, y={state.y:.3f} m, theta={state.theta:.3f} rad, "
                f"vx={state.vx:.3f} m/s, omega={state.omega:.3f} rad/s, "
                f"encL={left_count}, encR={right_count}"
            )

        print("\n[INFO] Dung dong co...")
        stop_motor(dev, LEFT_ID)
        stop_motor(dev, RIGHT_ID)
        print("[INFO] Ket thuc chuong trinh.")


if __name__ == "__main__":
    main()