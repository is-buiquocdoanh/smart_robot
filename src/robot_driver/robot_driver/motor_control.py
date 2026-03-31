import math
from typing import Optional, Tuple

from .odom import Odometry, OdomState
from .usb_can_a import USBCanA, CanFrame


# =========================
# Configuration / constants
# =========================
PORT = "/dev/usbcan"
SERIAL_BAUD = 2_000_000
# How long to wait for replies in transact() (seconds). A larger value
# increases reliability but can block the control loop longer.
RESPONSE_TIMEOUT = 0.3

LEFT_ID = 0x01
RIGHT_ID = 0x02

WHEEL_RADIUS = 0.074   # m
WHEEL_BASE = 0.30     # m
ENCODER_CPR = 10000   # count per revolution

MAX_VX = 1.0          # m/s
MAX_OMEGA = 3.0       # rad/s
MAX_RPM = 3000

# Gearbox ratio (motor revolutions : wheel revolutions).
# Default to 35.0 for a 1:35 planetary gearbox (motor turns 35x per wheel rev).
GEAR_RATIO = 35.0

# Inversion flags for motor direction. Changeable at runtime via helpers
# or via driver node ROS parameters.
LEFT_INVERT = False
RIGHT_INVERT = False

_INVERT_MAP = {
    LEFT_ID: LEFT_INVERT,
    RIGHT_ID: RIGHT_INVERT,
}


def set_inversion(node_id: int, invert: bool) -> None:
    _INVERT_MAP[node_id] = bool(invert)


def set_inversions(left: bool = False, right: bool = False) -> None:
    _INVERT_MAP[LEFT_ID] = bool(left)
    _INVERT_MAP[RIGHT_ID] = bool(right)


def apply_inversion(node_id: int, rpm: int) -> int:
    return -rpm if _INVERT_MAP.get(node_id, False) else rpm


def set_gear_ratio(r: float) -> None:
    """Set the global GEAR_RATIO used for conversions (motor revs per wheel rev)."""
    global GEAR_RATIO
    GEAR_RATIO = float(r)


def hex_bytes(s: str) -> bytes:
    return bytes.fromhex(s)


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(value, vmax))


def int16_to_hi_lo(value: int) -> Tuple[int, int]:
    v = value & 0xFFFF
    return (v >> 8) & 0xFF, v & 0xFF


def print_tx_rx(label: str, tx: bytes, rx: Optional[CanFrame]) -> None:
    # Small helper to print transmitted and received frames for debugging.
    print(f"\n[{label}]")
    print("TX:", " ".join(f"{b:02X}" for b in tx))
    if rx is None:
        print("RX: <timeout/no frame>")
    else:
        print("RX:", rx)


def transact(dev: USBCanA, node_id: int, data: bytes, label: str = "") -> Optional[CanFrame]:
    """
    Send a frame and wait for a response.

    This is the blocking, round-trip helper used by the CLI test mode.
    In a ROS real-time loop you should avoid calling transact() every
    tick because it will block up to RESPONSE_TIMEOUT.
    """
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


def run_speed_rpm(dev: USBCanA, node_id: int, rpm: int, wait_response: bool = True) -> Optional[CanFrame]:
    """Set motor speed in RPM.

    If wait_response is True (default) this will send the frame and wait for a response
    (the original blocking behavior). If wait_response is False it will only send the
    frame (non-blocking) and return None immediately. This is useful for real-time
    loops where waiting for a response each command can cause delays.
    """
    if not -32768 <= rpm <= 32767:
        raise ValueError("rpm must fit int16")
    hi, lo = int16_to_hi_lo(rpm)
    data = bytes([0x00, 0x1A, 0x10, hi, lo, 0x00, 0x00, 0x01])
    label = f"RUN SPEED {rpm} RPM ID=0x{node_id:02X}"
    if wait_response:
        return transact(dev, node_id, data, label)
    else:
        # Non-blocking send: useful for control loops. We still print the TX
        # for debugging but do not wait for a response.
        tx = dev.send_frame(node_id, data)
        print_tx_rx(label, tx, None)
        return None


def stop_motor(dev: USBCanA, node_id: int) -> Optional[CanFrame]:
    return transact(dev, node_id, hex_bytes("00 1A 00 00 00 FF 00 00"), f"STOP MOTOR ID=0x{node_id:02X}")


def read_encoder(dev: USBCanA, node_id: int) -> Optional[int]:
    """
    Read the 32-bit absolute encoder position from the motor controller.

    The response frame is decoded into a signed 32-bit value.
    Returns None on timeout or unexpected reply format.
    """
    rx = transact(dev, node_id, hex_bytes("00 2A E8 00 00 E9 00 00"), f"READ ENCODER ID=0x{node_id:02X}")
    if rx is None:
        return None

    # Expected data format: DLC=8 and response type code in rx.data[1].
    # Historically we expected 0x2B here, but some firmware/bridge versions
    # reply with 0x1B. Accept both to be more robust.
    if len(rx.data) != 8 or rx.data[1] not in (0x2B, 0x1B):
        # Print unexpected frame for debugging (keeps behavior non-fatal).
        print("\n[WARN] Unexpected READ ENCODER response:", rx)
        return None

    high16 = (rx.data[3] << 8) | rx.data[4]
    low16 = (rx.data[6] << 8) | rx.data[7]
    pos32 = (high16 << 16) | low16

    # convert unsigned->signed 32-bit
    if pos32 & 0x80000000:
        pos32 -= 0x100000000

    return pos32


def initialize_driver(dev: USBCanA, node_id: int) -> None:
    # Sequence of commands to set the controller into PC control + speed mode.
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

    # Convert wheel linear speeds to wheel RPM, then to motor RPM via gearbox
    wheel_rpm_left = linear_to_rpm(v_left)
    wheel_rpm_right = linear_to_rpm(v_right)

    motor_rpm_left = wheel_rpm_left * GEAR_RATIO
    motor_rpm_right = wheel_rpm_right * GEAR_RATIO

    rpm_left = int(round(motor_rpm_left))
    rpm_right = int(round(motor_rpm_right))

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

            # Here in CLI mode we use blocking calls to display encoder values
            run_speed_rpm(dev, LEFT_ID, apply_inversion(LEFT_ID, rpm_left))
            run_speed_rpm(dev, RIGHT_ID, apply_inversion(RIGHT_ID, rpm_right))

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