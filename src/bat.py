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


def read_register(ser, slave_id: int, reg_addr: int, retries: int = 5, timeout: float = 0.5):
    tx = build_read_frame(slave_id, reg_addr, 1)

    for attempt in range(1, retries + 1):
        ser.reset_input_buffer()
        ser.write(tx)
        ser.flush()
        # give device time to respond
        time.sleep(0.05)

        # read header: id(1), func(1), bytecount(1)
        ser.timeout = timeout
        header = ser.read(3)
        if len(header) < 3:
            print(f"[WARN] attempt {attempt}: short header ({len(header)} bytes)")
            if attempt == retries:
                raise TimeoutError("Không nhận đủ dữ liệu (header)")
            time.sleep(0.05)
            continue

        # debug: all 0xFF likely means no device / bus idle
        if header == b'\xff\xff\xff':
            print(f"[WARN] attempt {attempt}: received all 0xFF (no device / line idle)")
            if attempt == retries:
                raise TimeoutError("Thiết bị không phản hồi (0xFF)")
            time.sleep(0.05)
            continue

        resp_slave, func, bytecount = header[0], header[1], header[2]

        # if exception response (func >= 0x80) read exception code + CRC (1 + 2)
        if func & 0x80:
            rest = ser.read(3)
            if len(rest) < 3:
                raise TimeoutError("Không nhận đủ dữ liệu (exception)")
            full = header + rest
            print("RX (exception):", full.hex(" "))
            # CRC check
            if full[-2:] != modbus_crc(full[:-2]):
                raise ValueError("CRC sai (exception)")
            raise ValueError(f"Modbus exception code={rest[0]:02x}")

        # read data bytes + CRC (bytecount + 2)
        rest = ser.read(bytecount + 2)
        if len(rest) < (bytecount + 2):
            print(f"[WARN] attempt {attempt}: short data ({len(rest)} bytes, expected {bytecount+2})")
            if attempt == retries:
                raise TimeoutError("Không nhận đủ dữ liệu (data)")
            time.sleep(0.05)
            continue

        rx = header + rest
        print("TX:", tx.hex(" "))
        print("RX:", rx.hex(" "))

        # CRC verify
        crc_calc = modbus_crc(rx[:-2])
        if rx[-2:] != crc_calc:
            print(f"[WARN] attempt {attempt}: CRC mismatch")
            if attempt == retries:
                raise ValueError("CRC sai")
            time.sleep(0.05)
            continue

        # validate slave and function
        if resp_slave != slave_id or func != 0x04:
            raise ValueError(f"Unexpected response: slave={resp_slave:02x} func={func:02x}")

        # data parsing (for quantity=1 -> 2 bytes data)
        if bytecount < 2:
            raise ValueError("Bytecount too small")
        value = (rx[3] << 8) | rx[4]
        return value

    raise TimeoutError("Không nhận phản hồi sau nhiều lần thử")


def main():
    port = "/dev/ttyUSB0"
    baudrate = 9600
    slave_id = 1

    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=1
    )

    # nếu adapter RS485 hỗ trợ, bật rs485 mode để auto DE/RE (tuỳ platform)
    try:
        ser.rs485_mode = serial.rs485.RS485Settings()
    except Exception:
        pass

    try:
        percent = read_register(ser, slave_id, 0x0000)
        voltage_raw = read_register(ser, slave_id, 0x0001)
        alarm = read_register(ser, slave_id, 0x0002)

        voltage = voltage_raw / 10.0

        print(f"Pin: {percent}%")
        print(f"Dien ap: {voltage:.1f} V")
        print(f"Alarm: {alarm}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()