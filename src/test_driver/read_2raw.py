import serial
import struct
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200
SENSOR_IDS = [1, 2]

START_REG = 0x20
REG_QTY = 8
RESP_LEN = 21

SER_TIMEOUT = 0.5
REQUEST_GAP = 0.03      # nghỉ giữa 2 cảm biến
RETRY = 3
LOOP_DELAY = 0.1

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

def read_16ch_once(ser, sid):
    req = build_req(sid, START_REG, REG_QTY)

    # dọn byte rác cũ còn sót
    _ = ser.read_all()

    ser.write(req)
    ser.flush()

    resp = read_exact_with_timeout(ser, RESP_LEN, SER_TIMEOUT)

    if len(resp) != RESP_LEN:
        raise TimeoutError(f"Khong du {RESP_LEN} byte phan hoi, nhan {len(resp)} byte: {resp.hex(' ')}")

    if not check_crc(resp):
        raise ValueError(f"CRC sai: {resp.hex(' ')}")

    if resp[0] != sid or resp[1] != 0x03 or resp[2] != 16:
        raise ValueError(f"Khung khong hop le: {resp.hex(' ')}")

    return list(resp[3:19]), resp

def read_16ch(ser, sid):
    last_err = None
    for _ in range(RETRY):
        try:
            return read_16ch_once(ser, sid)
        except Exception as e:
            last_err = e
            time.sleep(REQUEST_GAP)
    raise last_err

def line_for_sensor(sid, ch):
    vals = " ".join([f"{v:3d}" for v in ch])
    return f"ID {sid}: {vals}"

def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=0.05
    )

    try:
        print("CH01 CH02 CH03 CH04 CH05 CH06 CH07 CH08 CH09 CH10 CH11 CH12 CH13 CH14 CH15 CH16")
        while True:
            for sid in SENSOR_IDS:
                try:
                    ch, raw = read_16ch(ser, sid)
                    print(line_for_sensor(sid, ch))
                    print(f"RAW {sid}: {raw.hex(' ')}")
                except Exception as e:
                    print(f"ID {sid}: Loi - {e}")

                time.sleep(REQUEST_GAP)

            print("-" * 80)
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()