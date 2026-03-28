"""
USBCanA low-level serial CAN adapter helper.

This module implements a simple blocking protocol for the Waveshare
USB-CAN-A device used by the motor controllers. Key points for debugging
and performance:

- send_frame(...) writes a packet to the serial device and returns immediately.
- read_frame(...) blocks while waiting for a full frame (honors serial timeout).
- transact(...) = send_frame + read_frame and is therefore a blocking round-trip.

If you need low-latency, real-time behaviour in a ROS node, avoid calling
`transact` on every control loop tick — instead use `send_frame` and
read responses in a separate thread or at a lower frequency.

Module-level notes:
- The protocol is variable-length with header 0xAA and trailer 0x55.
- The serial timeout passed into the Serial object controls how long
    read_frame will block when waiting for bytes.
"""

import serial
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class CanFrame:
    can_id: int
    data: bytes
    extended: bool = False
    remote: bool = False

    def __str__(self) -> str:
        data_hex = " ".join(f"{b:02X}" for b in self.data)
        return f"ID=0x{self.can_id:X} DATA=[{data_hex}] EXT={self.extended} RTR={self.remote}"


class USBCanA:
    """
    Waveshare USB-CAN-A, variable-length protocol.
    Standard data frame packet:
        AA | TYPE | ID_L | ID_H | DATA... | 55
    TYPE:
        bit5 = 0 standard, 1 extended
        bit4 = 0 data frame, 1 remote frame
        bit0..3 = DLC (0..8)
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 2_000_000, timeout: float = 0.2):
        # Create the pyserial Serial object. The `timeout` here affects how
        # long read_frame will block while waiting for the header and remaining
        # bytes. If your control loop cannot tolerate blocking, create a
        # dedicated thread for serial I/O and/or reduce the timeout value.
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(0.002)  # give some time for the device to be ready

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    @staticmethod
    def _build_type_byte(dlc: int, extended: bool = False, remote: bool = False) -> int:
        if not 0 <= dlc <= 8:
            raise ValueError("DLC must be 0..8")
        t = 0xC0  # base for variable-length frame family in Waveshare examples
        # We'll rebuild explicitly from bits:
        t = 0
        if extended:
            t |= (1 << 5)
        if remote:
            t |= (1 << 4)
        t |= dlc
        # protocol expects top bits under AA envelope; examples show standard data dlc8 => C8
        t |= 0xC0
        return t

    @staticmethod
    def _encode_id(can_id: int, extended: bool = False) -> bytes:
        if extended:
            if not 0 <= can_id <= 0x1FFFFFFF:
                raise ValueError("Extended CAN ID out of range")
            return can_id.to_bytes(4, byteorder="little", signed=False)
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("Standard CAN ID out of range")
        return can_id.to_bytes(2, byteorder="little", signed=False)

    def build_packet(self, can_id: int, data: bytes, extended: bool = False, remote: bool = False) -> bytes:
        if len(data) > 8:
            raise ValueError("CAN data must be <= 8 bytes")
        type_byte = self._build_type_byte(len(data), extended=extended, remote=remote)
        pkt = bytearray()
        pkt.append(0xAA)
        pkt.append(type_byte)
        pkt.extend(self._encode_id(can_id, extended=extended))
        pkt.extend(data)
        pkt.append(0x55)
        return bytes(pkt)

    def send_frame(self, can_id: int, data: bytes, extended: bool = False, remote: bool = False) -> bytes:
        pkt = self.build_packet(can_id, data, extended=extended, remote=remote)
        self.ser.write(pkt)
        self.ser.flush()
        return pkt

    def read_frame(self, timeout: Optional[float] = None) -> Optional[CanFrame]:
        """
        Read one variable-length packet.
        Returns None on timeout/no complete frame.
        """
        old_timeout = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout

        try:
            # Find header 0xAA. This loop is blocking and will return None
            # on serial timeout. In practice this means read_frame can block
            # up to `timeout` seconds if no data arrives; plan your architecture
            # accordingly (eg. use a reader thread).
            while True:
                first = self.ser.read(1)
                if not first:
                    return None
                if first[0] == 0xAA:
                    break

            type_b_raw = self.ser.read(1)
            if not type_b_raw:
                return None
            type_b = type_b_raw[0]

            dlc = type_b & 0x0F
            extended = bool((type_b >> 5) & 0x01)
            remote = bool((type_b >> 4) & 0x01)

            id_len = 4 if extended else 2
            id_bytes = self.ser.read(id_len)
            if len(id_bytes) != id_len:
                return None

            data = self.ser.read(dlc)
            if len(data) != dlc:
                return None

            end_b = self.ser.read(1)
            if not end_b or end_b[0] != 0x55:
                return None

            can_id = int.from_bytes(id_bytes, byteorder="little", signed=False)
            return CanFrame(can_id=can_id, data=data, extended=extended, remote=remote)

        finally:
            self.ser.timeout = old_timeout

    def transact(self, can_id: int, data: bytes, response_timeout: float = 0.3) -> tuple[bytes, Optional[CanFrame]]:
        tx = self.send_frame(can_id, data)
        rx = self.read_frame(timeout=response_timeout)
        return tx, rx