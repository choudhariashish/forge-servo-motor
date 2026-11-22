#!/usr/bin/env python3
"""Driver + CLI for XiaoR XR3220 serial bus servo (Feetech SCS/STS protocol).

Verified on hardware: the servo answers PING at 1,000,000 baud on
/dev/ttyUSB0 (CH341 USB debug cable), factory ID = 1.

Packet:  0xFF 0xFF | ID | LEN | INSTR/ERR | PARAM... | CHECKSUM
  LEN      = bytes after LEN (INSTR + params + checksum)
  CHECKSUM = ~(ID + LEN + INSTR + params) & 0xFF

Position is 0..4095 across the working range (12-bit magnetic encoder),
little-endian register layout (STS/SMS memory table).
"""

import argparse
import sys
import time

import serial

HEADER = 0xFF
BROADCAST_ID = 0xFE
BAUD = 1000000

# Instructions
PING = 0x01
READ = 0x02
WRITE = 0x03
REG_WRITE = 0x04      # buffered write, executes on ACTION
ACTION = 0x05
RESET = 0x06
SYNC_WRITE = 0x83

# Register addresses (verified on XR3220: SCS-style EPROM layout,
# little-endian words; SRAM area matches STS)
REG_ID = 5
REG_BAUD = 6              # 0=1M 1=500k 2=250k 3=128k 4=115200 5=76800
                          # 6=57600 7=38400 8=19200 9=14400 10=9600 11=4800
REG_MODE = 33             # 0=servo/position  1=wheel (constant speed)
REG_TORQUE_ENABLE = 40    # 0=off 1=on  (128 = damping on some models)
REG_GOAL_POS = 42         # u16
REG_GOAL_TIME = 44        # u16, ms
REG_GOAL_SPEED = 46       # u16, steps/s (bit15 = direction in wheel mode)
REG_TORQUE_LIMIT = 48     # u16, 0..1000
REG_LOCK = 55             # 0=writes go to flash too, 1=RAM only (safe for streams)
REG_PRESENT_POS = 56      # u16
REG_PRESENT_SPEED = 58    # i16
REG_PRESENT_LOAD = 60     # u16 (bit10 = direction)
REG_PRESENT_VOLTAGE = 62  # u8, units of 0.1 V
REG_PRESENT_TEMP = 63     # u8, deg C
REG_MOVING = 66           # u8
REG_PRESENT_CURRENT = 69  # i16, units of 6.5 mA


class ServoError(Exception):
    pass


def checksum(*data):
    return (~sum(data)) & 0xFF


def le16(v):
    return (v & 0xFF, (v >> 8) & 0xFF)


def u16(params):
    return params[0] | (params[1] << 8)


def s16(params):
    v = u16(params)
    return v - 65536 if v > 32767 else v


class BusServo:
    def __init__(self, port="/dev/ttyUSB0", baud=BAUD, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.reply_timeout = 2.0   # how long a real transaction waits for a reply

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- low level ------------------------------------------------------

    def _packet(self, sid, instr, params=()):
        body = bytes([sid, len(params) + 2, instr, *params])
        return bytes([HEADER, HEADER]) + body + bytes([checksum(*body)])

    def _read_packet(self):
        b = self.ser.read(1)
        while b and b[0] != HEADER:
            b = self.ser.read(1)
        if not b or self.ser.read(1) != bytes([HEADER]):
            return None
        hdr = self.ser.read(2)
        if len(hdr) < 2:
            return None
        sid, length = hdr
        rest = self.ser.read(length)
        if len(rest) < length:
            return None
        err, params, chk = rest[0], rest[1:-1], rest[-1]
        if chk != checksum(sid, length, err, *params):
            raise ServoError("bad checksum in response")
        if err:
            raise ServoError(f"servo {sid} error flags 0x{err:02x}")
        raw = bytes([HEADER, HEADER, sid, length]) + rest
        return sid, params, raw

    def _transact(self, sid, instr, params=(), expect_reply=True, wait=False):
        pkt = self._packet(sid, instr, params)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        if not expect_reply:
            return None
        # wait=False: fail fast on first timeout (bus scans).
        # wait=True:  keep reading until reply_timeout for slow servos.
        deadline = time.time() + self.reply_timeout
        while True:
            resp = self._read_packet()
            if resp is None:
                if not wait or time.time() >= deadline:
                    return None
                continue
            rsid, rparams, raw = resp
            if raw == pkt:
                ok = False        # echo of our own transmission
            elif rsid != sid:
                ok = False        # stale packet from another servo
            else:
                return rsid, rparams
            if time.time() >= deadline:
                return None

    # ---- basics ----------------------------------------------------------

    def ping(self, sid, timeout_ms=None, wait=False):
        """Ping a servo. timeout_ms gives fast scans: at 1 Mbaud a reply
        arrives in <1 ms, so ~7 ms is safe for probing absent IDs."""
        if timeout_ms is None:
            return self._transact(sid, PING, wait=wait) is not None
        old = self.ser.timeout
        self.ser.timeout = timeout_ms / 1000
        try:
            return self._transact(sid, PING, wait=wait) is not None
        finally:
            self.ser.timeout = old

    def read(self, sid, addr, length):
        resp = self._transact(sid, READ, (addr, length), wait=True)
        return resp[1] if resp else None

    def write(self, sid, addr, data, reg=False):
        """Write bytes at addr. reg=True = buffered, executes on action()."""
        self._transact(sid, REG_WRITE if reg else WRITE,
                       (addr, *data), expect_reply=False)

    def action(self, sid=BROADCAST_ID):
        self._transact(sid, ACTION, expect_reply=False)

    def scan(self, ids=range(0, 254), timeout_ms=8):
        return [i for i in ids if self.ping(i, timeout_ms=timeout_ms)]

    # ---- configuration ---------------------------------------------------

    def set_id(self, new_id, old_id=BROADCAST_ID):
        if not 0 <= new_id <= 253:
            raise ValueError("ID must be 0..253")
        self.write(old_id, REG_LOCK, (0,))      # unlock EPROM or ID write is ignored
        time.sleep(0.02)
        self.write(old_id, REG_ID, (new_id,))
        time.sleep(0.1)
        self.write(new_id, REG_LOCK, (1,))      # relock
        time.sleep(0.02)

    def set_baud(self, code, sid=BROADCAST_ID):
        """code: 0=1M 1=500k ... 10=9600. Reconnect at new rate afterwards."""
        self.write(sid, REG_BAUD, (code,))
        time.sleep(0.05)

    def torque(self, sid, on=True):
        self.write(sid, REG_TORQUE_ENABLE, (1 if on else 0,))

    def set_mode(self, sid, wheel=False):
        self.write(sid, REG_MODE, (1 if wheel else 0,))

    # ---- motion ----------------------------------------------------------

    def move(self, sid, position, time_ms=0, speed=0):
        """Goal position 0..4095. time_ms and/or speed (steps/s) shape the move;
        0 = use the other's limit / max."""
        position = max(0, min(4095, int(position)))
        self.write(sid, REG_GOAL_POS,
                   (*le16(position), *le16(int(time_ms)), *le16(int(speed))))

    def wheel(self, sid, speed):
        """Constant-speed mode. speed -32767..32767 steps/s (negative = CW)."""
        self.set_mode(sid, wheel=True)
        v = abs(int(speed)) | (0x8000 if speed < 0 else 0)
        self.write(sid, REG_GOAL_SPEED, le16(v))

    def servo_mode(self, sid):
        self.set_mode(sid, wheel=False)

    def stop(self, sid):
        self.write(sid, REG_GOAL_SPEED, le16(0))

    # ---- feedback --------------------------------------------------------

    def get_position(self, sid):
        p = self.read(sid, REG_PRESENT_POS, 2)
        return s16(p) if p else None

    def get_speed(self, sid):
        p = self.read(sid, REG_PRESENT_SPEED, 2)
        if not p:
            return None
        raw = u16(p)
        return -(raw & 0x7FFF) if raw & 0x8000 else raw

    def get_load(self, sid):
        p = self.read(sid, REG_PRESENT_LOAD, 2)
        if not p:
            return None
        v = u16(p) & 0x3FF
        return -v if u16(p) & 0x400 else v

    def get_voltage(self, sid):
        """Volts."""
        p = self.read(sid, REG_PRESENT_VOLTAGE, 1)
        return p[0] / 10 if p else None

    def get_temp(self, sid):
        """deg C."""
        p = self.read(sid, REG_PRESENT_TEMP, 1)
        return p[0] if p else None

    def get_current(self, sid):
        """mA."""
        p = self.read(sid, REG_PRESENT_CURRENT, 2)
        return s16(p) * 6.5 if p else None

    def is_moving(self, sid):
        p = self.read(sid, REG_MOVING, 1)
        return bool(p[0]) if p else None

    def status(self, sid):
        return {
            "pos": self.get_position(sid),
            "speed": self.get_speed(sid),
            "load": self.get_load(sid),
            "volts": self.get_voltage(sid),
            "temp_C": self.get_temp(sid),
            "mA": self.get_current(sid),
            "moving": self.is_moving(sid),
        }


# ---- CLI -----------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Control XR3220 Feetech-protocol bus servo")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=BAUD)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="ping IDs 0-253")
    sub.add_parser("status", help="all feedback").add_argument(
        "--id", type=int, default=1)

    sp = sub.add_parser("set-id", help="reprogram ID (connect ONE servo)")
    sp.add_argument("new_id", type=int)

    sp = sub.add_parser("move", help="goal position 0..4095")
    sp.add_argument("position", type=int)
    sp.add_argument("--id", type=int, default=1)
    sp.add_argument("--time", type=int, default=0, help="ms for the move")
    sp.add_argument("--speed", type=int, default=0, help="steps/s limit")

    sp = sub.add_parser("wheel", help="constant-speed mode")
    sp.add_argument("speed", type=int, help="-32767..32767, 0 stops")
    sp.add_argument("--id", type=int, default=1)

    sub.add_parser("servo-mode", help="back to position mode").add_argument(
        "--id", type=int, default=1)

    sp = sub.add_parser("torque", help="holding torque on/off")
    sp.add_argument("state", choices=["on", "off"])
    sp.add_argument("--id", type=int, default=1)

    sp = sub.add_parser("pos", help="read position")
    sp.add_argument("--id", type=int, default=1)

    sp = sub.add_parser("sweep", help="demo sweep 1024<->3072")
    sp.add_argument("--id", type=int, default=1)
    sp.add_argument("--cycles", type=int, default=3)

    args = p.parse_args()

    with BusServo(args.port, args.baud) as bus:
        if args.cmd == "scan":
            print("pinging IDs 0-253 ...")
            print("found:", bus.scan() or "none")

        elif args.cmd == "status":
            print(bus.status(args.id))

        elif args.cmd == "set-id":
            bus.set_id(args.new_id)
            print(f"ID set to {args.new_id}")

        elif args.cmd == "move":
            bus.move(args.id, args.position, args.time, args.speed)
            print(f"move id={args.id} -> {args.position} "
                  f"(t={args.time}ms v={args.speed})")

        elif args.cmd == "wheel":
            if args.speed == 0:
                bus.stop(args.id)
            else:
                bus.wheel(args.id, args.speed)
            print(f"wheel speed={args.speed}")

        elif args.cmd == "servo-mode":
            bus.servo_mode(args.id)
            print("servo mode")

        elif args.cmd == "torque":
            bus.torque(args.id, args.state == "on")
            print(f"torque {args.state}")

        elif args.cmd == "pos":
            print(bus.get_position(args.id))

        elif args.cmd == "sweep":
            for _ in range(args.cycles):
                for target in (1024, 3072):
                    bus.move(args.id, target, time_ms=1000)
                    time.sleep(1.1)
                    print("pos:", bus.get_position(args.id))


if __name__ == "__main__":
    try:
        main()
    except ServoError as e:
        sys.exit(f"servo error: {e}")
    except serial.SerialException as e:
        sys.exit(f"serial error: {e}")
