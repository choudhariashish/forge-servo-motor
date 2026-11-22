# forge-servo-motor

Python driver and web UI for XiaoR XR3220 serial bus servos
(Feetech SCS protocol, 1 Mbaud TTL half-duplex) over a CH341 USB
debug cable (`/dev/ttyUSB*`).

## Requirements

- Python 3 with `pyserial` (`sudo apt install python3-serial`
  or `pip install pyserial`)
- User in the `dialout` group (or `sudo chmod a+rw /dev/ttyUSB0`)
- One or more bus servos daisy-chained to the debug board, powered
  (6–12 V depending on model)

## Running the web server

```bash
cd forge-servo-motor
python3 server.py                    # http://localhost:8001
python3 server.py --port 8080 --serial /dev/ttyUSB1   # custom port/device
```

Open the printed URL in a browser. If the cable re-enumerates
(`ttyUSB0` → `ttyUSB1`, etc.) the server reconnects automatically —
no restart needed.

## Using the web UI

### Servos tab

Each detected servo gets its own tile within ~2 seconds of being
connected (a background thread scans bus IDs 0–253):

- **Dial + slider** — set target position 0–4095 (12-bit magnetic
  encoder, ~0–360°). Dragging the slider moves the servo live.
- **Actual / V / °C** — live feedback polled every 400 ms: real
  position, supply voltage, temperature.
- **move time (ms)** — how long the servo takes to reach the target.
- **new ID + Set** — renumber the servo (0–253). Only shown on the
  ID 1 tile, and only works with one servo connected.
- **Torque: ON/OFF** — release the motor to hand-position the horn;
  the position readout still tracks.
- **0°–360° presets** — quick jump buttons.
- Tiles dim with an "offline" tag if a servo stops answering.

### Script tab

A Python editor that executes against the live bus, with
**▶ Run** / **■ Stop** buttons and an output console. The script
auto-saves in the browser.

Available helpers:

| Function | Description |
|---|---|
| `move(id, pos, time_ms)` | goal position 0–4095 |
| `pos(id)` `volts(id)` `temp(id)` | read position / voltage / temp |
| `get_speed(id)` `load(id)` `current(id)` | measured speed, load, mA |
| `torque(id, on)` | holding torque on/off |
| `wheel_mode(id, speed)` | continuous-rotation mode (−32767..32767, <0 = rev) |
| `servo_mode(id)` | back to position mode |
| `status(id)` | all feedback as a dict |
| `ids()` | list of online servo IDs |
| `sleep(s)` `running()` `print(...)` | flow control |

Example:

```python
for _ in range(3):
    for sid in ids():
        move(sid, 2048, 800)
    sleep(1.0)
    for sid in ids():
        move(sid, 1024, 800)
    sleep(1.0)
print("done:", [pos(s) for s in ids()])
```

> Scripts run arbitrary Python on the host — keep the server on
> localhost or a trusted network.

## Command-line usage (servo.py)

Stop the web server first — only one process can hold the serial port.

```bash
python3 servo.py scan                  # find servo IDs on the bus
python3 servo.py status                # all feedback for ID 1
python3 servo.py move 2048 --time 1000 # position 0..4095
python3 servo.py wheel 2000            # constant-speed mode
python3 servo.py torque off            # release the horn
python3 servo.py set-id 7              # renumber (ONE servo only)
python3 servo.py sweep                 # demo sweep
```

## Protocol notes

- Packets: `FF FF | ID | LEN | CMD/ERR | params | checksum`,
  checksum `~(ID+LEN+CMD+params) & 0xFF`, broadcast ID `0xFE`.
- ID register is at address **5**, baud code at **6** (0 = 1 Mbaud);
  EPROM lock register **55** must be cleared before ID writes.
- SRAM map (goal 42, speed 46, torque-enable 40, position 56, etc.)
  follows the Feetech STS layout, little-endian.
