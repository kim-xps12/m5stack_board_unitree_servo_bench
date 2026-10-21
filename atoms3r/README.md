# AtomS3R Servo Link

Firmware for driving the J288/S288 servo from an M5Stack AtomS3R through the
[DYNAMIXEL TTL board](https://github.com/kim-xps12/m5stack_board_dynamixel_ttl_rs3485)
(MAX3485 with transistor auto-direction): a sine swing demo (`sin_swing`) and a bench
firmware with an analysis script (`bench`). In the bench, the AtomS3R doubles as the measuring instrument:
its RMT peripheral generates test patterns and records pin waveforms with 12.5 ns resolution,
so no oscilloscope is needed.

The servo packets are built and parsed by the unmodified STM32 code in
`../external/digital_servo/stm32/firmware/App/`, Unitree's repository as a git submodule
(`compat/gpio.h` stands in for the STM32 HAL header it includes).

## Wiring

| AtomS3R | Board | Note |
|---------|-------|------|
| Grove: G2, G1, 5V, GND | J2: TX, RX, +5V, GND | Straight Grove cable (TX lands on G2; checked on hardware) |
| G5 (bottom header) | J4 pin 3 (bus signal, MAX3485 A) | **Bench steps only** |

The bus swings 0–5 V when the servo is connected. Connect G5 only when the servo is
unplugged **and** J1 power is unplugged; the ESP32-S3 is not 5 V tolerant, and J4 pin 2
carries the servo supply.

## Usage

```bash
pio run -t upload                      # build and flash
python3 tools/bench.py check           # bench: wiring sanity
python3 tools/bench.py sweep           # bench: MAX3485 enable delay vs. preceding idle
python3 tools/bench.py packet          # bench: real packets at 4 / 6 Mbps on the A line
python3 tools/bench.py scan            # servo: find the ID
python3 tools/bench.py servo --id 0    # servo: error statistics
python3 tools/bench.py margin --id 0   # servo: baud-rate tolerance window
python3 tools/bench.py latency --id 0  # servo: DE release and response timing
python3 tools/first_edge.py all        # servo: tolerance to the first-edge delay, window with it cancelled
```

Default baud rate is 4 Mbps (`--baud` to change). Raw logs go to `logs/`.
Findings and the board redesign plan: [docs/ttl_board_redesign.md](docs/ttl_board_redesign.md);
explanation of the problem, the fix options and other vendors' circuits: [docs/s288_link_explained.md](docs/s288_link_explained.md) (both Japanese).
`margin` sends stop commands (mostly zero bytes), so it overstates the tolerance seen with
real, varying command data; see the sine swing results below.

## Sine Swing Demo

`src/sin_swing/` is a port of `python/sin_swing_demo.py`: the same defaults, centre capture,
amplitude ramp, speed feed-forward, `timeout=1` and `mode=0` on exit.

```bash
pio run -e sin_swing -t upload
```

Control it over the USB serial port, or with the front button (start with the last
parameters / stop):

```
start [amp=0.5] [freq=0.5] [kp=10] [kd=0.3] [duration=10] [ramp=1] [rate=200] [print=10] [id=0] [baud=3950617]
stop
status        # one mode=0 exchange, the servo does not move
```

The servo is set to 4 Mbps, but the firmware talks at 3.95 Mbps (80 MHz / (20 + 4/16)).
Measured missed replies while swinging (A = 0.5 rad, 200 Hz):

| Our baud rate | vs. 4 Mbps | Missed |
|---------------|-----------|--------|
| 4 000 000 | 0 % | ~51 % |
| 3 968 992 | −0.62 % | 30 % |
| 3 956 723 | −0.93 % | 1.7 % |
| 3 950 617 | −1.23 % | 0.08–0.4 % |
| 3 938 461 | −1.54 % | 44 % |

The window is narrow, so treat the offset as a workaround and re-check it on another servo.
A missed command is not fatal: the servo keeps the previous target until the next one.
