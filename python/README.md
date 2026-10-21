# Python Scripts

Scripts that drive the servo from a PC through Unitree's USB single-wire adapter.
Packets are built and parsed by `servo_protocol.py`, which replaces Unitree's
`servo_demo.py` with the same names: it packs the target position as signed
(`servo_demo.py` raises `struct.error` on a negative target) and reads the winding
temperature and voltage as unsigned. The default port and baud rate are set there.

```bash
pip install -r requirements.txt
```

## Sine Swing Demo (position control)

`sin_swing_demo.py` swings the output shaft along a sine wave around its current
position in mode 1, with speed feed-forward and a smooth amplitude ramp.

```bash
python sin_swing_demo.py --amp 0.5 --freq 0.5 --kp 10 --kd 0.3 --duration 10
```

| Option | Default | Description |
|--------|---------|-------------|
| `--port` | `SERIAL_PORT` in `servo_protocol.py` | Serial port |
| `--id` | 0 | Servo ID |
| `--amp` / `--freq` | 0.5 rad / 0.5 Hz | Sine amplitude / frequency |
| `--kp` / `--kd` | 10 / 0.3 | Stiffness / damping gains |
| `--duration` | 10 s | Swing time (`0` = until Ctrl+C) |
| `--ramp` | 1 s | Amplitude fade-in/out time |
| `--rate` | 200 Hz | Command rate |

Commands are sent with `timeout=1`, and `mode=0` (stop) is sent on exit.

## Soft Hold Demo (low-Kp backdrive)

`soft_hold_demo.py` holds the current position in mode 1 with a low Kp, so the
output shaft can be pushed by hand like a soft spring and returns when released.
It prints deflection, torque and apparent stiffness (torque / deflection).

```bash
python soft_hold_demo.py --kp 1.0 --kd 0.05   # soft spring
python soft_hold_demo.py --kp 0 --kd 0.2      # damper only, no holding force
```

Gains are quantized: Kp resolution ≈ 0.065 Nm/rad, Kd ≈ 0.00065. Kp below one
step rounds to 0 (the script warns). Runs until Ctrl+C unless `--duration` is set.

## Return to Zero

`return_to_zero.py` visits IDs 0–14 one by one and moves each responding servo
to the **output-side absolute encoder zero (`ExPos = 0`)** along the shortest
path, then confirms arrival within `--tol` on `ExPos`. A summary table is
printed at the end. ID 15 is the broadcast ID (no response), so it is excluded.

`outputPos` is not used as the reference: it comes from the single-turn rotor
encoder plus a turn counter that resets at power-off, so its zero is just the
power-on position. Commands are still sent in `outputPos` coordinates; the
remaining `ExPos` error (gear backlash etc.) is fed back slowly after the move.
The script also stops if `ExPos` and `outputPos` turn in opposite directions.

```bash
python return_to_zero.py                     # all IDs 0-14, hold at zero afterwards
python return_to_zero.py --ids 0 1 --release # send mode 0 after reaching zero
```

Servos are skipped if `MError` is non-zero or the move exceeds `--max-dist`
(default π rad, i.e. no limit). Successful servos keep holding (`timeout=0`)
unless `--release` is given.

## Torque Test (stall torque measurement)

`torque_test.py` drives the servo with a pure torque command (mode 1, Kp=0,
Kd=0) while the **output shaft is mechanically locked**. The command ramps up
to the target and **keeps pushing until Ctrl+C** (or for `--hold` seconds).
The first Ctrl+C ramps the torque back down; a second one stops immediately.
Feedback torque, speed, voltage and temperature are recorded and summarized,
including the drop between the first and last second of the hold (heating).

```bash
python torque_test.py --torque 5              # push +5 Nm until Ctrl+C
python torque_test.py --torque 5 --hold 10    # push +5 Nm for 10 s
python torque_test.py --dir -1 --csv log.csv  # negative protocol max, save all samples
```

Without `--torque`, the target is the protocol command limit (±36.9 Nm), which
is not necessarily the servo's real peak torque.

Safety stops: shaft moves (`--max-speed`, `--max-move`), winding temperature
over `--max-temp`, non-zero `MError`. `--allow-motion` disables the motion
stop; combine it with `--kd` to cap free-spin speed at about `|torque| / Kd`
(Kd has no effect while the shaft is stalled).

```bash
python torque_test.py --torque 0.1 --hold 10 --allow-motion --kd 0.2   # free shaft, ~0.5 rad/s max
```

`outputTor` is the servo's internal
estimate — compare with an external torque sensor / load cell for true values.
