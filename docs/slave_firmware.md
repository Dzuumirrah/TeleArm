# TeleArm slave firmware

Status: tested in a PC simulation (`tests/host/build_and_run_slave.sh`, 171 checks) and compiled with the real
ESP32 Arduino core 2.0.17 + FastAccelStepper 2.0.1 headers. **Not yet linked/flashed with PlatformIO or run on the board.**
Mechanical numbers are placeholders: see `docs/slave_mechanism_default.png`.

## Files

| Path | Content |
|---|---|
| `lib/configs/configurations.*` | new "Slave side" section: pins, drive train, homing, speeds, closed loop, PC output |
| `lib/actuation/actuation.h` | hardware API in raw units (microsteps, encoder counts) |
| `lib/actuation/actuators.cpp` | A4988 STEP/DIR via FastAccelStepper (hardware step pulses), EN#, limit switches, E_BTN |
| `lib/actuation/encoders.cpp` | TCA9548A channel select + AS5600 STATUS / RAW ANGLE |
| `lib/slave_session/slave_session.*` | 200 Hz loop: ESP-NOW receive, homing, following, encoder check, E-stop, faults, PC output |
| `src/slave/slave.cpp` | `setup()` / `loop()` only |
| `platformio.ini` | new env `slave` (`pio run -e slave -t upload`) |
| `tests/host/sim_actuation.cpp`, `sim_slave.h`, `test_slave.cpp`, `build_and_run_slave.sh` | simulated hardware + tests |

## Wiring as read by the firmware (Schematic_Slave rev 1.0 + your bodges)

| Signal | GPIO | Logic |
|---|---|---|
| STEP / DIR joint 0 base (U2) | 23 / 19 | |
| STEP / DIR joint 1 shoulder (U3) | 18 / 17 | |
| STEP / DIR joint 2 elbow (U4) | 16 / 4 | |
| A4988 EN# (all, bodge) | 32 | LOW = enabled. Driven LOW after init and kept LOW (also in E-stop/fault: holding torque) |
| LS1 / LS2 / LS3 | 25 / 26 / 27 | C -> LS, NC -> V, R‖C to GND: **HIGH = free, LOW = pressed or wire broken** |
| E_BTN | 33 | C -> 3V3, NC -> E_BTN, 10k down: **HIGH = normal, LOW = pressed or wire broken** |
| I2C SDA / SCL | 21 / 22 | 400 kHz, TCA9548A at 0x70 (RST pulled up on the module), AS5600 on channels 0/1/2 |

## Behaviour

```
power on ── 0.3 s ──> HOMING ──> RUN (follow master) 
                         ^          │ limit switch / stall
   E_BTN released 1 s ───┘          v
        ESTOP  <── E_BTN pressed (from any state, ≤ 10 ms) ── FAULT
```
- **Homing** (boot, and every time E_BTN is released for 1 s): joints one after another, order elbow → shoulder → base.
  Per joint: if already on the switch, move off it; SEEK toward the switch at 15 °/s; stop; LATCH away at 3 °/s until it
  releases → that point is `HOME_SWITCH_DEG`; PARK at `READY_DEG` (30 °/s). After all three: RUN.
- **RUN**: every JointPacket with `PKT_F_CALIBRATED` sets the targets (clamped to `JOINT_MIN/MAX_DEG`), moved with a
  trapezoid ramp (max 120 °/s, 300 °/s²). No packet for 300 ms → decelerate and hold; following resumes with the next packet.
- **E_BTN pressed**: all steppers stop immediately, drivers stay enabled (arm does not fall), master commands ignored.
  Releasing it starts a new homing. Pressing it is also the only way out of FAULT.
- **FAULT** (steppers stop, hold): a limit switch while running, a switch not found / not releasing during homing,
  a park move that does not finish, encoder vs step count > 8 ° (stall).
- **AS5600 closed loop**: after a joint is homed, its park move checks the encoder: direction is detected automatically,
  and the encoder must turn 1 ± 0.25 × the commanded motion (otherwise it is ignored and the joint runs open loop — this
  catches a wrong `MICROSTEPS`/J3 setting or a wrong ratio). At standstill, an error > 0.8 ° that lasts 200 ms is
  corrected by resetting the step counter from the encoder and moving back to the target.
  This replaces a PID: a stepper already follows its step counter; the encoder only has to catch lost steps.

## PC output (USB, 115200)

`PC_OUTPUT_BINARY = false` (default, for bring-up): master log/status lines as before (`[M ...]`), slave messages
`[S 12.345 I] ...`, and a 10 Hz line
`S RUN cmd=.. pos=.. enc=.. ls=000 es=0 link=ok mseq=.. ok=.. lost=.. bad=..`.

`PC_OUTPUT_BINARY = true` (for PC software): only frames with start 0xA5 and CRC-8 (poly 0x07):
every master frame forwarded unchanged (JOINT 1 with both quaternions, LOG 2, STATUS 3), `SlaveReport` (type 0x10, 50 Hz,
see `slave_session.h`) and slave messages as type 0x11 (LOG layout).

## Bring-up order

1. Set the A4988 current limit (Vref) before connecting motors.
2. Power up **holding E_BTN** → state ESTOP, nothing moves. Check the status line: `ls` changes when you press each switch,
   `es=1`, `enc` changes when you turn each motor by hand (relative degrees until homed). Check the printed slave MAC against `SLAVE_MAC`.
3. Release E_BTN with a hand ready on it: the elbow must move **toward** its switch. If it moves away, press E_BTN and flip
   `DIR_INVERT[2]`; repeat for shoulder and base.
4. Read the "encoder verified" / "encoder moved N× the step motion" lines after each joint parks.
5. Measure and set `HOME_SWITCH_DEG` (joint angle where each switch releases), `GEAR_RATIO`, `READY_DEG`, limits.

## Decisions taken without your answer (please check)

1. Mechanism defaults: 20T→60T gears (3:1), AS5600 on the motor rear shaft, 1/16 microstep, 200-step motors.
2. Switch positions: all on the negative side, just outside the soft limits (base −100 °, shoulder −40 °, elbow −10 °).
3. Homing order elbow → shoulder → base, sequential, each joint parks at the work-centre pose before the next starts.
4. E_BTN: homing starts after the button is **released** (not while pressed), so the arm never moves while it is held.
5. Drivers never disabled by firmware (your earlier decision: gravity). EN# only keeps them off during pin setup.
6. Joint angle convention identical to the master's IK output (`cartesian.cpp`); elbow motor assumed at the elbow (relative angle).
7. Slave clamps targets to the shared `JOINT_MIN/MAX_DEG` and has its own speed/accel limit on top of the master's slew limit.
8. `tests/host/build_and_run.sh` (master) still references `src/main.cpp`, which moved to `src/master/master.cpp`; not changed.
