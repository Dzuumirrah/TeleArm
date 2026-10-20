# TeleArm master firmware — design notes and decisions

See also [`ARCHITECTURE.md`](ARCHITECTURE.md) for the layer diagram and the function-by-function reference of `session.cpp`.

Status: written and tested **on a PC only** (host simulation). It has **not** been compiled with PlatformIO or run on the
ESP32 / real IMUs yet (the build sandbox could not download the PlatformIO packages). Treat the first flash as a bring-up.

## 1. What the master does

```
MPU9250 (upper arm) ─┐                                         ┌─ Kurpath axis vectors u,k,s
                     ├─ calibration ─ Madgwick ─ mounting/heading ─┤
MPU6500 (forearm)  ──┘   (bias, mag)   (9-axis / 6-axis)  (T,M)    └─ joint angles a,b,d (for monitoring)
                                                                   └─ fingertip (P frame) → robot target → IK → limits → JointPacket → ESP-NOW (50 Hz)
```
All kinematics run on the master. The slave only validates the packet and drives the actuators with `joint_deg[0..2]`.

## 2. Files

| Path | Content |
|---|---|
| `lib/configs/configurations.*` | pins (unchanged, final) + every tunable (rates, betas, timings, link lengths, limits, MAC) |
| `lib/kinematics/kinematics.h` | all types + function declarations (frames and conventions are written at the top) |
| `lib/kinematics/kinematic.cpp` | vector/matrix/quaternion helpers, sensor calibration, seeding, Kurpath axis vectors and joint angles |
| `lib/kinematics/madgwick.cpp` | Madgwick filter (6- and 9-axis) |
| `lib/kinematics/calibration.cpp` | two-pose calibration, QUICK re-anchor, hinge yaw correction, gyro bias, magnetometer calibration |
| `lib/kinematics/cartesian.cpp` | fingertip, human→robot mapping, IK/FK, joint limits and slew limit |
| `lib/connections/components.cpp` | SPI buses, IMU init/read, bench offsets |
| `lib/connections/storage.cpp` | flash (NVS) storage with magic/version/checksum |
| `lib/connections/networks.cpp` | ESP-NOW, packet CRC |
| `lib/session/session.*` | 200 Hz loop, calibration state machine, button, packet building |
| `src/main.cpp` | `setup()` / `loop()` only |
| `src/slave_rx_test/slave_rx_test.cpp` | receiver test sketch for the slave ESP32 (`pio run -e slave_test`) |
| `tests/host/` | PC shim, unit tests (20 302 checks), simulated end-to-end test (1 558 checks), `build_and_run.sh` |

## 3. How to operate it

Button on GPIO26 (active LOW):

| Gesture | Action |
|---|---|
| short press (after a reboot) | **QUICK**: pose 1 only; refreshes gyro bias and re-anchors the forearm heading to the upper arm |
| hold ≥ 2 s, release | **FULL**: pose 1, then pose 2 (stores mounting + heading) |
| hold ≥ 6 s, release | **MAG** (20 s rotate the upper-arm IMU in all directions) then FULL |
| power on with the button held ≥ 2 s | **BENCH**: IMUs lying flat, z up → accelerometer/gyro offsets (stored) |
| any press during a calibration | abort |

Pose 1: arm hanging straight down, elbow straight, still. Pose 2: arm straight forward at shoulder height (horizontal),
elbow straight, still. Each pose: 3 s countdown, then 2 s hold. Progress is printed on the serial monitor (115200).
**Nothing is transmitted until both segments are calibrated and anchored in this power cycle.**

## 4. Decisions (everything I decided without a final answer from you)

**Frames and joints**
1. Kurpath P frame: X = operator's right, Y = up, Z = back. N-pose (arm hanging) k = (0,−1,0), u = (0,0,−1), s = (1,0,0).
   Madgwick world frame: x north, y west, z up. Quaternion = Hamilton, body → world.
2. Kurpath Eq. 2–4 (axis vectors) are taken from the rotation matrix of the segment quaternion instead of from Euler angles
   (identical result, no Euler singularity). Joint a = `atan2(−k1z, −k1y)`, b = `atan2(k1x, √(k1y²+k1z²))`, d = `acos(k1·k2)` (Eq. 5, 6, 10, same
   N-pose convention as the paper). Checked against a truth model: max error 0.008°.

**Yaw reference of the MPU6500 (no magnetometer)**
3. Method: each IMU gets a mounting matrix `M` and a heading matrix `T` from the two calibration poses, so the PCB orientation does not
   matter. The 9250 heading comes from the magnetometer (north), so its anchoring survives a reboot. The 6500 has no heading reference
   after a reboot: QUICK re-anchors it to the 9250 in pose 1.
4. Consequence: the P frame is tied to the compass heading you faced during FULL calibration. After FULL the operator must face the
   same way (or recalibrate). A 6-axis-only FULL (no MAG calibration) works for the current power cycle only.
5. Optional continuous correction of the 6500 heading with the elbow-hinge constraint (`USE_HINGE_YAW_CORRECTION`, **default false**).
   Simulation with a 0.4 °/s gyro-bias step: tip error grows to ~120 mm after 2 min without it, stays < 7 mm with it. Not verified on a real arm
   (a real elbow is not a perfect hinge) — your call whether to enable it after hardware tests.

**Sensors**
6. Internal sample rate 1 kHz (divider 0), DLPF 4 on gyro and accelerometer, gyro ±500 dps, acc ±2 g; the loop polls at 200 Hz (every poll reads the
   newest sample; a 200 Hz internal rate would sometimes repeat/skip samples).
7. AK8963: read through the library (`initMagnetometer`, 100 Hz continuous). The magnetometer axis order (x/y swapped, z inverted) is **detected during
   MAG calibration**, together with hard-iron offset and soft-iron per-axis scale (least-squares ellipsoid fit). Quality gates reject bad rotations.
8. Madgwick gains: 0.05 (9-axis), 0.03 (6-axis); 0.4 for 3 s after every (re)seed and during calibration poses so the filter converges quickly. A pose is only
   accepted when the gyro is still (< 3 dps std) **and** the filter agrees with the accelerometer (< 1.5° mean tilt).
9. Offsets: accelerometer/gyro offsets from the library's `autoOffsets` (BENCH), stored in flash; gyro bias is refreshed at every calibration pose 1.

**Cartesian mapping and robot**
10. Fingertip = `L_upper·k1 + L_elbow_to_tip·k2` from the shoulder (straight wrist assumed). Placeholders: 0.28 m and 0.43 m (paper Table 5 values; edit `HUMAN_*`).
11. Human → robot: human forward → robot x, human left → robot y, up → z, scaled by `MAP_SCALE` (0.5) around a robot centre point (0.30, 0, 0.10) m.
    Robot placeholders: base height 0.10 m, links 0.30 m + 0.30 m. All in `configurations.cpp`.
12. IK: analytic, elbow-up. Joint meaning sent to the slave: [0] base yaw about z, [1] shoulder angle of link 1 from vertical toward the arm's radial direction,
    [2] elbow angle of link 2 relative to link 1 (0 = straight). **Check these zero positions/signs against your servo mounting.**
    Limits `JOINT_MIN/MAX_DEG` = {−90,−30,0} / {90,120,150} (placeholders) and a slew limit of 180 °/s. An unreachable target stretches/folds the arm toward it and sets a flag.

**Communication and storage**
13. Packet (`JointPacket`, 54 bytes, packed): start 0xA5, type, seq, µs timestamp, 3 joint angles [deg] (float), both segment quaternions (for debugging), flags, CRC-8 (poly 0x07).
    Flags: calibrated, magnetometer in use, limit clamp, slew limit, unreachable, IMU fault. Sent at 50 Hz, broadcast MAC by default (`SLAVE_MAC`), channel 1.
14. If the IMUs stop answering, nothing is sent (the slave must time out and hold); the flag `IMU_FAULT` stays set for 0.5 s after recovery.
15. Flash: ESP32 NVS (`Preferences`, namespace `telearm`), blob with magic, version and FNV-1a checksum; a corrupted/foreign blob is ignored.
16. Vibration feedback is not implemented (pin driven LOW). Pins are exactly as in your `configurations.cpp`.

**Changes to your framework signatures** (the stubs were empty, so I completed them; please check they suit you)
- `IMUData.time` float → `uint32_t` (a float loses µs precision after ~16 s). `joints` members byte → float.
- `alignYawOffset_6500(quaternion, float yaw)`, `calculateAxisVector(quaternion)` → `axisVector`, `MadgwickAHRSupdate(..., beta)`, `obtainIMUData` overloaded for 6500/9250.
- `MPUInit()` now returns `bool`. `configurations.h` uses `extern const` (a plain `const` in a header gives each file its own copy).
- `platformio.ini`: master env excludes `src/slave_rx_test/`; new env `slave_test`. `hideakitai/MPU9250` is not used by any code (you may remove it).
- `disableI2CinSPI()` (in your commented dataset logger) does not exist in MPU9250_WE 1.2.17, so it is not used; the library initialises SPI at 8 MHz.

## 5. Verification

Run `MPU_LIB=<path to MPU9250_WE/src> ./tests/host/build_and_run.sh` (add `DRIFT=1` for the drift experiment). It compiles the real firmware sources against a small PC shim
(Arduino/SPI/Preferences/ESP-NOW stand-ins) and the real MPU9250_WE library headers.

- Unit tests (20 302 checks): quaternion/matrix maths, Madgwick gradient, Kurpath joints, calibration with random mountings/headings, magnetometer calibration, IK/FK.
- Simulated end-to-end (1 558 checks): fake arm + two IMUs (random mounting, gyro bias, noise, hard/soft-iron, swapped AK8963 axes, lever-arm accelerations) driving the real session code:
  nothing sent before calibration; FULL (6-axis) and MAG+FULL flows; reboot + QUICK; wrong pose, motion during hold, abort, IMU read failure, corrupted flash, bench mode;
  packet CRC, 50 Hz timing, sequence numbers. Typical result over a 30 s arm motion: joint error < 1°, fingertip error ≈ 2 mm mean / 5 mm max.

## 6. Not verified / expect to tune on hardware

- Everything on the real ESP32: SPI timing with two buses, the AK8963 over SPI, ESP-NOW range/latency, flash writes. Real magnetic disturbances (metal, motors, the welding setup) are not modelled.
- The simulation uses the same pose definitions as the code, so it cannot tell whether the pose descriptions are comfortable/repeatable for an operator
  (a sloppy pose 2 of 80° flexion + 5° abduction gives up to ~4° orientation error; 10° forearm pronation error in QUICK gives ~1.7° elbow error).
- Finger/wrist: the 6500 sits on the forearm, so the fingertip assumes a fixed offset along the forearm. Choose the finger and measure `HUMAN_ELBOW_TO_TIP_LEN`.
- Link lengths, joint limits, centre point, scale and servo zero positions are placeholders.
