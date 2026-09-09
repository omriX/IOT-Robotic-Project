# Calibration log

## Session info

- **Date:** 09.09.2026
- **Location / surface** Home floor
- **Operator:** Omri Hamani
- **Notes:** Our first measurement!

## 1. COUNTS_PER_WHEEL_REV

**What this is:** how many encoder ticks correspond to exactly one full turn of a wheel.

**How it was measured:** `zero`, then rotate the *marked* wheel exactly one full turn by hand (the other
wheel free-spins, ignore it), then `read`. Repeat 5× clockwise and 5× counter-clockwise, mixing which
single wheel is turned so both encoders get exercised.

| Trial | Direction | Wheel | count_A | count_B | Notes |
|---|---|---|---|---|---|
| 1 | CW  | Right | -4219 | 0 |  |
| 2 | CW  | Right | -4218 | 0 |  |
| 3 | CW  | Left | 0 | 4213 |  |
| 4 | CW  | Left | 0 | 4199 |  |
| 5 | CW  | Right&Left | -4193 | 4200 | Both Wheels |
| 1 | CCW | Right | 4238 | 0 |  |
| 2 | CCW | Right | 4212 | 0 |  |
| 3 | CCW | Left | 0 | -4217 |  |
| 4 | CCW | Left | 0 | -4257 |  |
| 5 | CCW | Right&Left | 4214 | -4210 | Both Wheels |

**How the final value is computed:** average the *absolute value* of every non-zero count above.

- **Average |counts| per revolution: 4216**
- **COUNTS_PER_WHEEL_REV (final): 4216**

---

## 2. WHEEL_BASE_M

**What this is:** the distance between the two wheels' contact points with the ground (track width). This
is the single most sensitive constant for how accurately the robot's heading/rotation is tracked, so it's
worth getting right with a ruler, not eyeballing it.

**What to measure:** lay a ruler across the *bottom* of the chassis and measure from the **center of the
left tire's tread** to the **center of the right tire's tread** — equivalently, the distance between the two
motor shafts, since each wheel is centered on its own shaft.

- **WHEEL_BASE_M:** 11.6cm

---

## 3. WHEEL_RADIUS_M

**What this is:** effective rolling radius of the wheel, under load, on the surface the robot will actually
drive on (carpet compresses the tyre, so this is smaller than the free/unloaded radius).

**Method A — caliper, on the carpet, under the robot's own weight:**

- Measured: **0.02 m** (20 mm)

**Method B — back-solve from a straight drive:** `zero`, `pwm BOTH <val>` to drive forward a distance `D`
that you measure with a tape measure, `stop`, `read` the ticks, then:

```
WHEEL_RADIUS_M = D / (2 * PI * ticks / COUNTS_PER_WHEEL_REV)
```
where `ticks` = average of |count_A| and |count_B|.

| Trial | D (m) | count_A | count_B | A/B mismatch | Computed radius (m) |
|---|---|---|---|---|---|
| 1 | 0.36 | 10995 | 10876 | 1.1% | 0.0221 |
| 2 | 0.343 | 10542 | 10491 | 0.5% | 0.0219 |
| 3 | 0.353 | 10789 | 10726 | 0.6% | 0.0220 |

- **WHEEL_RADIUS_M (final): 0.0220 m** 

---

## 4. Sign / side convention

**What this is:** which physical wheel is "Motor A" vs "Motor B", and whether a positive commanded value
drives each wheel *forward* or *backward*. This gets used exactly once, at the boundary where `cmd_vel`
is translated into motor commands — everything upstream of it (positive `linear.x` = forward) depends on
getting this right, and it's checked with the wheels off the ground before the robot is ever allowed to
touch the floor.

**How to measure:** prop the robot up on a block so **both wheels are off the ground**. Send `pwm A 80`,
watch the wheel spin, and note whether the tread at the top of the wheel is moving toward the **front** or
the **back** of the robot (not CW/CCW — that depends on which side you're standing on, which is ambiguous).
Then `stop`, `pwm B 80`, repeat.

| Motor | Commanded PWM | Physical wheel | Direction observed (CW/CCW) | Forward or backward? | Encoder sign matches? |
|---|---|---|---|---|---|
| A | +80 | Right | CCW | backward | Yes |
| B | +80 | Left | CW | backward | Yes |

Both motors move their wheel *backward* under a positive raw command. Since `setpoint_raw = DIR_SIGN *
desired_forward_velocity`, and a positive raw setpoint is backward on both wheels, driving forward needs a
*negative* raw setpoint — so `DIR_SIGN = -1` for both. Motor A is the right wheel (from the table), so
`RIGHT_DIR_SIGN` is A's sign and `LEFT_DIR_SIGN` is B's.

- **MOTOR_A_IS_LEFT: false** (Motor A is the right wheel, per the table above)
- **LEFT_DIR_SIGN: -1.0** (applies to Motor B)
- **RIGHT_DIR_SIGN: -1.0** (applies to Motor A)

---

## 5. MAX_TICKS_PER_INTERVAL and deadband floor

**What this is:** the fastest the wheels can turn (in ticks per 50 ms control interval — the PID's native
unit), and the lowest PWM that reliably produces motion at all (below this, PWM is wasted as heat/noise).

**How it was measured:** wheels off the ground, `ramp A` then `ramp B` — this auto-sweeps PWM 0→250→0 in
steps, logging `pwm:<val> dtick:<val>` at each step over Serial/WebSerial.

| Motor | PWM at which motion starts (deadband floor) | Max dtick/50ms observed (up-ramp, at PWM 250) | dtick right at the deadband floor (PWM 40) |
|---|---|---|---|
| A | 40 | 358 | 27 → 67 over the 200 ms step |
| B | 40 | 360 | 0 → 55 over the 200 ms step |

- **MAX_TICKS_PER_INTERVAL: 350** 
- **Deadband floor (PWM): 40**
- **Max wheel speed (m/s): ≈ 0.23 m/s** (`350 / 4216 * (2·π·0.0220) * (1000/50)`, using the radius)
- **Min moving speed (m/s): ≈ 0.04 m/s** (using the last up-ramp reading at PWM 40 — 67 for A, 55 for B,
  averaged to 61 ticks/interval — converted the same way)

---

## Battery reading sanity check

- `read` at rest: `count_A:197940 count_B:80120 dtick_A:0 dtick_B:0 battery_V:7.69`

---

## Summary

**Final constants for `robot_base/robot_config.h`:**

| Constant | Value |
|---|---|
| `COUNTS_PER_WHEEL_REV` | 4216 |
| `WHEEL_BASE_M` | 0.116 |
| `WHEEL_RADIUS_M` | 0.0220 |
| `MOTOR_A_IS_LEFT` | false |
| `LEFT_DIR_SIGN` | -1.0 |
| `RIGHT_DIR_SIGN` | -1.0 |
| `MAX_TICKS_PER_INTERVAL` | 350 |
| `SPEED_DEADBAND` | 40 (up from the existing sketches' 35) |

