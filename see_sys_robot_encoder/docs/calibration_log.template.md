# Calibration log — template

Record using `robot_calibration/robot_calibration.ino`.

## Session info

- **Date:**
- **Location / surface** (e.g. carpet, lab floor, bench):
- **Operator:**
- **Notes:**

---

## 1. COUNTS_PER_WHEEL_REV

**What this is:** how many encoder ticks correspond to exactly one full turn of a wheel.

**How to measure:** `zero`, then rotate the *marked* wheel exactly one full turn by hand (the other wheel
free-spins, ignore it), then `read`. Repeat 5× clockwise and 5× counter-clockwise, mixing which single
wheel is turned so both encoders get exercised.

| Trial | Direction | Wheel | count_A | count_B | Notes |
|---|---|---|---|---|---|
| 1 | CW  |  |  |  |  |
| 2 | CW  |  |  |  |  |
| 3 | CW  |  |  |  |  |
| 4 | CW  |  |  |  |  |
| 5 | CW  |  |  |  |  |
| 1 | CCW |  |  |  |  |
| 2 | CCW |  |  |  |  |
| 3 | CCW |  |  |  |  |
| 4 | CCW |  |  |  |  |
| 5 | CCW |  |  |  |  |

**How to compute the final value:** average the *absolute value* of every non-zero count above. If the
spread across trials is more than a few percent, something's inconsistent (slipping grip while rotating by
hand, miscounted turns) — investigate before trusting the average.

- **Average |counts| per revolution:**
- **COUNTS_PER_WHEEL_REV (final):**

---

## 2. WHEEL_BASE_M

**What this is:** the distance between the two wheels' contact points with the ground (track width).

**What to measure:** lay a ruler across the *bottom* of the chassis and measure from the **center of the
left tire's tread** to the **center of the right tire's tread** - equivalently, the distance between the
two motor shafts, since each wheel is centered on its own shaft. Take 2–3 independent measurements and
average them; a single reading is easy to get wrong by a few mm, which matters a lot here.

| Trial | Measured WHEEL_BASE_M |
|---|---|
| 1 |  |
| 2 |  |
| 3 |  |

- **WHEEL_BASE_M (final):**

---

## 3. WHEEL_RADIUS_M

**What this is:** effective rolling radius of the wheel, under load, on the surface the robot will
actually drive on (a soft surface compresses the tyre, so this can be smaller than the free/unloaded
radius — measure on the same surface the robot will run on).

**Method A — caliper, under the robot's own weight, on the actual running surface:**

- Measured:

**Method B — back-solve from a straight drive:** `zero`, `pwm BOTH <val>` to drive forward a distance `D`
that you measure with a tape measure, `stop`, `read` the ticks, then:

```
WHEEL_RADIUS_M = D / (2 * PI * ticks / COUNTS_PER_WHEEL_REV)
```
where `ticks` = average of |count_A| and |count_B|.

Watch the A/B tick counts as you go - if they differ by more than ~2%, the robot probably didn't drive straight. Re-run it rather than average it in.

| Trial | D (m) | count_A | count_B | A/B mismatch | Computed radius (m) |
|---|---|---|---|---|---|
| 1 |  |  |  |  |  |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |

- **Back-solved average:**
- **WHEEL_RADIUS_M (final):** (cross-check Method A and Method B agree within a few percent before
  finalizing)

---

## 4. MAX_TICKS_PER_INTERVAL and deadband floor

**What this is:** the fastest the wheels can turn (in ticks per 50 ms control interval — the PID's native
unit), and the lowest PWM that reliably produces motion at all (below this, PWM is wasted as heat/noise).

**How to measure:** wheels off the ground, `ramp A` then `ramp B` — this auto-sweeps PWM 0→250→0 in steps,
logging `pwm:<val> dtick:<val>` at each step over Serial/WebSerial. Save the raw output to a file so it can
be re-analyzed later (e.g. `ramp_results_<date>.txt`).

**A note on reading the log:** the down-ramp readings at a given PWM will read *higher* than the up-ramp
readings at the same PWM — that's wheel momentum/inertia carrying over from the higher speed just
commanded, not inconsistency. Use the **up-ramp** values (spinning up from a standstill) for both the
deadband floor and the min-speed figure, since that matches how the robot actually starts moving from
`cmd_vel = 0`. Use the **plateau near PWM 250** (last reading or two at that step) for the max-speed figure.

| Motor | PWM at which motion reliably starts (up-ramp) | Max dtick/50ms (up-ramp, near PWM 250) | dtick at the deadband-floor PWM |
|---|---|---|---|
| A |  |  |  |
| B |  |  |  |

- **MAX_TICKS_PER_INTERVAL (final):** (a few ticks below the lower of the two motors' max, for margin —
  both wheels must respect the same limit so saturation-scaling preserves turning radius)
- **Deadband floor (PWM):**
- **Max wheel speed (m/s)** = `MAX_TICKS_PER_INTERVAL / COUNTS_PER_WHEEL_REV * wheel circumference * (1000 / 50)`:
- **Min moving speed (m/s)** = ticks/interval at the deadband floor, converted the same way:

---

## Battery reading sanity check

- `read` battery_V at rest:
- Compare against the pack's expected nominal/full/low voltages and the code's `LOW_BATTERY_THRESHOLD_V` —
  note any discrepancy.

---

## Summary — final constants for `robot_base/robot_config.h`

| Constant | Value |
|---|---|
| `COUNTS_PER_WHEEL_REV` |  |
| `WHEEL_BASE_M` |  |
| `WHEEL_RADIUS_M` |  |
| `MAX_TICKS_PER_INTERVAL` |  |
| `SPEED_DEADBAND` |  |

`MOTOR_A_IS_LEFT` / `LEFT_DIR_SIGN` / `RIGHT_DIR_SIGN` aren't repeated here — see `calibration_log.md` for
the wiring-fixed values, unless the wiring changed this session.
