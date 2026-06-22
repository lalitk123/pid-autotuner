# PID Autotuner (LemLib angular)

A small, drop-in **auto-tuner for the angular (turn-to-heading) PID** of a
[LemLib](https://lemlib.readthedocs.io/) chassis on a VEX V5 robot (PROS).

Instead of hand-tuning `kP`, `kI`, `kD`, this commands repeated turns and counts
**oscillations** (sign flips of the heading error) to converge on usable
constants automatically. It's a Ziegler–Nichols-style approach: push `kP` to the
edge of instability, damp with `kD`, then trim steady-state error with `kI`.

> Extracted and refactored from a VEX competition codebase. The original tuner
> read the chassis and constants from globals; here they're passed in by
> reference so the module drops into any project.

## How it works

`pidTuneAngular(chassis, target, kp, ki, kd)` runs three phases in order,
starting from `kp = 3, ki = 0, kd = 0`:

1. **kP** — raise `kP` by `0.1` while a trial produces zero oscillations; back
   off by `0.05 × oscillations` if it overshoots more than once. Loops until a
   trial yields *exactly one* oscillation (the stability edge).
2. **kD** — add `0.5` of `kD` whenever oscillation occurs; subtract `0.1` if the
   turn is sluggish (steady-state error > 1.5°). Loops until oscillations hit 0.
3. **kI** — add `0.01` of `kI` while there's leftover error and `ki < 0.3`; back
   off if it reintroduces oscillation. Loops until the turn settles within 1°.

Each trial: reset pose to `(0,0,0)`, command `turnToHeading(target, 2000ms)`,
then sample heading error every 10ms for 2s (200 samples). Final constants are
printed over the terminal:

```
PID: 5.300000, 0.020000, 1.500000
```

Copy those into your real `lemlib::ControllerSettings` (the tuner does **not**
persist them).

## Requirements

This is **not standalone** — it needs the surrounding robot runtime to compile
and only does anything useful on a real robot on the field:

- A PROS project with **LemLib** (`lemlib/api.hpp`, `lemlib::Chassis`,
  `lemlib::angleError`, `turnToHeading`).
- PROS RTOS (`pros::delay`).
- The robot physically able to spin in place safely.

## Usage

1. Copy `include/pid_autotuner.hpp` and `src/pid_autotuner.cpp` into your PROS
   project (matching its `include/` and `src/` layout).
2. Call it from autonomous (or a debug task), passing your chassis and the
   constants you want filled in:

   ```cpp
   #include "pid_autotuner.hpp"

   float kp, ki, kd;
   autotuner::pidTuneAngular(chassis, 90, kp, ki, kd); // tune a 90° turn
   ```

3. Watch the terminal, note the printed `PID:` line, and hardcode the values
   into your controller settings. Re-run across several angles (e.g. 45, 60, 90,
   120, 150, 180) and pick constants that behave well across the range.

### Lower-level API

The individual phases are exposed if you want a custom loop. Each sets
`oscillation` to the sign-flip count for the trial:

```cpp
int osc;
autotuner::tune_kp(chassis, 90, osc, kp, ki, kd);
autotuner::tune_kd(chassis, 90, osc, kp, ki, kd);
autotuner::tune_ki(chassis, 90, osc, kp, ki, kd);
```

## Caveats

- **Angular only.** There's no lateral/drive tuner here.
- The kP phase insists on landing on *exactly one* oscillation, so it can run a
  while or thrash near the boundary — watch it and stop if it won't converge.
- Tuning happens live on the robot; make sure the area is clear before running.
