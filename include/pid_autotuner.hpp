// pid_autotuner.hpp
//
// Oscillation-counting auto-tuner for a LemLib chassis' angular
// (turn-to-heading) PID controller.
//
// The tuner repeatedly commands turns and counts how many times the heading
// error changes sign (each sign flip = one overshoot/oscillation). It walks
// kP up to the edge of instability, then adds kD to damp it, then adds kI to
// remove steady-state error. Final constants are printed over the terminal so
// you can copy them into your real controller settings.
//
// This is a portable refactor of the original project's globals-based tuner:
// the chassis and the three constants are passed in by reference instead of
// being read from globals, so the module can drop into any LemLib project.
#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep

namespace autotuner {

// Runs the full three-phase tune (kP, then kD, then kI) against `target`
// degrees. Mutates kp/ki/kd in place and prints the final values.
void pidTuneAngular(lemlib::Chassis& chassis, int target,
                    float& kp, float& ki, float& kd);

// Individual tuning phases. Each commands one turn to `target`, samples the
// heading error for ~2s, sets `oscillation` to the number of sign flips
// observed, and nudges the relevant constant. Exposed for custom tuning loops.
void tune_kp(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd);
void tune_ki(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd);
void tune_kd(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd);

} // namespace autotuner
