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

struct TunerConfig {
    float kp_start      = 3.0f;   // initial kP before tuning begins
    float kp_step_up    = 0.1f;   // kP increment when no oscillation
    float kp_step_down  = 0.05f;  // kP decrement per overshoot iteration
    float kd_step_up    = 0.5f;   // kD increment when oscillating
    float kd_step_down  = 0.1f;   // kD decrement when under-damped
    float ki_step_up    = 0.01f;  // kI increment when steady-state error remains
    float ki_step_down  = 0.005f; // kI decrement per overshoot iteration
    float ki_max        = 0.3f;   // kI ceiling
    float settle_error  = 1.0f;   // deg — acceptable steady-state error for kI phase
    float kd_settle_err = 1.5f;   // deg — threshold that triggers kD reduction
    int   sample_ms     = 10;     // sampling interval inside the observation window
    int   sample_count  = 200;    // number of samples per turn (total window = sample_ms * sample_count)
    int   turn_timeout  = 2000;   // ms passed to turnToHeading
    int   phase_delay   = 500;    // ms between iterations within a phase
    int   max_iters     = 50;     // per-phase iteration cap (prevents infinite loops)
};

// Runs the full three-phase tune (kP, then kD, then kI) against `target`
// degrees. Mutates kp/ki/kd in place and prints the final values.
// Returns false if any phase hit max_iters without converging.
bool pidTuneAngular(lemlib::Chassis& chassis, int target,
                    float& kp, float& ki, float& kd,
                    TunerConfig cfg = TunerConfig{});

// Individual tuning phases. Each commands one turn to `target`, samples the
// heading error, sets `oscillation` to the number of sign flips observed, and
// nudges the relevant constant. Exposed for custom tuning loops.
void tune_kp(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd, const TunerConfig& cfg);
void tune_ki(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd, const TunerConfig& cfg);
void tune_kd(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd, const TunerConfig& cfg);

} // namespace autotuner
