// pid_autotuner.cpp
#include "pid_autotuner.hpp"

#include <cstdio>

#include "pros/rtos.hpp" // pros::delay

namespace autotuner {

// Loosens settle slew for small turns where the controller would otherwise
// never report "settled".
static float settleSlew(int target) { return target < 30 ? 4.0f : 5.0f; }

struct SampleResult {
    int   oscillations;
    float finalError;
};

// Commands one turn and counts heading-error sign flips over the observation
// window defined by cfg.sample_count and cfg.sample_ms.
static SampleResult countOscillations(lemlib::Chassis& chassis, int target,
                                      float kp, float ki, float kd,
                                      const TunerConfig& cfg) {
    chassis.setPose(0, 0, 0);
    float currError = lemlib::angleError(target, chassis.getPose().theta, false);
    float prevError;
    int   oscillations = 0;

    chassis.turnToHeading(target, cfg.turn_timeout,
                          {.PIDConstants = {kp, ki, kd, settleSlew(target)}});

    for (int i = 0; i < cfg.sample_count; i++) {
        prevError = currError;
        currError = lemlib::angleError(target, chassis.getPose().theta, false);
        if (currError * prevError < 0) {
            oscillations++;
            printf("\n oscillation! curr: %f, prev: %f\n", currError, prevError);
        }
        pros::delay(cfg.sample_ms);
    }

    return {oscillations, currError};
}

void tune_kp(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd, const TunerConfig& cfg) {
    auto [osc, err] = countOscillations(chassis, target, kp, ki, kd, cfg);
    oscillation = osc;
    if (osc == 0) {
        kp += cfg.kp_step_up;
    } else if (osc > 1) {
        kp -= cfg.kp_step_down; // fixed step — avoids large kP swings on many oscillations
    }
}

void tune_kd(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd, const TunerConfig& cfg) {
    auto [osc, err] = countOscillations(chassis, target, kp, ki, kd, cfg);
    oscillation = osc;
    if (osc > 0) {
        kd += cfg.kd_step_up;
    } else if (err > cfg.kd_settle_err) {
        kd -= cfg.kd_step_down;
    }
}

void tune_ki(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd, const TunerConfig& cfg) {
    auto [osc, err] = countOscillations(chassis, target, kp, ki, kd, cfg);
    oscillation = osc;
    if (err > cfg.settle_error && ki < cfg.ki_max) {
        ki += cfg.ki_step_up;
    } else if (osc > 0) {
        ki -= cfg.ki_step_down * osc;
    }
}

bool pidTuneAngular(lemlib::Chassis& chassis, int target,
                    float& kp, float& ki, float& kd, TunerConfig cfg) {
    int oscillation = 0;
    kp = cfg.kp_start;
    ki = 0;
    kd = 0;

    // Phase 1: raise kP until we see exactly one oscillation (or just tip over).
    // Use < 2 instead of != 1 so a jump from 0→2 doesn't loop forever.
    int iters = 0;
    do {
        tune_kp(chassis, target, oscillation, kp, ki, kd, cfg);
        printf("kp: %f\n", kp);
        pros::delay(cfg.phase_delay);
    } while (oscillation < 2 && ++iters < cfg.max_iters);
    if (iters >= cfg.max_iters) {
        printf("WARNING: kP phase did not converge after %d iterations\n", cfg.max_iters);
        return false;
    }

    // Phase 2: add kD to damp the oscillations down to zero.
    iters = 0;
    do {
        tune_kd(chassis, target, oscillation, kp, ki, kd, cfg);
        printf("kd: %f\n", kd);
        pros::delay(cfg.phase_delay);
    } while (oscillation > 0 && ++iters < cfg.max_iters);
    if (iters >= cfg.max_iters) {
        printf("WARNING: kD phase did not converge after %d iterations\n", cfg.max_iters);
        return false;
    }

    // Phase 3: add kI to eliminate steady-state error.
    iters = 0;
    do {
        tune_ki(chassis, target, oscillation, kp, ki, kd, cfg);
        printf("ki: %f\n", ki);
        pros::delay(cfg.phase_delay);
    } while (oscillation > 0 &&
             lemlib::angleError(target, chassis.getPose().theta, false) > cfg.settle_error &&
             ++iters < cfg.max_iters);
    if (iters >= cfg.max_iters) {
        printf("WARNING: kI phase did not converge after %d iterations\n", cfg.max_iters);
        return false;
    }

    printf("PID: %f, %f, %f\n", kp, ki, kd);
    return true;
}

} // namespace autotuner
