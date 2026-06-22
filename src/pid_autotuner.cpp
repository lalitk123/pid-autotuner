// pid_autotuner.cpp
#include "pid_autotuner.hpp"

#include <cstdio>

#include "pros/rtos.hpp" // pros::delay

namespace autotuner {

// The settle slew (4th PID constant) is loosened for small turns (<30 deg)
// where the controller would otherwise never report "settled".
static float settleSlew(int target) { return target < 30 ? float(4) : float(5); }

void tune_kp(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd) {
    chassis.setPose(0, 0, 0);
    oscillation = 0;
    float currError = lemlib::angleError(target, chassis.getPose().theta, false);
    float prevError;
    chassis.turnToHeading(target, 2000, {.PIDConstants = {kp, ki, kd, settleSlew(target)}});
    for (int i = 0; i < 200; i++) {
        prevError = currError;
        currError = lemlib::angleError(target, chassis.getPose().theta, false);
        if (currError * prevError < 0) {
            oscillation++;
            printf("\n oscillation! curr: %f, prev: %f\n", currError, prevError);
        }
        pros::delay(10);
    }
    if (oscillation == 0) {
        kp += 0.1;
    } else if (oscillation > 1) {
        kp -= 0.05 * oscillation;
    }
}

void tune_ki(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd) {
    chassis.setPose(0, 0, 0);
    oscillation = 0;
    float currError = lemlib::angleError(target, chassis.getPose().theta, false);
    float prevError;
    chassis.turnToHeading(target, 2000, {.PIDConstants = {kp, ki, kd, settleSlew(target)}});
    for (int i = 0; i < 200; i++) {
        prevError = currError;
        currError = lemlib::angleError(target, chassis.getPose().theta, false);
        if (currError * prevError < 0) {
            oscillation++;
            printf("\n oscillation! curr: %f, prev: %f\n", currError, prevError);
        }
        pros::delay(10);
    }

    if (currError > 1 && ki < 0.3) {
        ki += 0.01;
    } else if (oscillation > 0) {
        ki -= 0.005 * oscillation;
    }
}

void tune_kd(lemlib::Chassis& chassis, int target, int& oscillation,
             float& kp, float& ki, float& kd) {
    chassis.setPose(0, 0, 0);
    oscillation = 0;
    float currError = lemlib::angleError(target, chassis.getPose().theta, false);
    float prevError;
    chassis.turnToHeading(target, 2000, {.PIDConstants = {kp, ki, kd, settleSlew(target)}});
    for (int i = 0; i < 200; i++) {
        prevError = currError;
        currError = lemlib::angleError(target, chassis.getPose().theta, false);
        pros::delay(10);
        if (currError * prevError < 0) {
            oscillation++;
        }
    }
    if (oscillation > 0) {
        kd += 0.5;
    } else if (currError > 1.5) {
        kd -= 0.1;
    }
}

void pidTuneAngular(lemlib::Chassis& chassis, int target,
                    float& kp, float& ki, float& kd) {
    int oscillation = 0;
    kp = 3;
    ki = 0;
    kd = 0;
    do {
        tune_kp(chassis, target, oscillation, kp, ki, kd);
        printf("kp: %f\n", kp);
        pros::delay(500);
    } while (oscillation != 1);
    do {
        tune_kd(chassis, target, oscillation, kp, ki, kd);
        printf("kd: %f\n", kd);
        pros::delay(500);
    } while (oscillation > 0);
    do {
        tune_ki(chassis, target, oscillation, kp, ki, kd);
        printf("ki: %f\n", ki);
        pros::delay(500);
    } while (oscillation > 0 && lemlib::angleError(target, chassis.getPose().theta, false) > 1);
    printf("PID: %f, %f, %f \n", kp, ki, kd);
}

} // namespace autotuner
