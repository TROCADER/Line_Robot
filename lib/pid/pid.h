#pragma once

#include <stdbool.h>

// TODO: Optimize variable sizes
typedef struct PID_controller
{
    double Kp;
    double Ki;
    double Kd;
    double setpoint;
    double prev_err;
    double integ;
    double min;
    double max;
} PID_t;

/// @brief Calculates one step of a PID controller
/// @todo Optimize variable sizes
/// @param controller PID Controller
/// @param var Environment variable, data
/// @param dt Time step (ms)
/// @return Proportional term
double pid_calc(PID_t *controller, double var, double dt, bool cap);