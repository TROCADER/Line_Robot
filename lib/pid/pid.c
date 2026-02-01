#include "pid.h"

double calc(PID_t *controller, double var, double dt, bool cap)
{
    // Calculate error as of now
    double err = controller->setpoint - var;

    // Kp - Proportional term
    double prop = controller->Kp * err;

    // Add the error to the controller's "memory"
    controller->integ += err * dt;

    // Ki - Integral term
    double integ = controller->Ki * controller->integ;

    // Kd - Derivative term
    double deriv = controller->Kd * (err - controller->prev_err) / dt;

    controller->prev_err = err;

    double k_final = prop + integ + deriv;

    // Capping incase too low or too high. Use at discretion
    if (cap)
    {
        if (k_final < controller->min)
        {
            return controller->min;
        }
        else if (k_final > controller->max)
        {
            return controller->max;
        }
    }

    return k_final;
}