
#ifndef PID_H
#define PID_H

#include <Arduino.h>

// ---------------------------------------------------------------------
struct PID
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float prevError;
    float outMin, outMax; // clamp

    PID(float p = 0, float i = 0, float d = 0, float minOut = -2.0f, float maxOut = 2.0f)
        : kp(p), ki(i), kd(d),
          integrator(0.0f), prevError(0.0f),
          outMin(minOut), outMax(maxOut) {}

    float update(float dt, float setpoint, float measurement)
    {
        float error = setpoint - measurement;

        // P
        float P = kp * error;

        // I
        integrator += error * dt;
        float I = ki * integrator;

        // D (derivative on measurement)
        float derivative = (error - prevError) / dt;
        float D = kd * derivative;

        float out = P + I + D;

        // clamp output
        if (out > outMax)
            out = outMax;
        if (out < outMin)
            out = outMin;

        prevError = error;
        return out;
    }

    void reset()
    {
        integrator = 0.0f;
        prevError = 0.0f;
    }
};
#endif // PID_H
