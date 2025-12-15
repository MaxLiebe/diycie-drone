#ifndef MADGWICKIMU_H
#define MADGWICKIMU_H
#include <Arduino.h>

struct MadgwickIMU
{
    // Quaternion representing rotation from "world" (NED-like) to body FRD
    // q1 = w, q2 = x, q3 = y, q4 = z
    float q1, q2, q3, q4;
    // Filter gain (rad/s). Larger = more accel trust, more damping, less gyro drift.
    float beta;

    MadgwickIMU(float betaInit = 0.1f)
        : q1(1.0f), q2(0.0f), q3(0.0f), q4(0.0f), beta(betaInit) {}

    void reset()
    {
        q1 = 1.0f;
        q2 = q3 = q4 = 0.0f;
    }

    // gx,gy,gz: rad/s (FRD), ax,ay,az: accel in any units (here g), dt: seconds
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt)
    {
        float q1 = this->q1;
        float q2 = this->q2;
        float q3 = this->q3;
        float q4 = this->q4;

        float recipNorm;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q1, _2q2, _2q3, _2q4;
        float _4q1, _4q2, _4q3;
        float _8q2, _8q3;
        float q1q1, q2q2, q3q3, q4q4;

        if (dt <= 0.0f)
            return;

        // --- 1) Gyro-only quaternion derivative -------------------------
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

        // --- 2) Accelerometer correction (if non-zero) -----------------
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
            // Normalise accelerometer
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q4 = 2.0f * q4;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _4q3 = 4.0f * q3;
            _8q2 = 8.0f * q2;
            _8q3 = 8.0f * q3;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;
            q4q4 = q4 * q4;

            // Gradient descent step (standard Madgwick IMU, gravity = +Z)
            // f(q) = [2(q2 q4 - q1 q3) - ax;
            //         2(q1 q2 + q3 q4) - ay;
            //         2(0.5 - q2^2 - q3^2) - az]
            s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
            s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
            s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;

            // Normalise gradient
            recipNorm = 1.0f / sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            s4 *= recipNorm;

            // Apply feedback
            qDot1 -= beta * s1;
            qDot2 -= beta * s2;
            qDot3 -= beta * s3;
            qDot4 -= beta * s4;
        }

        // --- 3) Integrate to yield quaternion ---------------------------
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
        q4 += qDot4 * dt;

        // --- 4) Normalise quaternion -----------------------------------
        recipNorm = 1.0f / sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        this->q1 = q1 * recipNorm;
        this->q2 = q2 * recipNorm;
        this->q3 = q3 * recipNorm;
        this->q4 = q4 * recipNorm;
    }

    // Convert quaternion to FRD Euler:
    //  roll  (φ) > 0: right wing down
    //  pitch (θ) > 0: nose up (nose down is negative)
    //  yaw   (ψ): rotation about +Z (down)

    void toEulerFRD(float &roll, float &pitch, float &yaw) const
    {
        // q = [w, x, y, z]
        float w = q1;
        float x = q2;
        float y = q3;
        float z = q4;

        // Rotation matrix R (body -> world) from quaternion
        float R11 = 1.0f - 2.0f * (y * y + z * z);
        float R12 = 2.0f * (x * y - z * w);
        float R13 = 2.0f * (x * z + y * w);
        float R21 = 2.0f * (x * y + z * w);
        float R22 = 1.0f - 2.0f * (x * x + z * z);
        float R23 = 2.0f * (y * z - x * w);
        float R31 = 2.0f * (x * z - y * w);
        float R32 = 2.0f * (y * z + x * w);
        float R33 = 1.0f - 2.0f * (x * x + y * y);

        // World "down" = [0, 0, 1]^T.
        // For a body->world R, the down vector in BODY frame is the 3rd ROW of R^T,
        // i.e. the 3rd row of R:
        float ax = R31; // like accelX (gravity component)
        float ay = R32; // like accelY
        float az = R33; // like accelZ

        // Use EXACT SAME formulas as your accel-only angles:
        //
        // rollAcc  = atan2(ay, az);                       // right-wing-down positive
        // pitchAcc = atan2(-ax, sqrt(ay^2 + az^2));       // nose-down negative

        // roll > 0: right wing down
        roll = atan2f(ay, az);

        // pitch < 0: nose down (nose up positive)
        pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        // Yaw about +Z (down)
        yaw = atan2f(R21, R11);
    }
};
#endif // MADGWICKIMU_H