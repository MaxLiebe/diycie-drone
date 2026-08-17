#ifndef MADGWICKIMU_H
#define MADGWICKIMU_H
#include <Arduino.h>

// =============================================================================
//  MadgwickIMU — 6-DOF Madgwick filter (gyro + accel).
//
//  Changes vs previous revision (drift-related, axis conventions UNCHANGED):
//    • Gyro-bias estimation (Madgwick's ζ term, from his 2010 report). The
//      normalised-gradient filter absorbs gyro bias up to 2β with no steady
//      error (β = 0.02 → 2.3 dps) but past that the estimate runs away
//      completely. The MPU6050 zero moves with temperature, so the ζ estimator
//      tracks slow bias drift (slew ≤ 2·ζ rad/s², gated to near-hover
//      conditions) and β can stay small.
//    • Accel gate: the accel correction is skipped when |a| is far from 1 g
//      (hard maneuvers, bumps, landing impacts). Otherwise identical maths.
//    • β is public and may be changed at run time (main uses a high β while
//      disarmed on the ground so the estimate converges instantly, and the
//      flight β once armed).
// =============================================================================
struct MadgwickIMU
{
    // Quaternion representing rotation from "world" (NED-like) to body FRD
    // q1 = w, q2 = x, q3 = y, q4 = z
    float q1, q2, q3, q4;

    // Filter gain (rad/s). Larger = more accel trust, more damping, less gyro drift.
    float beta;

    // Gyro-bias estimation gain (Madgwick ζ). 0 = off. Bias slews at ≤ 2ζ rad/s².
    //   0.001 → ≤ 0.11 dps per second, i.e. a 3 dps thermal shift is tracked
    //   in ~30 s. Small enough that a short transient can't pull it far.
    float zeta;

    // Estimated gyro bias (rad/s, body frame). Subtracted from the gyro input.
    float wbx, wby, wbz;

    // Bias-estimator gates
    float biasClampRad = 5.0f * (PI / 180.0f);     // never estimate more than ±5 dps
    float biasGateRateRad = 20.0f * (PI / 180.0f); // only estimate when |ω| < 20 dps
    // Accel gate: skip accel correction when |a| outside [accMinG, accMaxG]
    float accMinG = 0.6f;
    float accMaxG = 1.4f;

    // Proportional ("linear") region for the accel correction, in radians of
    // attitude error. 0 = classic Madgwick: the gradient is normalised, so the
    // correction is ALWAYS a full β slew regardless of how small the error is
    // (bang-bang). In hover the accelerometer only sees the tilt through drag
    // (lag ≈ 1/DRAG_C ≈ 4 s), and a bang-bang correction through that lag
    // produces a slow relay limit cycle: the drone wanders ±0.5 m/s / ±3° with
    // a period of several seconds and never settles. With linearRegionRad > 0
    // the correction rate is proportional to the error below that angle
    // (gain 2β/linearRegionRad rad/s per rad, ≈ Mahony Kp) and saturates at
    // the full β slew above it, so large errors still converge as before but
    // small ones are damped instead of chattering.
    float linearRegionRad = 0.15f;

    MadgwickIMU(float betaInit = 0.1f, float zetaInit = 0.0f)
        : q1(1.0f), q2(0.0f), q3(0.0f), q4(0.0f),
          beta(betaInit), zeta(zetaInit), wbx(0.0f), wby(0.0f), wbz(0.0f) {}

    void reset()
    {
        q1 = 1.0f;
        q2 = q3 = q4 = 0.0f;
        wbx = wby = wbz = 0.0f;
    }

    // gx,gy,gz: rad/s (FRD), ax,ay,az: accel in g (any units also work), dt: seconds
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt)
    {
        if (dt <= 0.0f)
            return;

        float q1 = this->q1;
        float q2 = this->q2;
        float q3 = this->q3;
        float q4 = this->q4;

        float recipNorm;
        float s1 = 0.0f, s2 = 0.0f, s3 = 0.0f, s4 = 0.0f;
        bool haveGradient = false;

        // --- 1) Accelerometer gradient (if usable) ----------------------
        const float aNorm = sqrtf(ax * ax + ay * ay + az * az);
        const bool accOk = (aNorm > 1e-6f) && (aNorm >= accMinG) && (aNorm <= accMaxG);
        if (accOk)
        {
            recipNorm = 1.0f / aNorm;
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            const float _2q1 = 2.0f * q1;
            const float _2q2 = 2.0f * q2;
            const float _2q3 = 2.0f * q3;
            const float _2q4 = 2.0f * q4;
            const float _4q1 = 4.0f * q1;
            const float _4q2 = 4.0f * q2;
            const float _4q3 = 4.0f * q3;
            const float _8q2 = 8.0f * q2;
            const float _8q3 = 8.0f * q3;
            const float q1q1 = q1 * q1;
            const float q2q2 = q2 * q2;
            const float q3q3 = q3 * q3;
            const float q4q4 = q4 * q4;

            // Gradient descent step (standard Madgwick IMU, gravity = +Z)
            // f(q) = [2(q2 q4 - q1 q3) - ax;
            //         2(q1 q2 + q3 q4) - ay;
            //         2(0.5 - q2^2 - q3^2) - az]
            s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
            s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
            s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;

            const float sNorm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
            if (sNorm > 1e-12f)
            {
                // |s| ≈ 2 × attitude error angle for small errors.
                // Classic: unit direction. Linear region: scale down when small.
                float scaleS = 1.0f / sNorm;
                if (linearRegionRad > 0.0f)
                {
                    const float sSat = 2.0f * linearRegionRad;
                    if (sNorm < sSat)
                        scaleS = 1.0f / sSat;
                }
                s1 *= scaleS;
                s2 *= scaleS;
                s3 *= scaleS;
                s4 *= scaleS;
                haveGradient = true;
            }
        }

        // --- 2) Gyro-bias estimation (Madgwick ζ), gated to near-hover ---
        if (haveGradient && zeta > 0.0f)
        {
            const float rate = sqrtf(gx * gx + gy * gy + gz * gz);
            if (rate < biasGateRateRad)
            {
                // ω_err = 2 q* ⊗ s  (direction of gyro error, |ω_err| = 2)
                const float wex = 2.0f * (q1 * s2 - q2 * s1 - q3 * s4 + q4 * s3);
                const float wey = 2.0f * (q1 * s3 + q2 * s4 - q3 * s1 - q4 * s2);
                const float wez = 2.0f * (q1 * s4 - q2 * s3 + q3 * s2 - q4 * s1);
                wbx = clampf(wbx + zeta * wex * dt, -biasClampRad, biasClampRad);
                wby = clampf(wby + zeta * wey * dt, -biasClampRad, biasClampRad);
                wbz = clampf(wbz + zeta * wez * dt, -biasClampRad, biasClampRad);
            }
        }

        // --- 3) Gyro-only quaternion derivative (bias-corrected gyro) -----
        gx -= wbx;
        gy -= wby;
        gz -= wbz;

        float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
        float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
        float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
        float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

        // --- 4) Apply accel feedback --------------------------------------
        if (haveGradient)
        {
            qDot1 -= beta * s1;
            qDot2 -= beta * s2;
            qDot3 -= beta * s3;
            qDot4 -= beta * s4;
        }

        // --- 5) Integrate & normalise -------------------------------------
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
        q4 += qDot4 * dt;

        recipNorm = 1.0f / sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        this->q1 = q1 * recipNorm;
        this->q2 = q2 * recipNorm;
        this->q3 = q3 * recipNorm;
        this->q4 = q4 * recipNorm;
    }

    // Convert quaternion to FRD Euler:  (UNCHANGED — verified conventions)
    //  roll  (φ) > 0: right wing down
    //  pitch (θ) > 0: nose up (nose down is negative)
    //  yaw   (ψ): rotation about +Z (down)
    void toEulerFRD(float &roll, float &pitch, float &yaw) const
    {
        const float w = q1, x = q2, y = q3, z = q4;

        // Rotation matrix R (body -> world) from quaternion
        const float R11 = 1.0f - 2.0f * (y * y + z * z);
        const float R21 = 2.0f * (x * y + z * w);
        const float R31 = 2.0f * (x * z - y * w);
        const float R32 = 2.0f * (y * z + x * w);
        const float R33 = 1.0f - 2.0f * (x * x + y * y);

        // World "down" expressed in the BODY frame = 3rd row of R  (like the accel)
        const float ax = R31;
        const float ay = R32;
        const float az = R33;

        roll = atan2f(ay, az);                         // right wing down > 0
        pitch = atan2f(-ax, sqrtf(ay * ay + az * az)); // nose up > 0
        yaw = atan2f(R21, R11);                        // about +Z (down)
    }

private:
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};
#endif // MADGWICKIMU_H
