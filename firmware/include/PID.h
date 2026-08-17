// =============================================================================
//  PID.h — rate-loop PID controller, esp-fc-style.
//
//    • Cascaded D-term filtering: optional Notch + PT1 + PT2 (esp-fc layout)
//    • Setpoint-derivative feed-forward with PT1 smoothing (Kf)
//    • I-term relax (esp-fc: factor on |setpoint - LPF(setpoint)|)
//    • Saturation-aware anti-windup using an external saturation flag that the
//      mixer feeds back when any motor clipped
//    • D-on-measurement (no setpoint kicks)
//
//  Changes vs previous revision:
//    • First update() after reset() primes prevMeasurement/prevSetpoint instead
//      of differentiating from 0 → no D/F kick on the first tick after arming.
//    • Anti-windup is now "conditional integration": when the mixer reports
//      saturation the integrator is only frozen if integrating would push the
//      output further INTO saturation. It may still unwind. Previously all
//      three axes froze whenever any motor clipped, so a hard yaw/thrust move
//      could stall the roll/pitch I-terms and let the attitude sag.
//
//  All filters use Filter.h (header-only). Same numerics as esp-fc.
// =============================================================================
#ifndef FC_PID_H
#define FC_PID_H

#include <Arduino.h>
#include "Filter.h"

struct PID
{
  // ── Tuning (set once) ─────────────────────────────────────────────────────
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float kf = 0.0f;          // setpoint feed-forward (rate of change)

  float outMin = -1.0f;
  float outMax =  1.0f;

  float iLimit = 0.5f;      // |I-term| clamp (output-units)

  // I-term relax: scale I error down when stick is moving fast, like esp-fc.
  // 0 = off. typical 15..30 (deg/s scale). Small = aggressive relax.
  float relaxRateDps = 20.0f;

  // ── State ────────────────────────────────────────────────────────────────
  float integrator = 0.0f;
  float prevMeasurement = 0.0f;
  float prevSetpoint = 0.0f;
  float lastOutput = 0.0f;
  bool  primed = false;
  bool  outputSaturated = false;     // set externally by mixer if it clipped

  // Filters (init via configure())
  fc::Biquad dNotch;     // optional resonance notch on D
  fc::PT1    dLpf1;      // primary D LPF
  fc::PT2    dLpf2;      // secondary D LPF (esp-fc cascade)
  fc::PT1    fLpf;       // F-term smoothing
  fc::PT1    relaxLpf;   // I-term relax setpoint LPF

  // ── Configuration ─────────────────────────────────────────────────────────
  // Call once in setup(), after sample-rate is known.
  // Pass 0 to any cutoff to disable that stage.
  void configure(float sampleRateHz,
                 float dLpfHz   = 90.0f,   // primary D LPF
                 float dLpf2Hz  = 150.0f,  // secondary D LPF
                 float fLpfHz   = 25.0f,   // F-term LPF (setpoint smoothing FF)
                 float relaxHz  = 8.0f,    // I-term relax setpoint cutoff
                 float dNotchHz = 0.0f,    // 0 = disabled
                 float dNotchCutoffHz = 0.0f)
  {
    dLpf1.init(sampleRateHz, dLpfHz);
    dLpf2.init(sampleRateHz, dLpf2Hz);
    fLpf .init(sampleRateHz, fLpfHz);
    relaxLpf.init(sampleRateHz, relaxHz);

    if (dNotchHz > 0.0f && dNotchCutoffHz > 0.0f)
    {
      const float q = fc::Biquad::notchQ(dNotchHz, dNotchCutoffHz);
      dNotch.init(fc::Biquad::NOTCH, sampleRateHz, dNotchHz, q);
    }
    else
    {
      dNotch.b0 = 1.0f; dNotch.b1 = dNotch.b2 = dNotch.a1 = dNotch.a2 = 0.0f;
    }
    reset();
  }

  void reset()
  {
    integrator = 0.0f;
    prevMeasurement = 0.0f;
    prevSetpoint = 0.0f;
    lastOutput = 0.0f;
    primed = false;
    outputSaturated = false;
    dNotch.reset();
    dLpf1.reset();
    dLpf2.reset();
    fLpf.reset();
    relaxLpf.reset();
  }

  // ── Update ────────────────────────────────────────────────────────────────
  // setpoint, measurement: same units (rad/s recommended for rate loop).
  // dt: seconds (used for I, D and F). Filter coefficients are fixed at
  //     configure() time — call at (approximately) that rate.
  inline float update(float dt, float setpoint, float measurement)
  {
    if (dt <= 0.0f) dt = 1e-3f;

    if (!primed)
    {
      // First sample after reset: no history → no derivative terms.
      prevMeasurement = measurement;
      prevSetpoint    = setpoint;
      primed = true;
    }

    // --- P -----------------------------------------------------------------
    const float error = setpoint - measurement;
    const float P = kp * error;

    // --- D on measurement (sign flip), notch + cascade LPF -----------------
    float D = 0.0f;
    if (kd > 0.0f)
    {
      float rawD = -(measurement - prevMeasurement) / dt;
      rawD = dNotch.update(rawD);
      rawD = dLpf1 .update(rawD);
      rawD = dLpf2 .update(rawD);
      D = kd * rawD;
    }

    // --- F (setpoint derivative, smoothed) ---------------------------------
    float F = 0.0f;
    if (kf > 0.0f)
    {
      const float rawF = (setpoint - prevSetpoint) / dt;
      F = kf * fLpf.update(rawF);
    }

    // --- I with relax + conditional anti-windup ----------------------------
    float iErr = error;
    if (ki > 0.0f && relaxRateDps > 0.0f)
    {
      // esp-fc: relaxBase = setpoint - LPF(setpoint)
      const float relaxBase = setpoint - relaxLpf.update(setpoint);
      // factor = max(0, 1 - |relaxBase[deg/s]| / relaxRateDps)
      const float baseDps = relaxBase * (180.0f / PI);
      float factor = 1.0f - fabsf(baseDps) / relaxRateDps;
      if (factor < 0.0f) factor = 0.0f;
      iErr *= factor;
    }

    if (ki > 0.0f)
    {
      // Integrate unless the mixer is saturated AND we'd push further into it.
      const bool allowI = !outputSaturated || (iErr * lastOutput < 0.0f);
      if (allowI)
      {
        integrator += iErr * dt;
        const float iLim = iLimit / ki;
        if (integrator >  iLim) integrator =  iLim;
        if (integrator < -iLim) integrator = -iLim;
      }
    }
    const float I = ki * integrator;

    // Pre-clamp output
    float out = P + I + D + F;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;

    prevMeasurement = measurement;
    prevSetpoint    = setpoint;
    lastOutput      = out;

    return out;
  }
};

#endif // FC_PID_H
