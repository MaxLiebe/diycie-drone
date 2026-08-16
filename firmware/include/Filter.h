// =============================================================================
//  Filter.h — header-only DSP primitives ported from rtlopez/esp-fc
//  (lib/Espfc/src/Utils/Filter.h+cpp), trimmed to what a tiny FC needs:
//      PT1 / PT2 / PT3 low-pass         (Betaflight cascaded RC)
//      Biquad LPF (RBJ cookbook)        (steeper roll-off, ~Q=0.707 Butterworth)
//      Biquad Notch                     (kills resonance peaks)
//
//  All are single-precision, allocation-free, and small enough to live on the
//  stack of an ESP32-C3.  Same numeric formulas as esp-fc — proven in flight.
// =============================================================================
#ifndef FC_FILTER_H
#define FC_FILTER_H

#include <Arduino.h>
#include <math.h>

namespace fc
{

// ── PT1 (single-pole RC low-pass) ────────────────────────────────────────────
struct PT1
{
  float k = 0.0f;
  float v = 0.0f;

  void init(float sampleRateHz, float cutoffHz)
  {
    if (cutoffHz <= 0.0f || sampleRateHz <= 0.0f) { k = 0.0f; return; }
    const float rc = 1.0f / (2.0f * PI * cutoffHz);
    const float dt = 1.0f / sampleRateHz;
    k = dt / (dt + rc);
  }
  void reset(float seed = 0.0f) { v = seed; }
  inline float update(float n) { v += k * (n - v); return v; }
};

// ── PT2 (two cascaded PT1, Bessel-like, esp-fc correction factor) ───────────
struct PT2
{
  float k = 0.0f;
  float v0 = 0.0f, v1 = 0.0f;

  void init(float sampleRateHz, float cutoffHz)
  {
    constexpr float CORR = 1.553773974f; // 1/sqrt(2^(1/2)-1) — preserves -3dB @ cutoff
    if (cutoffHz <= 0.0f || sampleRateHz <= 0.0f) { k = 0.0f; return; }
    const float fc = cutoffHz * CORR;
    const float rc = 1.0f / (2.0f * PI * fc);
    const float dt = 1.0f / sampleRateHz;
    k = dt / (dt + rc);
  }
  void reset(float seed = 0.0f) { v0 = v1 = seed; }
  inline float update(float n)
  {
    v0 += k * (n  - v0);
    v1 += k * (v0 - v1);
    return v1;
  }
};

// ── PT3 (three cascaded PT1) ─────────────────────────────────────────────────
struct PT3
{
  float k = 0.0f;
  float v0 = 0.0f, v1 = 0.0f, v2 = 0.0f;

  void init(float sampleRateHz, float cutoffHz)
  {
    constexpr float CORR = 1.961459177f;
    if (cutoffHz <= 0.0f || sampleRateHz <= 0.0f) { k = 0.0f; return; }
    const float fc = cutoffHz * CORR;
    const float rc = 1.0f / (2.0f * PI * fc);
    const float dt = 1.0f / sampleRateHz;
    k = dt / (dt + rc);
  }
  void reset(float seed = 0.0f) { v0 = v1 = v2 = seed; }
  inline float update(float n)
  {
    v0 += k * (n  - v0);
    v1 += k * (v0 - v1);
    v2 += k * (v1 - v2);
    return v2;
  }
};

// ── Biquad (RBJ cookbook), DF2 transposed, LPF or Notch ─────────────────────
struct Biquad
{
  enum Type { LPF, NOTCH };
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
  float x1 = 0.0f, x2 = 0.0f;

  void init(Type type, float sampleRateHz, float freqHz, float q)
  {
    if (freqHz <= 0.0f || sampleRateHz <= 0.0f || q <= 0.0f)
    {
      b0 = 1.0f; b1 = b2 = a1 = a2 = 0.0f;
      return;
    }
    const float omega = (2.0f * PI * freqHz) / sampleRateHz;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2.0f * q);

    float B0, B1, B2, A0, A1, A2;
    switch (type)
    {
      case LPF:
        B0 = (1.0f - cs) * 0.5f;
        B1 =  1.0f - cs;
        B2 = (1.0f - cs) * 0.5f;
        A0 =  1.0f + alpha;
        A1 = -2.0f * cs;
        A2 =  1.0f - alpha;
        break;
      case NOTCH:
      default:
        B0 =  1.0f;
        B1 = -2.0f * cs;
        B2 =  1.0f;
        A0 =  1.0f + alpha;
        A1 = -2.0f * cs;
        A2 =  1.0f - alpha;
        break;
    }
    b0 = B0 / A0;
    b1 = B1 / A0;
    b2 = B2 / A0;
    a1 = A1 / A0;
    a2 = A2 / A0;
  }

  // Q for a notch given centre & cutoff (esp-fc approximation).
  static float notchQ(float centerHz, float cutoffHz)
  {
    if (cutoffHz <= 0.0f || centerHz <= cutoffHz) return 1.0f;
    return (cutoffHz * centerHz) / ((centerHz - cutoffHz) * (centerHz + cutoffHz));
  }

  void reset() { x1 = x2 = 0.0f; }

  inline float update(float n)
  {
    const float result = b0 * n + x1;
    x1 = b1 * n - a1 * result + x2;
    x2 = b2 * n - a2 * result;
    return result;
  }
};

} // namespace fc

#endif // FC_FILTER_H
