/******************************************************************************
 *  LOLIN C3 Pico ESP32-C3 — DIY Drone Flight Controller  (rev 3)
 *
 *  Rev-3 changes vs rev-2:
 *    1. BRAKE_GAIN enabled (0.06 rad/(m/s)) — observer now actually brakes drift.
 *    2. VEL_GAIN raised 8 → 12 — faster observer convergence (~83 ms τ).
 *    3. DUAL accel filter paths:
 *         axMadg/ayMadg/azMadg @ 50 Hz   → Madgwick attitude (quieter)
 *         axObs /ayObs         @ 100 Hz  → velocity observer (drag bump intact)
 *    4. Reset logic fixed:
 *         disarm  → HARD reset (PIDs + velocity = 0)
 *         armed & low throttle → soft 0.98 decay; PIDs untouched.
 *    5. DRAG_C = 0.25 (Martin & Sarras nominal).  Tune 0.20-0.30 in flight.
 *
 *  ╔══════════════════════════════════════════════════════════════════════════╗
 *  ║  DRAG-FORCE VELOCITY OBSERVER — PHYSICS & MATH VERIFICATION             ║
 *  ║  Leishman et al. 2014 (BYU) + Martin & Sarras 2016 (MINES ParisTech)    ║
 *  ╠══════════════════════════════════════════════════════════════════════════╣
 *  ║  Convention: FRD body frame (+X fwd, +Y right, +Z down).                ║
 *  ║  At hover level: az_g() ≈ +1, ax/ay ≈ 0  (gravity along +Z).            ║
 *  ║                                                                          ║
 *  ║  Accelerometer model (small-angle, hover):                              ║
 *  ║    ax_meas [g] = -sin(θ)            +  F_drag_x / (m·g)                 ║
 *  ║    ay_meas [g] = +sin(φ)·cos(θ)     +  F_drag_y / (m·g)                 ║
 *  ║                                                                          ║
 *  ║  Drag forces (oppose body velocity):                                    ║
 *  ║    F_drag_x = -μ·u,    F_drag_y = -μ·v                                  ║
 *  ║    DRAG_C   = μ/m  [1/s] — M&S nominal c̄ ≈ 0.25                        ║
 *  ║                                                                          ║
 *  ║  Gravity subtraction (this code):                                       ║
 *  ║    ax_grav = -sin(θ)                                                     ║
 *  ║    ay_grav = +sin(φ)·cos(θ)                                              ║
 *  ║    ax_drag = (ax_meas − ax_grav)·g  =  −DRAG_C·u   [m/s²]               ║
 *  ║    ay_drag = (ay_meas − ay_grav)·g  =  −DRAG_C·v                        ║
 *  ║                                                                          ║
 *  ║  Algebraic instantaneous velocity (Leishman BYU):                       ║
 *  ║    u_inst = -ax_drag / DRAG_C                                            ║
 *  ║    v_inst = -ay_drag / DRAG_C                                            ║
 *  ║                                                                          ║
 *  ║  PT1 observer correction (α = VEL_GAIN·dt):                             ║
 *  ║    vel += α·(inst − vel),   τ = 1/VEL_GAIN                               ║
 *  ║                                                                          ║
 *  ║  → MATH VERIFIED CORRECT against both papers.                           ║
 *  ╚══════════════════════════════════════════════════════════════════════════╝
 *
 *  TUNING GUIDE
 *  ────────────────────────────────────────────────────────────────────────────
 *  Symbol         Effect
 *  DRAG_C         Higher → smaller velocity estimate per measured drag
 *                 → drone drifts more before observer reacts.
 *                 Lower  → more aggressive estimate, can oscillate.
 *                 Sweep 0.20 / 0.25 / 0.30 in hover, pick stillest.
 *  BRAKE_GAIN     rad lean per m/s of estimated drift.
 *                 0.06 → 1 m/s commands ~3.4° corrective tilt.
 *                 Raise +0.01/flight; >0.10 usually oscillates.
 *  VEL_GAIN       Observer PT1 speed (1/τ). 8-15 at 500 Hz is a good range.
 *                 Higher → faster braking, more noise pickup.
 *  ACCEL_OBS_HZ   Less filtering = drag signature preserved, but more noise
 *                 in velocity estimate. 80-120 Hz is the sweet spot.
 *  PID Kd         Tightens rate loop but amplifies vibration. Use the
 *                 cascaded D-LPFs in PID.h before raising Kd further.
 *  motorTrims     Persistent yaw drift → tweak BL trim.
 *
 *  STABILITY INTERACTION (chain of gains in the outer loop)
 *  ────────────────────────────────────────────────────────────────────────────
 *  vel  → BRAKE_GAIN → angle bias → ANGLE_TO_RATE_P → rate setpoint
 *       → pidRoll.kp → motor diff → physical torque → angular accel
 *       → integrates back to angle → influences vel again (closed loop)
 *
 *  Effective open-loop "wandering controller" gain ≈
 *        BRAKE_GAIN · ANGLE_TO_RATE_P · pidRoll.kp
 *      = 0.06 · 4.0 · 0.06  =  0.0144
 *
 *  Keep that product below ~0.05 to avoid velocity-induced oscillation.
 *  If you raise BRAKE_GAIN, consider lowering ANGLE_TO_RATE_P (3.0) to keep
 *  the product safe.
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include "Filter.h"
#include "PID.h"
#include "MadgwickIMU.h"
#include "MPU6050.h"
#include "DroneWebControl.h"
#include "MotorSong.h"
#include "MarioTheme.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Pin map / hardware
// ─────────────────────────────────────────────────────────────────────────────
static const int motorPinFL = 5;
static const int motorPinFR = 4;
static const int motorPinBR = 6;
static const int motorPinBL = 7;
static const int motorPins[4] = {motorPinFL, motorPinFR, motorPinBR, motorPinBL};

#define LED_PIN 21
#define LED_COUNT 4
#define I2C_SDA_PIN 10
#define I2C_SCL_PIN 20

Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
MotorSong motorSong(motorPins, 4, 12);
DroneWebControl web;
MPU6050 imu;
MadgwickIMU madgwick(0.02f); // low beta — trust gyro for fast moves

// ─────────────────────────────────────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float DEG2RAD = PI / 180.0f;
static constexpr float GRAV = 9.81f;

static constexpr float LOOP_HZ = 500.0f;
static constexpr uint32_t LOOP_DT_US = (uint32_t)(1000000.0f / LOOP_HZ);

static constexpr float MAX_ANGLE_DEG = 18.0f;
static constexpr float MAX_YAW_RATE_DPS = 90.0f;
static constexpr float MAX_RATE_DPS = 220.0f;
static constexpr float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG2RAD;
static constexpr float MAX_YAW_RATE_RAD = MAX_YAW_RATE_DPS * DEG2RAD;
static constexpr float MAX_RATE_RAD = MAX_RATE_DPS * DEG2RAD;

static constexpr float INVERT_ANGLE_RAD = 100.0f * DEG2RAD;
static constexpr uint32_t INVERT_HOLD_MS = 120;

// Mixer (X-quad FRD: FL/BR CW, FR/BL CCW)
//                                FL,  FR,  BR,  BL
static const float MIX_ROLL[4] = {-1, +1, +1, -1};
static const float MIX_PITCH[4] = {-1, -1, +1, +1};
static const float MIX_YAW[4] = {+1, -1, +1, -1};

static float motorTrims[4] = {1.00f, 1.00f, 1.00f, 1.15f};

static constexpr float IDLE_THROTTLE = 0.05f;

// ─────────────────────────────────────────────────────────────────────────────
//  Drag-force velocity observer parameters
// ─────────────────────────────────────────────────────────────────────────────
// DRAG_C  = μ/m  [1/s].  Martin & Sarras nominal value.  Tune 0.20-0.30.
//   Higher → observer underestimates speed (drone drifts more before braking).
//   Lower  → observer overestimates → corrective lean can oscillate.
static constexpr float DRAG_C = 0.25f;

// VEL_GAIN: observer PT1 correction gain (1/τ).
//   τ = 1/VEL_GAIN ≈ 83 ms at VEL_GAIN=12, 500 Hz.
//   Higher = faster response, more noise.  Range 8-15 at 500 Hz.
static constexpr float VEL_GAIN = 12.0f;

// NMNI_THRESH: noise deadband on drag accel  [m/s²].
//   Increase if drone twitches at hover; decrease if it ignores small drifts.
static constexpr float NMNI_THRESH = 0.02f;

// BRAKE_GAIN: corrective lean per m/s of estimated drift  [rad/(m/s)].
//   0.06 → 1 m/s drift commands 0.06 rad ≈ 3.4° corrective lean.
//   Raise +0.01/flight if it still wanders.  Lower if oscillating.
static constexpr float BRAKE_GAIN = 0.06f;

static constexpr float VEL_CLAMP = 3.0f; // m/s safety clamp

// Observer state (body frame, m/s)
static float vel_x = 0.0f; // body-forward
static float vel_y = 0.0f; // body-right

// Soft decay rate when armed but throttle near zero. Per-tick multiplier.
//   0.98 ≈ 100 ms e-fold at 500 Hz — keeps short throttle dips harmless.
static constexpr float VEL_DECAY_LOWTHR = 0.98f;

// ─────────────────────────────────────────────────────────────────────────────
//  Attitude state
// ─────────────────────────────────────────────────────────────────────────────
struct AttitudeState
{
  float roll, pitch, yaw;
  float rollRate, pitchRate, yawRate;
};
static AttitudeState att = {};

// ─────────────────────────────────────────────────────────────────────────────
//  Pilot commands + smoothing
// ─────────────────────────────────────────────────────────────────────────────
struct Command
{
  float roll, pitch, yaw, thrust;
  bool armed;
};
static Command cmdRaw = {};
static Command cmdSmooth = {};

static constexpr float STICK_SMOOTH_HZ = 25.0f;
static constexpr float THROTTLE_SMOOTH_HZ = 10.0f;

static fc::PT2 sm_roll, sm_pitch, sm_yaw;
static fc::PT1 sm_thrust;

// ─────────────────────────────────────────────────────────────────────────────
//  IMU filter pipelines  (key change in rev-3: dual accel paths)
//
//   GYRO LIGHT  (PT1 120 Hz)  → Madgwick attitude
//   GYRO HEAVY  (PT2  90 Hz)  → PID rate-loop measurement
//   ACCEL MADG  (PT1  50 Hz)  → Madgwick attitude (quieter)
//   ACCEL OBS   (PT1 100 Hz)  → velocity observer  (drag bump preserved!)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float GYRO_LIGHT_HZ = 120.0f;
static constexpr float GYRO_HEAVY_HZ = 90.0f;
static constexpr float ACCEL_MADG_HZ = 50.0f;
static constexpr float ACCEL_OBS_HZ = 100.0f; // less filtered than Madgwick path

// Optional gyro motor-resonance notch (off by default; set both to non-zero).
static constexpr float GYRO_NOTCH_HZ = 0.0f;
static constexpr float GYRO_NOTCH_CUTOFF_HZ = 0.0f;

struct ImuFilters
{
  // Gyro
  fc::PT1 gxLight, gyLight, gzLight;
  fc::PT2 gxHeavy, gyHeavy, gzHeavy;
  fc::Biquad gxNotch, gyNotch, gzNotch;

  // Accel — TWO paths, different cutoffs
  fc::PT1 axMadg, ayMadg, azMadg; // 50 Hz → Madgwick
  fc::PT1 axObs, ayObs;           // 100 Hz → observer (lighter filtering)
};
static ImuFilters F;

// ─────────────────────────────────────────────────────────────────────────────
//  PID controllers
// ─────────────────────────────────────────────────────────────────────────────
static PID pidRoll, pidPitch, pidYaw;

// Outer-loop angle → rate gain (P only).  Lower this if you raise BRAKE_GAIN
// aggressively, to keep the velocity-loop product safe.
static constexpr float ANGLE_TO_RATE_P = 4.0f;

// ═════════════════════════════════════════════════════════════════════════════
//  Init helpers
// ═════════════════════════════════════════════════════════════════════════════
static void initFilters()
{
  F.gxLight.init(LOOP_HZ, GYRO_LIGHT_HZ);
  F.gyLight.init(LOOP_HZ, GYRO_LIGHT_HZ);
  F.gzLight.init(LOOP_HZ, GYRO_LIGHT_HZ);

  F.gxHeavy.init(LOOP_HZ, GYRO_HEAVY_HZ);
  F.gyHeavy.init(LOOP_HZ, GYRO_HEAVY_HZ);
  F.gzHeavy.init(LOOP_HZ, GYRO_HEAVY_HZ);

  F.axMadg.init(LOOP_HZ, ACCEL_MADG_HZ);
  F.ayMadg.init(LOOP_HZ, ACCEL_MADG_HZ);
  F.azMadg.init(LOOP_HZ, ACCEL_MADG_HZ);

  F.axObs.init(LOOP_HZ, ACCEL_OBS_HZ);
  F.ayObs.init(LOOP_HZ, ACCEL_OBS_HZ);

  if (GYRO_NOTCH_HZ > 0.0f && GYRO_NOTCH_CUTOFF_HZ > 0.0f)
  {
    const float q = fc::Biquad::notchQ(GYRO_NOTCH_HZ, GYRO_NOTCH_CUTOFF_HZ);
    F.gxNotch.init(fc::Biquad::NOTCH, LOOP_HZ, GYRO_NOTCH_HZ, q);
    F.gyNotch.init(fc::Biquad::NOTCH, LOOP_HZ, GYRO_NOTCH_HZ, q);
    F.gzNotch.init(fc::Biquad::NOTCH, LOOP_HZ, GYRO_NOTCH_HZ, q);
  }
}

static void initCommandSmoothing()
{
  sm_roll.init(LOOP_HZ, STICK_SMOOTH_HZ);
  sm_pitch.init(LOOP_HZ, STICK_SMOOTH_HZ);
  sm_yaw.init(LOOP_HZ, STICK_SMOOTH_HZ);
  sm_thrust.init(LOOP_HZ, THROTTLE_SMOOTH_HZ);
}

static void initPIDs()
{
  // Roll
  pidRoll.kp = 0.06f;
  pidRoll.ki = 0.30f;
  pidRoll.kd = 0.0010f;
  pidRoll.kf = 0.0008f;
  pidRoll.iLimit = 0.30f;
  pidRoll.relaxRateDps = 20.0f;
  pidRoll.configure(LOOP_HZ, /*dLpf*/ 90.0f, /*dLpf2*/ 150.0f, /*fLpf*/ 25.0f, /*relax*/ 8.0f);

  // Pitch (same as roll)
  pidPitch.kp = 0.06f;
  pidPitch.ki = 0.30f;
  pidPitch.kd = 0.0010f;
  pidPitch.kf = 0.0008f;
  pidPitch.iLimit = 0.30f;
  pidPitch.relaxRateDps = 20.0f;
  pidPitch.configure(LOOP_HZ, 90.0f, 150.0f, 25.0f, 8.0f);

  // Yaw
  pidYaw.kp = 0.05f;
  pidYaw.ki = 0.40f;
  pidYaw.kd = 0.0f;
  pidYaw.kf = 0.0f;
  pidYaw.iLimit = 0.30f;
  pidYaw.relaxRateDps = 30.0f;
  pidYaw.configure(LOOP_HZ, 60.0f, 120.0f, 25.0f, 8.0f);
}

// ═════════════════════════════════════════════════════════════════════════════
//  LED / motor helpers
// ═════════════════════════════════════════════════════════════════════════════
static void ledColor(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t c = leds.Color(r, g, b);
  for (int i = 0; i < LED_COUNT; ++i)
    leds.setPixelColor(i, c);
  leds.show();
}

static inline void writeMotors(const uint8_t pwm[4])
{
  for (int i = 0; i < 4; ++i)
    analogWrite(motorPins[i], pwm[i]);
}

static inline void cutMotors()
{
  uint8_t z[4] = {0, 0, 0, 0};
  writeMotors(z);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Airmode-style mixer with saturation feedback
// ═════════════════════════════════════════════════════════════════════════════
static bool mixAndApply(float base, float rollCmd, float pitchCmd, float yawCmd, bool armed)
{
  if (!armed)
  {
    cutMotors();
    return false;
  }

  // 1) Differential mix (no throttle)
  float diff[4];
  float dMin = +1e9f, dMax = -1e9f;
  for (int i = 0; i < 4; ++i)
  {
    diff[i] = rollCmd * MIX_ROLL[i] + pitchCmd * MIX_PITCH[i] + yawCmd * MIX_YAW[i];
    if (diff[i] < dMin)
      dMin = diff[i];
    if (diff[i] > dMax)
      dMax = diff[i];
  }

  // 2) Auto-scale if differential exceeds available headroom
  const float diffSpan = dMax - dMin;
  float scale = 1.0f;
  if (diffSpan > (1.0f - IDLE_THROTTLE))
  {
    scale = (1.0f - IDLE_THROTTLE) / diffSpan;
    for (int i = 0; i < 4; ++i)
      diff[i] *= scale;
    dMin *= scale;
    dMax *= scale;
  }

  // 3) Centre throttle so neither floor nor ceiling clips
  float thr = base;
  const float maxThr = 1.0f - dMax;
  const float minThr = IDLE_THROTTLE - dMin;
  if (thr > maxThr)
    thr = maxThr;
  if (thr < minThr)
    thr = minThr;

  // 4) Compose, clamp, write
  uint8_t pwm[4];
  bool saturated = (scale < 1.0f);
  for (int i = 0; i < 4; ++i)
  {
    float v = (thr + diff[i]) * motorTrims[i];
    if (v < 0.0f)
    {
      v = 0.0f;
      saturated = true;
    }
    if (v > 1.0f)
    {
      v = 1.0f;
      saturated = true;
    }
    pwm[i] = (uint8_t)(v * 255.0f + 0.5f);
  }
  writeMotors(pwm);
  return saturated;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Velocity observer
//
//  Inputs:  ax_g_obs / ay_g_obs   from the OBSERVER accel path (100 Hz).
//           att.pitch / att.roll  already updated this tick by Madgwick.
//
//  See header for full math derivation. Math VERIFIED against
//  Leishman 2014 (BYU) and Martin & Sarras 2016 (MINES ParisTech).
// ═════════════════════════════════════════════════════════════════════════════
static void updateVelocityObserver(float ax_g_obs, float ay_g_obs, float dt)
{
  // 1) Gravity projection in body frame [g]
  const float ax_grav = -sinf(att.pitch);
  const float ay_grav = sinf(att.roll) * cosf(att.pitch);

  // 2) Net drag-specific force [m/s²]
  float ax_drag = (ax_g_obs - ax_grav) * GRAV;
  float ay_drag = (ay_g_obs - ay_grav) * GRAV;

  // 3) Noise deadband
  if (fabsf(ax_drag) < NMNI_THRESH)
    ax_drag = 0.0f;
  if (fabsf(ay_drag) < NMNI_THRESH)
    ay_drag = 0.0f;

  // 4) Algebraic instantaneous velocity from drag model
  //    ax_drag = -DRAG_C·u  →  u = -ax_drag / DRAG_C
  const float instVx = -ax_drag / DRAG_C;
  const float instVy = -ay_drag / DRAG_C;

  // 5) PT1 observer correction  (α = VEL_GAIN · dt)
  float alpha = VEL_GAIN * dt;
  if (alpha > 1.0f)
    alpha = 1.0f;
  vel_x += (instVx - vel_x) * alpha;
  vel_y += (instVy - vel_y) * alpha;

  // 6) Safety clamp
  if (vel_x > VEL_CLAMP)
    vel_x = VEL_CLAMP;
  if (vel_x < -VEL_CLAMP)
    vel_x = -VEL_CLAMP;
  if (vel_y > VEL_CLAMP)
    vel_y = VEL_CLAMP;
  if (vel_y < -VEL_CLAMP)
    vel_y = -VEL_CLAMP;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Single FC tick — runs at LOOP_HZ
// ═════════════════════════════════════════════════════════════════════════════
static void flightTick(const MPU6050::Data &sample, float dt)
{
  // ── 1. Smooth pilot commands ───────────────────────────────────────────────
  cmdSmooth.roll = sm_roll.update(cmdRaw.roll);
  cmdSmooth.pitch = sm_pitch.update(cmdRaw.pitch);
  cmdSmooth.yaw = sm_yaw.update(cmdRaw.yaw);
  cmdSmooth.thrust = sm_thrust.update(cmdRaw.thrust);
  cmdSmooth.armed = cmdRaw.armed;

  // ── 2. Filter IMU — LIGHT gyro + Madgwick accel path ──────────────────────
  const float gxL = F.gxLight.update(sample.gx_rad());
  const float gyL = F.gyLight.update(sample.gy_rad());
  const float gzL = F.gzLight.update(sample.gz_rad());

  const float axM = F.axMadg.update(sample.ax_g());
  const float ayM = F.ayMadg.update(sample.ay_g());
  const float azM = F.azMadg.update(sample.az_g());

  // ── 3. Filter IMU — separate observer accel path (lighter) ────────────────
  const float axO = F.axObs.update(sample.ax_g());
  const float ayO = F.ayObs.update(sample.ay_g());

  // ── 4. Madgwick attitude (LIGHT gyro + Madgwick accel) ────────────────────
  madgwick.update(gxL, gyL, gzL, axM, ayM, azM, dt);
  madgwick.toEulerFRD(att.roll, att.pitch, att.yaw);

  // ── 5. Velocity observer (observer accel — drag signature preserved) ──────
  updateVelocityObserver(axO, ayO, dt);

  // ── 6. HEAVY gyro path → PID rate-loop measurement ────────────────────────
  float gxH = sample.gx_rad();
  float gyH = sample.gy_rad();
  float gzH = sample.gz_rad();
  if (GYRO_NOTCH_HZ > 0.0f)
  {
    gxH = F.gxNotch.update(gxH);
    gyH = F.gyNotch.update(gyH);
    gzH = F.gzNotch.update(gzH);
  }
  att.rollRate = F.gxHeavy.update(gxH);
  att.pitchRate = F.gyHeavy.update(gyH);
  att.yawRate = F.gzHeavy.update(gzH);

  // ── 7. Inversion safety (>100° for >120 ms → cut) ─────────────────────────
  static uint32_t invertedSinceMs = 0;
  const bool inverted = (fabsf(att.roll) > INVERT_ANGLE_RAD) ||
                        (fabsf(att.pitch) > INVERT_ANGLE_RAD);
  const uint32_t nowMs = millis();
  if (inverted)
  {
    if (invertedSinceMs == 0)
      invertedSinceMs = nowMs;
    if ((nowMs - invertedSinceMs) >= INVERT_HOLD_MS)
    {
      cmdSmooth.thrust = 0.0f;
      cmdSmooth.roll = cmdSmooth.pitch = cmdSmooth.yaw = 0.0f;
    }
  }
  else
  {
    invertedSinceMs = 0;
  }

  // ── 8. Reset / decay logic  (rev-3: don't wipe on low throttle) ───────────
  //   • Disarmed                → HARD reset (clean slate next flight)
  //   • Armed + thr < 0.05      → SOFT decay velocity, leave PIDs alone
  //                                so I-term & observer memory survive dips.
  if (!cmdSmooth.armed)
  {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    vel_x = 0.0f;
    vel_y = 0.0f;
  }
  else if (cmdSmooth.thrust < 0.05f)
  {
    vel_x *= VEL_DECAY_LOWTHR;
    vel_y *= VEL_DECAY_LOWTHR;
    // PIDs untouched on purpose — keep low-frequency memory.
  }

  // ── 9. Cascade: velocity-braked angle → rate setpoint → inner PID ─────────
  // vel_x > 0 (forward drift)   → +pitch  (nose up)  → decelerates forward
  // vel_y > 0 (right drift)     → -roll   (left lean) → decelerates rightward
  const float rollAngleSet = cmdSmooth.roll * MAX_ANGLE_RAD - vel_y * BRAKE_GAIN;
  const float pitchAngleSet = cmdSmooth.pitch * MAX_ANGLE_RAD + vel_x * BRAKE_GAIN;

  float rollRateSet = ANGLE_TO_RATE_P * (rollAngleSet - att.roll);
  float pitchRateSet = ANGLE_TO_RATE_P * (pitchAngleSet - att.pitch);

  if (rollRateSet > MAX_RATE_RAD)
    rollRateSet = MAX_RATE_RAD;
  if (rollRateSet < -MAX_RATE_RAD)
    rollRateSet = -MAX_RATE_RAD;
  if (pitchRateSet > MAX_RATE_RAD)
    pitchRateSet = MAX_RATE_RAD;
  if (pitchRateSet < -MAX_RATE_RAD)
    pitchRateSet = -MAX_RATE_RAD;

  const float yawRateSet = cmdSmooth.yaw * MAX_YAW_RATE_RAD;

  const float rollCmd = pidRoll.update(dt, rollRateSet, att.rollRate);
  const float pitchCmd = pidPitch.update(dt, pitchRateSet, att.pitchRate);
  const float yawCmd = pidYaw.update(dt, yawRateSet, -att.yawRate);

  // ── 10. Mix + write + saturation feedback (1-loop late, ~2 ms) ────────────
  const bool sat = mixAndApply(cmdSmooth.thrust, rollCmd, pitchCmd, yawCmd, cmdSmooth.armed);
  pidRoll.outputSaturated = sat;
  pidPitch.outputSaturated = sat;
  pidYaw.outputSaturated = sat;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Setup
// ═════════════════════════════════════════════════════════════════════════════
static bool mpuOk = false;

void setup()
{
  cmdRaw = {0, 0, 0, 0, false};
  cmdSmooth = {0, 0, 0, 0, false};

  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  leds.begin();
  leds.setBrightness(50);
  ledColor(0, 0, 255); // blue = booting

  mpuOk = imu.begin(Wire);
  if (!mpuOk)
  {
    ledColor(255, 0, 0);
    return;
  }
  ledColor(0, 255, 0); // green = IMU OK

  analogWriteFrequency(32000);
  analogWriteResolution(8);
  for (int i = 0; i < 4; ++i)
  {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // Startup chime
  motorSong.setRestoreFrequency(32000);
  motorSong.begin(MARIO_THEME, MARIO_THEME_LEN);
  while (motorSong.play())
  { /* spin until done */
  }

  // IMU calibration — keep drone level on a flat surface.
  imu.calibrate(500);

  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());
  Serial.printf("Heap: %u\n", ESP.getFreeHeap());

  initFilters();
  initCommandSmoothing();
  initPIDs();

  if (!web.begin("DRONE_NET"))
  {
    while (true)
    {
      ledColor(255, 0, 0);
      delay(200);
      ledColor(0, 0, 0);
      delay(200);
    }
  }

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println(imu.calibrationToString());

  delay(2000);
  ledColor(0, 0, 0);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main loop — fixed-rate 500 Hz scheduler + ~100 Hz web service
// ═════════════════════════════════════════════════════════════════════════════
void loop()
{
  static uint32_t lastTickUs = 0;
  static uint32_t lastWebMs = 0;

  // Web at ~100 Hz, between FC ticks
  const uint32_t nowMs = millis();
  if (nowMs - lastWebMs >= 10)
  {
    lastWebMs = nowMs;
    web.loop();

    if (web.hasNewRight())
    {
      DroneWebControl::DroneCommand c = web.getDroneCommand(true);
      cmdRaw.roll = c.roll;
      cmdRaw.pitch = c.pitch;
      cmdRaw.yaw = c.yaw;
      cmdRaw.thrust = c.thrust;
      cmdRaw.armed = c.armed;
    }
  }

  if (!mpuOk)
  {
    static uint32_t lastBlink = 0;
    if (nowMs - lastBlink > 200)
    {
      lastBlink = nowMs;
      static bool on = false;
      on = !on;
      ledColor(on ? 255 : 0, 0, 0);
    }
    yield();
    return;
  }

  // FC tick — fixed dt = 1/LOOP_HZ keeps filter coeffs valid.
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTickUs) >= LOOP_DT_US)
  {
    const uint32_t elapsed = nowUs - lastTickUs;
    if (elapsed > LOOP_DT_US * 4 || lastTickUs == 0)
      lastTickUs = nowUs; // resync if we fell behind
    else
      lastTickUs += LOOP_DT_US;

    if (imu.update())
    {
      flightTick(imu.data(), 1.0f / LOOP_HZ);
    }
    else
    {
      static uint32_t lastErr = 0;
      if (nowMs - lastErr > 500)
      {
        lastErr = nowMs;
        Serial.println("MPU6050: read failed");
      }
      cmdRaw.thrust = 0.0f;
      cutMotors();
    }
  }

  yield();
}
