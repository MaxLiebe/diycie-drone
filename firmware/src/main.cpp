/******************************************************************************
 *  LOLIN C3 Pico ESP32-C3 — DIY Drone Flight Controller  (rev 4)
 *
 *  Rev-4 — DRIFT FIXES (axis conventions untouched, all verified as-is)
 *  ────────────────────────────────────────────────────────────────────────────
 *   1. VELOCITY OBSERVER  (the big one)
 *        Rev-3 subtracted an attitude-derived "gravity" term from the body-x/y
 *        accelerometer before dividing by DRAG_C. But in flight the body-frame
 *        accelerometer does NOT contain a gravity/tilt term at all: the only
 *        forces on the airframe are thrust (along body -Z, zero x/y component)
 *        and drag. What the accel reads on x/y IS the drag — no subtraction
 *        needed. That is the whole point of the Leishman / Martin-Sarras
 *        drag-force method. Subtracting -sin(θ) turned the estimate into
 *        −(horizontal acceleration)/DRAG_C, which includes the tilt-induced
 *        acceleration of the brake itself:
 *          - with rev-3's sign the loop was stable but ANTI-braking: after any
 *            gust/stick input the drone kept drifting ~3× longer than with no
 *            observer at all (simulated: 1 m/s gust never settles in 30 s;
 *            no observer 12 s; corrected observer 3.2 s).
 *          - simply flipping the sign is UNSTABLE (positive feedback through
 *            the tilt-induced acceleration, loop gain g·BRAKE/DRAG_C = 2.35)
 *            and runs to VEL_CLAMP → 10° lean → fly-away.
 *        Corrected: u = ax_meas·g/DRAG_C, v = ay_meas·g/DRAG_C using the raw
 *        body-frame accel. Bonus: no longer depends on attitude accuracy.
 *
 *   2. LEVEL TRIM (web UI, saved in NVS)
 *        If the sensor's "level" and the propeller plane's "level" differ by
 *        δ, the drone drifts at u = g·δ/DRAG_C (1° → 0.7 m/s) and NEITHER the
 *        accelerometer NOR the drag observer can see it: in steady drift the
 *        horizontal specific force in the body frame is exactly zero. The
 *        only cure is a setpoint offset — the trim buttons.
 *
 *   3. LEVEL CALIBRATION IS PERSISTED (NVS)
 *        Rev-3 recomputed the level rotation on every boot from whatever the
 *        drone was resting on. Booting on a 1–2° tilted floor gave a different
 *        "level" every flight → different drift every flight, and trims would
 *        be meaningless. Now: calibrate ONCE on a known level surface
 *        (auto on first boot, or "Recalibrate level" in the UI), and reuse.
 *        The gyro zero (temperature dependent) is still re-measured at every
 *        boot (calibrateGyro, ~1 s, any orientation, must be still).
 *
 *   4. GYRO ZERO DRIFT (temperature) — insurance
 *        The normalised-gradient Madgwick can absorb up to 2β of gyro bias
 *        with ZERO steady error (β=0.02 → 2.3 dps), but past that the estimate
 *        runs away completely (3 dps → 25° error in 30 s, simulated). The
 *        MPU6050 zero moves with temperature (spec ±20 dps over -40…85 °C), so
 *        a warm ESC/battery bay can eat into that margin during a flight.
 *        Madgwick now runs a slow, gated gyro-bias estimator (ζ, from
 *        Madgwick's own report) so this can't happen, β stays 0.02 in flight,
 *        and a high β is used while disarmed so the estimate is converged
 *        before take-off. Estimated bias is printed on Serial (disarmed).
 *
 *   5. TIMING
 *        The 500 Hz tick now integrates with the MEASURED dt (clamped 0.5–4×
 *        nominal) instead of a fixed 2 ms, and no longer runs "catch-up" ticks
 *        back-to-back. Any WiFi/HTTP stall (page load, slow client) used to
 *        integrate 2 ms of gyro for 6–20 ms of real time = attitude error →
 *        drift. Late ticks are counted and printed (Serial) so you can see
 *        whether the web link is starving the loop.
 *
 *   6. MIXER / ANTI-WINDUP
 *        motorTrims (BL ×1.15) were applied AFTER the head-room computation,
 *        so at higher throttle BL clipped at 1.0, the mixer flagged
 *        "saturated" and ALL THREE integrators froze → attitude sag → drift.
 *        Trims are now inside the head-room calculation and the PID uses
 *        conditional integration (may always unwind).
 *
 *   7. Small: accel bias so rest reads exactly 1.000 g, gyro bias rounded,
 *        first PID tick after arming no longer produces a D/F kick.
 *
 *   8. ACCEL "SILICON" CALIBRATION IN THE FIRMWARE (6 poses, web UI)
 *        The old calibration.cpp sketch produced only the three scale
 *        constants and threw the sensor-frame offsets away, leaving the
 *        per-boot Rodrigues step to interpret any X/Y offset as tilt. Now the
 *        6-pose routine lives in the FC: it solves offset AND scale per axis
 *        (ellipsoid → unit sphere), applies them BEFORE the mounting rotation,
 *        checks all six poses against |a| = 1 g, stores everything in NVS and
 *        finishes with the level calibration on the last (flat) pose.
 *        Note: this is not what makes a drone drift on its own — a pure
 *        offset was already absorbed by the level cal — but a wrong SCALE
 *        (yours: Z 0.957) does distort the tilt seen at |a| ≠ 1 g, and the
 *        cleaner pipeline makes the trims/level repeatable.
 *
 *   9. ATTITUDE-ESTIMATOR LIMIT CYCLE  (found in simulation, see REPORT.md)
 *        Classic Madgwick normalises the gradient, so the accel correction is
 *        always a full 2β slew no matter how small the error — bang-bang. In
 *        hover the accelerometer only sees tilt through drag (lag ≈ 1/DRAG_C
 *        ≈ 4 s); a bang-bang correction through that lag is a relay
 *        oscillator: the simulated rev-3 drone wandered ±0.5–0.8 m/s, ±3°,
 *        period ~8 s, growing after a gust and never settling. Madgwick now
 *        has a proportional region (linearRegionRad = 0.15 rad ≈ 8.6°: gain
 *        2β/0.15 ≈ 0.27 rad/s per rad, like Mahony Kp) and the full β slew
 *        only beyond it. Simulated: gust of 1 m/s decays to < 0.05 m/s in
 *        ~8 s and stays there; hover creep < 0.05 m/s. Also VEL_CLAMP 3 → 2
 *        m/s (max brake lean 6.9° instead of 10°): an external push reads as
 *        drag with the wrong sign for as long as it lasts, the clamp bounds
 *        that.
 *
 *  BENCH TESTS (do these before the first flight with rev-4)
 *  ────────────────────────────────────────────────────────────────────────────
 *   a) Serial @115200 prints "acc(g) ax ay az | att | vel" at 5 Hz while
 *      disarmed. Hold the drone level, slide it FORWARD along the table and
 *      stop: ax goes NEGATIVE while speeding up, POSITIVE while slowing down
 *      (slowing down = what drag does in flight). "vel x" (also in the UI)
 *      follows ax with the SAME sign. Push RIGHT: same for ay / vel y. If ax
 *      is positive while speeding up, stop — tell me before flying with
 *      BRAKE_GAIN > 0. (On a tilted table "vel" shows a false constant value
 *      while disarmed — expected, it is only meaningful in the air and is
 *      zeroed when you arm.)
 *   b) UI telemetry: tilt nose up → pitch positive; right wing down → roll
 *      positive (as before). Put it flat → both return to ~0 within a second.
 *   c) First flight: hover, hands off sticks. If it drifts steadily one way,
 *      press the trim button of the direction you want it to GO (opposite to
 *      the drift), 0.2° per press. Trims are stored on the drone.
 *
 *  ╔══════════════════════════════════════════════════════════════════════════╗
 *  ║  DRAG-FORCE VELOCITY OBSERVER — corrected derivation                    ║
 *  ║  Leishman et al. 2014 (BYU) + Martin & Sarras 2016                       ║
 *  ╠══════════════════════════════════════════════════════════════════════════╣
 *  ║  FRD body frame (+X fwd, +Y right, +Z down). This code's accel reads    ║
 *  ║  m = -f/g where f = specific force = (thrust + drag)/mass  [body frame] ║
 *  ║  (at rest on the ground f = -g_body, so m = +g_body/g: az=+1, ax=-sinθ). ║
 *  ║  In flight thrust is along body -Z ⇒ f_x = drag_x/m = -DRAG_C·u          ║
 *  ║    ax_meas [g] = -f_x/g = +DRAG_C·u/g   (no tilt term!)                  ║
 *  ║    ay_meas [g] = -f_y/g = +DRAG_C·v/g                                    ║
 *  ║  ⇒ u = ax_meas·g/DRAG_C ,  v = ay_meas·g/DRAG_C                          ║
 *  ║  PT1 observer:  vel += VEL_GAIN·dt·(inst - vel)                          ║
 *  ║  Brake: forward drift (u>0) → +pitch (nose up); right (v>0) → -roll.    ║
 *  ║  Closed loop: effective drag DRAG_C + g·BRAKE_GAIN ≈ 0.25+0.59 = 0.84/s ║
 *  ╚══════════════════════════════════════════════════════════════════════════╝
 *
 *  TUNING GUIDE
 *  ────────────────────────────────────────────────────────────────────────────
 *  TRIM (UI)      First thing to fix a steady one-direction drift.
 *  DRAG_C         Higher → smaller velocity estimate per measured drag.
 *                 Sweep 0.20 / 0.25 / 0.30 in hover, pick stillest.
 *  BRAKE_GAIN     rad lean per m/s of estimated drift. 0.06 → 3.4°/(m/s).
 *                 Raise +0.01/flight; >0.10 usually oscillates.
 *  VEL_GAIN       Observer PT1 speed (1/τ). 8-15 at 500 Hz.
 *  MADGWICK_BETA_FLIGHT  0.02–0.05. Higher = trusts accel more (levels
 *                 faster after a stall/bump) but feels mushier in fast moves.
 *  MADGWICK_ZETA  0 = off. 0.001 tracks ~0.1 dps/s of gyro zero drift.
 *
 *  Effective open-loop "wandering controller" gain ≈
 *        BRAKE_GAIN · ANGLE_TO_RATE_P · pidRoll.kp = 0.06 · 4.0 · 0.06 = 0.0144
 *  Keep that product below ~0.05.
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
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
Preferences prefs;

// ─────────────────────────────────────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float DEG2RAD = PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI;
static constexpr float GRAV = 9.81f;

static constexpr float LOOP_HZ = 500.0f;
static constexpr float LOOP_DT_S = 1.0f / LOOP_HZ;
static constexpr uint32_t LOOP_DT_US = (uint32_t)(1000000.0f / LOOP_HZ);
static constexpr float LOOP_DT_MIN_S = 0.5f * LOOP_DT_S; // clamps for measured dt
static constexpr float LOOP_DT_MAX_S = 4.0f * LOOP_DT_S;

static constexpr float MAX_ANGLE_DEG = 18.0f;
static constexpr float MAX_YAW_RATE_DPS = 90.0f;
static constexpr float MAX_RATE_DPS = 220.0f;
static constexpr float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG2RAD;
static constexpr float MAX_YAW_RATE_RAD = MAX_YAW_RATE_DPS * DEG2RAD;
static constexpr float MAX_RATE_RAD = MAX_RATE_DPS * DEG2RAD;

static constexpr float INVERT_ANGLE_RAD = 100.0f * DEG2RAD;
static constexpr uint32_t INVERT_HOLD_MS = 120;

// Madgwick gains
static constexpr float MADGWICK_BETA_FLIGHT = 0.02f; // unchanged (low = trust gyro)
static constexpr float MADGWICK_BETA_GROUND = 0.50f; // disarmed: converge fast
static constexpr float MADGWICK_ZETA = 0.001f;       // gyro-bias tracking, 0 = off
MadgwickIMU madgwick(MADGWICK_BETA_GROUND, MADGWICK_ZETA);

// Mixer (X-quad FRD: FL/BR CW, FR/BL CCW)     (UNCHANGED)
//                                FL,  FR,  BR,  BL
static const float MIX_ROLL[4] = {-1, +1, +1, -1};
static const float MIX_PITCH[4] = {-1, -1, +1, +1};
static const float MIX_YAW[4] = {+1, -1, +1, -1};

static float motorTrims[4] = {1.00f, 1.00f, 1.00f, 1.15f};
static constexpr bool MIXER_TRIM_AWARE_HEADROOM = true; // see mixAndApply()

static constexpr float IDLE_THROTTLE = 0.05f;

// ─────────────────────────────────────────────────────────────────────────────
//  Drag-force velocity observer parameters
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float DRAG_C = 0.25f;      // μ/m [1/s], tune 0.20-0.30
static constexpr float VEL_GAIN = 12.0f;    // observer PT1 1/τ
static constexpr float NMNI_THRESH = 0.02f; // deadband on drag accel [m/s²]
static constexpr float BRAKE_GAIN = 0.06f;  // rad lean per m/s
static constexpr float VEL_CLAMP = 2.0f;    // m/s  (rev-3: 3.0 — see header §9)
static constexpr float VEL_DECAY_LOWTHR = 0.98f;
// Below this (smoothed) thrust the drone is assumed to be on the ground, where
// the accel reads gravity/tilt instead of drag → observer output is decayed.
// Take off from LEVEL ground: a tilted pad leaks into the estimate until airborne.
static constexpr float OBS_MIN_THRUST = 0.20f;

static float vel_x = 0.0f; // body-forward, m/s
static float vel_y = 0.0f; // body-right, m/s

// ─────────────────────────────────────────────────────────────────────────────
//  Level trim (radians, applied to the angle setpoint) — set from UI / NVS
// ─────────────────────────────────────────────────────────────────────────────
static float trimRollRad = 0.0f;
static float trimPitchRad = 0.0f;
static bool trimDirty = false; // needs saving to NVS (deferred while armed)

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
//  IMU filter pipelines (dual accel paths)
//   GYRO LIGHT  (PT1 120 Hz)  → Madgwick attitude
//   GYRO HEAVY  (PT2  90 Hz)  → PID rate-loop measurement
//   ACCEL MADG  (PT1  50 Hz)  → Madgwick attitude (quieter)
//   ACCEL OBS   (PT1 100 Hz)  → velocity observer  (drag bump preserved)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float GYRO_LIGHT_HZ = 120.0f;
static constexpr float GYRO_HEAVY_HZ = 90.0f;
static constexpr float ACCEL_MADG_HZ = 50.0f;
static constexpr float ACCEL_OBS_HZ = 100.0f;

static constexpr float GYRO_NOTCH_HZ = 0.0f; // 0 = off
static constexpr float GYRO_NOTCH_CUTOFF_HZ = 0.0f;

struct ImuFilters
{
  fc::PT1 gxLight, gyLight, gzLight;
  fc::PT2 gxHeavy, gyHeavy, gzHeavy;
  fc::Biquad gxNotch, gyNotch, gzNotch;
  fc::PT1 axMadg, ayMadg, azMadg;
  fc::PT1 axObs, ayObs;
};
static ImuFilters F;

// ─────────────────────────────────────────────────────────────────────────────
//  PID controllers
// ─────────────────────────────────────────────────────────────────────────────
static PID pidRoll, pidPitch, pidYaw;
static constexpr float ANGLE_TO_RATE_P = 4.0f;

// ─────────────────────────────────────────────────────────────────────────────
//  Diagnostics
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t lateTickCount = 0;            // ticks with dt > 1.5× nominal
static uint32_t maxTickDtUs = 0;              // worst dt in the current report window
static float dbgAx = 0, dbgAy = 0, dbgAz = 0; // observer-path accel (g) for bench test

// ═════════════════════════════════════════════════════════════════════════════
//  Small helpers
// ═════════════════════════════════════════════════════════════════════════════
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ═════════════════════════════════════════════════════════════════════════════
//  NVS persistence (level calibration + trims)
// ═════════════════════════════════════════════════════════════════════════════
static constexpr uint8_t CALIB_VERSION = 2;

static void saveLevelCalibration()
{
  float rot[9];
  imu.getRotation(rot);
  prefs.begin("fc", false);
  prefs.putUChar("calv", CALIB_VERSION);
  prefs.putBytes("rot", rot, sizeof(rot));
  prefs.putShort("abx", imu.accelBiasX);
  prefs.putShort("aby", imu.accelBiasY);
  prefs.putShort("abz", imu.accelBiasZ);
  prefs.putBytes("ascale", imu.accelScale, sizeof(imu.accelScale));
  prefs.putBytes("aoff", imu.accelOffset, sizeof(imu.accelOffset));
  prefs.end();
}

static bool loadLevelCalibration()
{
  prefs.begin("fc", true);
  bool ok = false;
  if (prefs.getUChar("calv", 0) == CALIB_VERSION && prefs.getBytesLength("rot") == 9 * sizeof(float))
  {
    float rot[9];
    prefs.getBytes("rot", rot, sizeof(rot));
    const int16_t abx = prefs.getShort("abx", 0);
    const int16_t aby = prefs.getShort("aby", 0);
    const int16_t abz = prefs.getShort("abz", 0);
    imu.setCalibration(rot, abx, aby, abz);
    if (prefs.getBytesLength("ascale") == sizeof(imu.accelScale) &&
        prefs.getBytesLength("aoff") == sizeof(imu.accelOffset))
    {
      float sc[3], of[3];
      prefs.getBytes("ascale", sc, sizeof(sc));
      prefs.getBytes("aoff", of, sizeof(of));
      imu.setAccelSensorCal(sc, of);
    }
    ok = true;
  }
  prefs.end();
  return ok;
}

static void saveTrims()
{
  prefs.begin("fc", false);
  prefs.putFloat("trimR", trimRollRad * RAD2DEG);
  prefs.putFloat("trimP", trimPitchRad * RAD2DEG);
  prefs.end();
  trimDirty = false;
}

static void loadTrims()
{
  prefs.begin("fc", true);
  const float r = prefs.getFloat("trimR", 0.0f);
  const float p = prefs.getFloat("trimP", 0.0f);
  prefs.end();
  web.setTrim(r, p); // clamps
  float rr, pp;
  web.getTrim(rr, pp);
  trimRollRad = rr * DEG2RAD;
  trimPitchRad = pp * DEG2RAD;
}

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
//  Airmode-style mixer with saturation feedback  (trim-aware head-room)
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

  // 2) Auto-scale if differential exceeds available head-room
  const float diffSpan = dMax - dMin;
  float scale = 1.0f;
  if (diffSpan > (1.0f - IDLE_THROTTLE))
  {
    scale = (1.0f - IDLE_THROTTLE) / diffSpan;
    for (int i = 0; i < 4; ++i)
      diff[i] *= scale;
  }

  // 3) Centre throttle so no motor clips floor or ceiling.
  //    Motor i outputs (thr + diff[i]) * trim[i]  ∈ [IDLE, 1]
  //    MIXER_TRIM_AWARE_HEADROOM = true : trims are inside the head-room maths
  //      → collective is capped at 1/max(trim) (0.87 with BL = 1.15) but the
  //        differential (attitude authority) is preserved at full stick.
  //    false : rev-3 behaviour — trims applied after, BL clips at 1.0 above
  //        ~0.87 collective, torque error + "saturated" flag every tick.
  float maxThr = +1e9f, minThr = -1e9f;
  for (int i = 0; i < 4; ++i)
  {
    const float invTrim = MIXER_TRIM_AWARE_HEADROOM ? (1.0f / motorTrims[i]) : 1.0f;
    const float hi = invTrim - diff[i];
    const float lo = IDLE_THROTTLE * invTrim - diff[i];
    if (hi < maxThr)
      maxThr = hi;
    if (lo > minThr)
      minThr = lo;
  }
  float thr = base;
  if (thr > maxThr)
    thr = maxThr;
  if (thr < minThr)
    thr = minThr; // floor wins if both violated (keeps props spinning)

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
//  Velocity observer  (corrected — see header §1; no gravity subtraction)
// ═════════════════════════════════════════════════════════════════════════════
static void updateVelocityObserver(float ax_g_obs, float ay_g_obs, float dt)
{
  // 1) Body-frame specific force on x/y is drag only (thrust is along -Z).
  //    This code's accel reads m = -f/g, so ax_meas = +DRAG_C·u/g.
  float ax_drag = ax_g_obs * GRAV; // = +DRAG_C·u  [m/s²]
  float ay_drag = ay_g_obs * GRAV; // = +DRAG_C·v

  // 2) Noise deadband
  if (fabsf(ax_drag) < NMNI_THRESH)
    ax_drag = 0.0f;
  if (fabsf(ay_drag) < NMNI_THRESH)
    ay_drag = 0.0f;

  // 3) Algebraic instantaneous velocity from the drag model
  const float instVx = ax_drag / DRAG_C;
  const float instVy = ay_drag / DRAG_C;

  // 4) PT1 observer correction (α = VEL_GAIN·dt)
  const float alpha = clampf(VEL_GAIN * dt, 0.0f, 1.0f);
  vel_x += (instVx - vel_x) * alpha;
  vel_y += (instVy - vel_y) * alpha;

  // 5) Safety clamp
  vel_x = clampf(vel_x, -VEL_CLAMP, VEL_CLAMP);
  vel_y = clampf(vel_y, -VEL_CLAMP, VEL_CLAMP);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Single FC tick — runs at ~LOOP_HZ, dt = measured (clamped)
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

  // ── 3. Observer accel path (lighter filtering) ────────────────────────────
  const float axO = F.axObs.update(sample.ax_g());
  const float ayO = F.ayObs.update(sample.ay_g());
  dbgAx = axO;
  dbgAy = ayO;
  dbgAz = azM;

  // ── 4. Madgwick attitude ──────────────────────────────────────────────────
  madgwick.beta = cmdSmooth.armed ? MADGWICK_BETA_FLIGHT : MADGWICK_BETA_GROUND;
  madgwick.update(gxL, gyL, gzL, axM, ayM, azM, dt);
  madgwick.toEulerFRD(att.roll, att.pitch, att.yaw);

  // ── 5. Velocity observer ──────────────────────────────────────────────────
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

  // ── 8. Reset / decay logic ────────────────────────────────────────────────
  //   • Disarmed            → HARD reset of PIDs (attitude kept). The velocity
  //                           observer keeps running for the bench test /
  //                           telemetry only — it is zeroed on the arm edge.
  //   • Armed + thr < OBS_MIN_THRUST → SOFT decay velocity (on the ground the
  //                           accel reads gravity, not drag), PIDs untouched.
  static bool prevArmed = false;
  if (!cmdSmooth.armed)
  {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
  }
  else
  {
    if (!prevArmed)
    {
      vel_x = 0.0f; // arm edge: start clean
      vel_y = 0.0f;
    }
    if (cmdSmooth.thrust < OBS_MIN_THRUST)
    {
      vel_x *= VEL_DECAY_LOWTHR;
      vel_y *= VEL_DECAY_LOWTHR;
    }
  }
  prevArmed = cmdSmooth.armed;

  // ── 9. Cascade: stick + trim + velocity brake → angle → rate → inner PID ──
  // vel_x > 0 (forward drift) → +pitch (nose up)   → decelerates forward
  // vel_y > 0 (right drift)   → -roll  (left lean) → decelerates rightward
  const float rollAngleSet = cmdSmooth.roll * MAX_ANGLE_RAD + trimRollRad - vel_y * BRAKE_GAIN;
  const float pitchAngleSet = cmdSmooth.pitch * MAX_ANGLE_RAD + trimPitchRad + vel_x * BRAKE_GAIN;

  const float rollRateSet = clampf(ANGLE_TO_RATE_P * (rollAngleSet - att.roll), -MAX_RATE_RAD, MAX_RATE_RAD);
  const float pitchRateSet = clampf(ANGLE_TO_RATE_P * (pitchAngleSet - att.pitch), -MAX_RATE_RAD, MAX_RATE_RAD);
  const float yawRateSet = cmdSmooth.yaw * MAX_YAW_RATE_RAD;

  const float rollCmd = pidRoll.update(dt, rollRateSet, att.rollRate);
  const float pitchCmd = pidPitch.update(dt, pitchRateSet, att.pitchRate);
  const float yawCmd = pidYaw.update(dt, yawRateSet, -att.yawRate); // (UNCHANGED sign)

  // ── 10. Mix + write + saturation feedback (1 tick late) ───────────────────
  const bool sat = mixAndApply(cmdSmooth.thrust, rollCmd, pitchCmd, yawCmd, cmdSmooth.armed);
  pidRoll.outputSaturated = sat;
  pidPitch.outputSaturated = sat;
  pidYaw.outputSaturated = sat;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Level calibration (blocking ~2.5 s). Only ever called while DISARMED.
// ═════════════════════════════════════════════════════════════════════════════
static void doLevelCalibration()
{
  cutMotors();
  ledColor(255, 160, 0); // amber = calibrating, don't touch
  web.setTelemetry(0, 0, 0, 0, 0, "calibrating");
  imu.calibrate(500);
  saveLevelCalibration();
  madgwick.reset(); // level again by definition, drop bias estimate too
  Serial.println(imu.calibrationToString());
  ledColor(0, 0, 0);
}

// ═════════════════════════════════════════════════════════════════════════════
//  6-face accelerometer calibration ("silicon offsets": ellipsoid → sphere)
//
//  Model per axis k (in the frame readRaw() outputs, i.e. after sign flips):
//        a_k [g] = (raw_k - offset_k) * scale_k / SENS
//  Classic 6-position: for each axis take the face with the largest +reading
//  and the one with the largest -reading (dominant faces, found automatically —
//  labels/order don't matter):
//        offset_k = (pos_k + neg_k) / 2         scale_k = 2·SENS / (pos_k - neg_k)
//  Then every face is checked against |a| = 1 g (residual). Cross-axis
//  misalignment is not modelled (it is small on the MPU6050 and the mounting
//  rotation R absorbs the part that matters for "level").
//
//  Driven from the web UI: /cal6?cmd=capture (six times) → after the sixth
//  capture the solve runs, the result is stored, and — because the LAST pose
//  is "flat on the level surface" — the level calibration runs on it.
// ═════════════════════════════════════════════════════════════════════════════
static constexpr int CAL6_SAMPLES = 1000; // ~2 s per face
static float cal6Face[6][3];
static int cal6Next = 0; // 0..5 = next face to capture, 6 = done

static void cal6Reset()
{
  cal6Next = 0;
  web.setCal6State(0, "ready — capture pose 1");
}

static bool cal6Capture(float out[3])
{
  MPU6050::Data d;
  for (int i = 0; i < 100; ++i)
  {
    imu.readRawSample(d);
    delay(2);
  }
  double sx = 0, sy = 0, sz = 0;
  int n = 0, fails = 0;
  while (n < CAL6_SAMPLES && fails < 200)
  {
    if (imu.readRawSample(d))
    {
      sx += d.accelX;
      sy += d.accelY;
      sz += d.accelZ;
      ++n;
    }
    else
      ++fails;
    delay(2);
  }
  if (n < CAL6_SAMPLES / 2)
    return false;
  out[0] = (float)(sx / n);
  out[1] = (float)(sy / n);
  out[2] = (float)(sz / n);
  return true;
}

static bool cal6Solve(String &report)
{
  const float SENS = MPU6050::Data::ACCEL_SENS_4G;
  float scale[3], offset[3];
  for (int k = 0; k < 3; ++k)
  {
    float pos = -1e9f, neg = +1e9f;
    for (int i = 0; i < 6; ++i)
    {
      if (cal6Face[i][k] > pos)
        pos = cal6Face[i][k];
      if (cal6Face[i][k] < neg)
        neg = cal6Face[i][k];
    }
    if (pos < 0.7f * SENS || neg > -0.7f * SENS)
    {
      report = String("FAILED: axis ") + "XYZ"[k] + " never pointed up AND down (pos " +
               String(pos, 0) + ", neg " + String(neg, 0) + "). Restart.";
      return false;
    }
    offset[k] = 0.5f * (pos + neg);
    scale[k] = 2.0f * SENS / (pos - neg);
    if (fabsf(scale[k] - 1.0f) > 0.10f || fabsf(offset[k]) > 1200.0f)
    {
      report = String("FAILED: implausible axis ") + "XYZ"[k] + " scale " + String(scale[k], 4) +
               " offset " + String(offset[k], 0) + ". Restart.";
      return false;
    }
  }

  // Residuals on all six faces
  float maxErr = 0.0f;
  String res;
  for (int i = 0; i < 6; ++i)
  {
    const float ax = (cal6Face[i][0] - offset[0]) * scale[0] / SENS;
    const float ay = (cal6Face[i][1] - offset[1]) * scale[1] / SENS;
    const float az = (cal6Face[i][2] - offset[2]) * scale[2] / SENS;
    const float mag = sqrtf(ax * ax + ay * ay + az * az);
    const float err = fabsf(mag - 1.0f);
    if (err > maxErr)
      maxErr = err;
    res += "face " + String(i + 1) + " |a|=" + String(mag, 4) + "\n";
  }
  const char *grade = maxErr > 0.015f ? "POOR (redo)" : maxErr > 0.005f ? "acceptable"
                                                                        : "excellent";

  imu.setAccelSensorCal(scale, offset);

  report = "scale " + String(scale[0], 5) + " " + String(scale[1], 5) + " " + String(scale[2], 5) +
           "\noffset " + String(offset[0], 1) + " " + String(offset[1], 1) + " " + String(offset[2], 1) +
           " LSB\n" + res + "max err " + String(maxErr, 4) + " g → " + grade;
  return true;
}

// Called from loop() (disarmed) when the UI asks for a capture.
static void cal6HandleCapture()
{
  if (cal6Next >= 6)
    cal6Reset();
  ledColor(255, 160, 0);
  web.setCal6State(cal6Next, "capturing pose " + String(cal6Next + 1) + " — hold still");
  float f[3];
  if (!cal6Capture(f))
  {
    web.setCal6State(cal6Next, "IMU read failed, try again");
    ledColor(0, 0, 0);
    return;
  }
  for (int k = 0; k < 3; ++k)
    cal6Face[cal6Next][k] = f[k];
  Serial.printf("cal6 face %d: %.1f %.1f %.1f\n", cal6Next + 1, f[0], f[1], f[2]);
  ++cal6Next;

  if (cal6Next < 6)
  {
    web.setCal6State(cal6Next, "pose " + String(cal6Next) + " ok (" + String(f[0], 0) + ", " +
                                   String(f[1], 0) + ", " + String(f[2], 0) + ") — next pose");
    ledColor(0, 0, 0);
    return;
  }

  String report;
  if (!cal6Solve(report))
  {
    Serial.println(report);
    cal6Next = 0;
    web.setCal6State(0, report);
    ledColor(0, 0, 0);
    return;
  }
  Serial.println(report);
  // Pose 6 = flat on the level surface → redo the level/bias/gyro calibration
  // with the new sensor-frame correction, then store everything.
  web.setCal6State(6, report + "\nlevel calibration running…");
  imu.calibrate(500);
  saveLevelCalibration();
  madgwick.reset();
  Serial.println(imu.calibrationToString());
  web.setCal6State(6, report + "\nsaved + level calibrated ✔");
  ledColor(0, 0, 0);
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

  // ── IMU calibration ─────────────────────────────────────────────────────
  //  Level rotation + accel bias: from NVS if present, else measure now
  //  (drone must be on a level surface for that first boot!) and save.
  //  Gyro zero: ALWAYS re-measured at boot (temperature dependent).
  if (loadLevelCalibration())
  {
    Serial.println("Level calibration loaded from NVS");
    imu.calibrateGyro(500); // ~1.1 s, keep still
  }
  else
  {
    Serial.println("No level calibration in NVS — calibrating now (keep level & still)");
    ledColor(255, 160, 0);
    imu.calibrate(500);
    saveLevelCalibration();
  }
  loadTrims();

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

  cal6Reset();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println(imu.calibrationToString());
  Serial.printf("Trim: roll %.2f deg, pitch %.2f deg\n", trimRollRad * RAD2DEG, trimPitchRad * RAD2DEG);

  delay(2000);
  ledColor(0, 0, 0);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main loop — ~500 Hz FC tick (measured dt) + ~100 Hz web service
// ═════════════════════════════════════════════════════════════════════════════
void loop()
{
  static uint32_t nextTickUs = 0;    // scheduled time of the next tick
  static uint32_t lastTickRunUs = 0; // when the previous tick actually ran
  static uint32_t lastWebMs = 0;
  static uint32_t lastDbgMs = 0;

  const uint32_t nowMs = millis();

  // ── Web at ~100 Hz, between FC ticks ─────────────────────────────────────
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

    if (web.takeTrimChanged())
    {
      float r, p;
      web.getTrim(r, p);
      trimRollRad = r * DEG2RAD;
      trimPitchRad = p * DEG2RAD;
      trimDirty = true;
    }
    if (trimDirty && !cmdRaw.armed)
      saveTrims(); // NVS write only while disarmed (takes a few ms)

    if (web.takeCalibrateRequest() && !cmdRaw.armed && mpuOk)
      doLevelCalibration();

    const int c6 = web.takeCal6Request();
    if (c6 != 0 && !cmdRaw.armed && mpuOk)
    {
      if (c6 == 2)
        cal6Reset();
      else
        cal6HandleCapture();
    }

    web.setTelemetry(att.roll * RAD2DEG, att.pitch * RAD2DEG, att.yaw * RAD2DEG,
                     vel_x, vel_y, cmdRaw.armed ? "armed" : "disarmed");
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

  // ── FC tick ──────────────────────────────────────────────────────────────
  const uint32_t nowUs = micros();
  if (nextTickUs == 0)
  {
    nextTickUs = nowUs;
    lastTickRunUs = nowUs - LOOP_DT_US;
  }
  if ((int32_t)(nowUs - nextTickUs) >= 0)
  {
    // Schedule the next tick. If we're more than one period late (WiFi stall,
    // calibration…) re-sync instead of running catch-up ticks back-to-back.
    nextTickUs += LOOP_DT_US;
    if ((int32_t)(nowUs - nextTickUs) >= 0)
      nextTickUs = nowUs + LOOP_DT_US;

    // Measured dt since the previous tick actually ran, clamped [0.5×, 4×]
    const uint32_t elapsedUs = nowUs - lastTickRunUs;
    lastTickRunUs = nowUs;
    float dt = elapsedUs * 1e-6f;
    if (dt < LOOP_DT_MIN_S)
      dt = LOOP_DT_MIN_S;
    if (dt > LOOP_DT_MAX_S)
      dt = LOOP_DT_MAX_S;
    if (elapsedUs > (uint32_t)(1.5f * LOOP_DT_US))
      lateTickCount++;
    if (elapsedUs > maxTickDtUs)
      maxTickDtUs = elapsedUs;

    if (imu.update())
    {
      flightTick(imu.data(), dt);
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

  // ── Serial diagnostics — DISARMED ONLY (a stalled USB-CDC write must never
  //    block the flight loop). lateTickCount keeps counting while armed, so
  //    you can read it after landing.
  if (!cmdRaw.armed && Serial && (nowMs - lastDbgMs >= 200))
  {
    lastDbgMs = nowMs;
    Serial.printf("acc(g) %+.3f %+.3f %+.3f | att(deg) r%+.1f p%+.1f y%+.1f | vel %+.2f %+.2f | gbias(dps) %+.2f %+.2f | late %lu max %lu us\n",
                  dbgAx, dbgAy, dbgAz,
                  att.roll * RAD2DEG, att.pitch * RAD2DEG, att.yaw * RAD2DEG,
                  vel_x, vel_y,
                  madgwick.wbx * RAD2DEG, madgwick.wby * RAD2DEG,
                  (unsigned long)lateTickCount, (unsigned long)maxTickDtUs);
    maxTickDtUs = 0;
  }

  yield();
}
