/***********************************************************************
 *  LOLIN C3 Pico – MPU6050 + NeoPixel, modular version
 *
 *  - Motor pin definitions kept, but not used.
 *  - MPU6050 configured for drone-style use:
 *      * Gyro:  ±2000 Degr/s
 *      * Accel: ±4 g
 *      * DLPF:  42 Hz
 *      * Fs:    500 Hz output
 *  - Mpu6050Data struct:
 *      * Holds raw data.
 *      * Provides scaling helpers (g, Degr/s, rad/s).
 *      * Provides toString() for nice logs.
 ***********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
// --------------------------------------------------------------------
// Pin map
// --------------------------------------------------------------------

const int motorPin1 = 4;
const int motorPin2 = 0;
const int motorPin3 = 5;
const int motorPin4 = 6;
const int motorPins[4] = {motorPin4, motorPin1, motorPin2, motorPin3};
const int ledIndex[4] = {3, 0, 1, 2};

#define LED_PIN 7
#define LED_COUNT 4
#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 8

Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --------------------------------------------------------------------
// Drone control center
// --------------------------------------------------------------------

struct Command
{
  // [-1,1] sticks values
  float roll;  // desired roll (or roll angle / rate, depending on mode)
  float pitch; // desired pitch
  float yaw;   // desired yaw rate (typically)
  // [0,1]
  float thrust; // collective thrust command
};

enum ControlMode
{
  MODE_ANGLE_RATE, // outer angle P -> inner rate PID
};

ControlMode controlMode = MODE_ANGLE_RATE;

static constexpr float DEG2RAD = PI / 180.0f;

// For debug printing
const char *armNames[4] = {"FL", "FR", "BR", "BL"};

// Mixer signs for an X quad in FRD:
//
// roll > 0 (right wing UP) -> increase thrust on left motors (FL, BL)
// pitch > 0 (nose up)          -> increase thrust on front motors (FL, FR)
// yaw sign depends on spin directions; assuming FL/BR CCW, FR/BL CW:
const float MIX_ROLL[4] = {+1.0f, -1.0f, -1.0f, +1.0f};  // [FL, FR, BR, BL]
const float MIX_PITCH[4] = {+1.0f, +1.0f, -1.0f, -1.0f}; // [FL, FR, BR, BL]
const float MIX_YAW[4] = {-1.0f, +1.0f, -1.0f, +1.0f};   // [FL, FR, BR, BL]

// Outer P gain: angle error [rad] -> desired rate [rad/s]
const float MAX_ANGLE_DEG = 18.0f;    // max commanded tilt
const float MAX_YAW_RATE_DPS = 90.0f; // max commanded yaw rate
const float ANGLE_TO_RATE_P = 5.0f;   // tweak later

// Angle-rate mode tuning
const float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG2RAD;
const float MAX_YAW_RATE_RAD = MAX_YAW_RATE_DPS * DEG2RAD;

// --------------------------------------------------------------------
// MPU6050 definitions & constants
// --------------------------------------------------------------------

const uint8_t MPU6050_ADDR = 0x68;
const uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REG_CONFIG = 0x1A;
const uint8_t MPU6050_REG_GYRO_CFG = 0x1B;
const uint8_t MPU6050_REG_ACCEL_CFG = 0x1C;
const uint8_t MPU6050_REG_PWR = 0x6B;
const uint8_t MPU6050_REG_WHOAMI = 0x75;
const uint8_t MPU6050_REG_ACCEL = 0x3B; // ACCEL_XOUT_H

// Chosen full-scale ranges:
//  - Accel: +-4 g  => 8192 LSB/g
//  - Gyro:  +-2000 degr/s => 16.4 LSB/(Degr/s)
static constexpr float ACCEL_SENS_4G = 8192.0f;
static constexpr float GYRO_SENS_2000DPS = 16.4f;

bool mpuOk = false; // true if MPU6050 initialized OK

// --------------------------------------------------------------------
// Struct to hold one MPU6050 sample (raw) + helpers
// --------------------------------------------------------------------
// ---------------- Attitude state (what we feed into PIDs later) -----
struct AttitudeState
{
  float roll;  // rad
  float pitch; // rad
  float yaw;   // rad (relative, will drift without mag)

  float rollRate;  // rad/s
  float pitchRate; // rad/s
  float yawRate;   // rad/s
};
// ---------------- Attitude state instance ----------------------------
AttitudeState att; // global attitude state (rad / rad/s, FRD)

// --------------------------------------------------------------------
// Madgwick IMU filter (6-DOF, gyro + accel, FRD)
// --------------------------------------------------------------------

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

MadgwickIMU madgwick(0.03f); // global filter instance

// --------------------------------------------------------------------
// IMU calibration offsets (in raw sensor counts, FRD frame)
// --------------------------------------------------------------------
int16_t accelBiasX = 0;
int16_t accelBiasY = 0;
int16_t accelBiasZ = 0;

int16_t gyroBiasX = 0;
int16_t gyroBiasY = 0;
int16_t gyroBiasZ = 0;

struct Mpu6050Data
{
  // Raw sensor data
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  float temperatureC;
  unsigned long timestampMs;

  // ---- Scaling helpers: accel in g ---------------------------------
  float ax_g() const
  {
    return accelX / ACCEL_SENS_4G;
  }
  float ay_g() const
  {
    return accelY / ACCEL_SENS_4G;
  }
  float az_g() const
  {
    return accelZ / ACCEL_SENS_4G;
  }

  float accelMagnitude_g() const
  {
    const float x = ax_g();
    const float y = ay_g();
    const float z = az_g();
    return sqrtf(x * x + y * y + z * z);
  }

  // ---- Scaling helpers: gyro in Degr/s and rad/s -------------------
  float gx_dps() const
  {
    return gyroX / GYRO_SENS_2000DPS;
  }
  float gy_dps() const
  {
    return gyroY / GYRO_SENS_2000DPS;
  }
  float gz_dps() const
  {
    return gyroZ / GYRO_SENS_2000DPS;
  }

  float gx_rad() const
  {
    return gx_dps() * DEG2RAD;
  }
  float gy_rad() const
  {
    return gy_dps() * DEG2RAD;
  }
  float gz_rad() const
  {
    return gz_dps() * DEG2RAD;
  }
  String toString() const
  {
    String s;
    s.reserve(160);

    s += "t=";
    s += timestampMs;
    s += " ms | Accel[g]: (";
    s += ax_g();
    s += ", ";
    s += ay_g();
    s += ", ";
    s += az_g();
    s += ") | Gyro[dps]: (";
    s += gx_dps();
    s += ", ";
    s += gy_dps();
    s += ", ";
    s += gz_dps();
    s += ") | Temp=";
    s += temperatureC;
    s += " C";

    return s;
  }
};
// ---------------------------------------------------------------------
struct PID
{
  float kp;
  float ki;
  float kd;

  float integrator;
  float prevError;
  float outMin, outMax; // clamp

  PID(float p = 0, float i = 0, float d = 0, float minOut = -1.0f, float maxOut = 1.0f)
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
// PID controllers for roll, pitch, yaw
PID pidRoll(0.12f, 0.40f, 0.0020f);
PID pidPitch(0.12f, 0.40f, 0.0020f);
PID pidYaw(0.08f, 0.25f, 0.0000f);

// --------------------------------------------------------------------
// LED helpers
// --------------------------------------------------------------------

void ledColor(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t c = leds.Color(r, g, b);
  for (int i = 0; i < LED_COUNT; i++)
  {
    leds.setPixelColor(i, c);
  }
  leds.show();
}

void setLED(uint8_t r, uint8_t g, uint8_t b, int i)
{
  if (i < 0 || i >= LED_COUNT)
    return;
  leds.setPixelColor(i, leds.Color(r, g, b));
  leds.show();
}

// ---------------------------------------------------------------------
// Motor helpers
// ---------------------------------------------------------------------

void setSoloMotorPWM(int idx, uint8_t pwmValue)
{
  analogWrite(motorPins[idx], pwmValue);
}

void setMotors(const uint8_t pwmValues[4])
{
  for (int i = 0; i < 4; i++)
  {
    // Current "PWM" for this motor (0..255)
    uint8_t pwm = pwmValues[i];

    // Map 0..255 -> green..red
    uint8_t red = pwm;         // more thrust -> more red
    uint8_t green = 255 - pwm; // less thrust -> more green
    uint8_t blue = 0;

    // Show on the corresponding LED instead of driving the motor
    // set led brightness
    leds.setBrightness((pwm - 128) * 2);
    setLED(red, green, blue, ledIndex[i]);

    // When you want real motors:
    // analogWrite(motorPins[i], pwmValues[i]);
  }
}

// Convert normalized [0..1] motor outputs to PWM + LEDs
void applyMotorOutputs(const float motorOut[4])
{
  uint8_t pwm[4];

  Serial.print("MOTORS: ");
  for (int i = 0; i < 4; ++i)
  {
    // clamp [0,1]
    float v = motorOut[i];
    if (v < 0.0f)
      v = 0.0f;
    if (v > 1.0f)
      v = 1.0f;

    // map to 0..255
    uint8_t p = (uint8_t)(v * 255.0f + 0.5f);
    pwm[i] = p;

    Serial.print(armNames[i]);
    Serial.print("=");
    Serial.print(v, 3);
    if (i < 3)
      Serial.print("  ");
  }
  Serial.println();

  // Use your existing LED visualiser + (optionally) motors
  setMotors(pwm);
}

// Angle-rate control:
//  - cmd.roll/pitch in [-1,1] -> desired angles
//  - outer P: angle error -> desired rate
//  - inner PID: rate error -> control signal in [-1,1]
//  - mixed into 4 motors
void controlAngleRate(const Command &cmd, float dt)
{
  // 1) Desired angles from sticks
  float rollAngleSet = cmd.roll * MAX_ANGLE_RAD;   // rad
  float pitchAngleSet = cmd.pitch * MAX_ANGLE_RAD; // rad

  // 2) Angle error -> desired angular rates (rad/s)
  float rollAngleErr = rollAngleSet - att.roll;
  float pitchAngleErr = pitchAngleSet - att.pitch;

  float rollRateSet = ANGLE_TO_RATE_P * rollAngleErr;
  float pitchRateSet = ANGLE_TO_RATE_P * pitchAngleErr;

  // Limit desired rates a bit
  const float MAX_RATE_RAD = 120.0f * DEG2RAD; // safety clamp
  if (rollRateSet > MAX_RATE_RAD)
    rollRateSet = MAX_RATE_RAD;
  if (rollRateSet < -MAX_RATE_RAD)
    rollRateSet = -MAX_RATE_RAD;
  if (pitchRateSet > MAX_RATE_RAD)
    pitchRateSet = MAX_RATE_RAD;
  if (pitchRateSet < -MAX_RATE_RAD)
    pitchRateSet = -MAX_RATE_RAD;

  // Yaw: directly commanded as rate
  float yawRateSet = cmd.yaw * MAX_YAW_RATE_RAD;

  // 3) Inner rate PIDs (already defined globally)
  float rollCmd = pidRoll.update(dt, rollRateSet, att.rollRate);
  float pitchCmd = pidPitch.update(dt, pitchRateSet, att.pitchRate);
  float yawCmd = pidYaw.update(dt, yawRateSet, att.yawRate);

  // 4) Mixer: base thrust + axis commands
  float motorOut[4];
  float base = cmd.thrust; // [0..1]

  for (int i = 0; i < 4; ++i)
  {
    motorOut[i] = base + rollCmd * MIX_ROLL[i] + pitchCmd * MIX_PITCH[i] + yawCmd * MIX_YAW[i];
  }

  // 5) Apply (clamp & send to LEDs/motors)
  applyMotorOutputs(motorOut);
}

// --------------------------------------------------------------------
// I2C helpers for MPU6050
// --------------------------------------------------------------------

bool mpuWriteByte(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0)
  { // repeated start
    return false;
  }

  uint8_t read = Wire.requestFrom(MPU6050_ADDR, length, true);
  if (read != length)
  {
    return false;
  }

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = Wire.read();
  }
  return true;
}

// --------------------------------------------------------------------
// MPU6050 init & read
// --------------------------------------------------------------------

bool initMpu6050()
{
  // Check WHO_AM_I
  uint8_t who = 0;
  if (!mpuReadBytes(MPU6050_REG_WHOAMI, &who, 1))
  {
    Serial.println("MPU6050: Failed to read WHO_AM_I");
    return false;
  }

  if (!(who == 0x70 || who == 0x69 || who == 0x68))
  {
    Serial.print("MPU6050: Unexpected WHO_AM_I: 0x");
    Serial.println(who, HEX);
    return false;
  }

  // Wake up (clear sleep bit, internal 8 MHz clock)
  if (!mpuWriteByte(MPU6050_REG_PWR, 0x00))
  {
    Serial.println("MPU6050: Failed to write PWR_MGMT_1");
    return false;
  }

  // Set DLPF to 42 Hz (CONFIG)
  // DLPF_CFG = 3 -> 42 Hz gyro/accel bandwidth, 1 kHz internal sample
  if (!mpuWriteByte(MPU6050_REG_CONFIG, 0x03))
  {
    Serial.println("MPU6050: Failed to write CONFIG");
    return false;
  }

  // Sample rate divider: Fs = 1000 / (1 + div)
  // div = 1 -> 500 Hz output sample rate
  if (!mpuWriteByte(MPU6050_REG_SMPLRT_DIV, 1))
  {
    Serial.println("MPU6050: Failed to write SMPLRT_DIV");
    return false;
  }

  // Gyro full scale: FS_SEL = 3 -> ±2000 Degr/s
  // Bits 4:3 = 11b => 0x18
  if (!mpuWriteByte(MPU6050_REG_GYRO_CFG, 0x18))
  {
    Serial.println("MPU6050: Failed to write GYRO_CONFIG");
    return false;
  }

  // Accel full scale: AFS_SEL = 1 -> ±4 g
  // Bits 4:3 = 01b => 0x08
  if (!mpuWriteByte(MPU6050_REG_ACCEL_CFG, 0x08))
  {
    Serial.println("MPU6050: Failed to write ACCEL_CONFIG");
    return false;
  }

  Serial.println("MPU6050: Initialized for drone-style settings");
  return true;
}

// --------------------------------------------------------------------
// Apply calibration offsets to a sample (still in raw counts)
// --------------------------------------------------------------------
void applyImuCalibration(Mpu6050Data &d)
{
  d.accelX -= accelBiasX;
  d.accelY -= accelBiasY;
  d.accelZ -= accelBiasZ;

  d.gyroX -= gyroBiasX;
  d.gyroY -= gyroBiasY;
  d.gyroZ -= gyroBiasZ;
}

bool readMpu6050(Mpu6050Data &data)
{
  uint8_t raw[14];

  if (!mpuReadBytes(MPU6050_REG_ACCEL, raw, sizeof(raw)))
    return false;

  data.accelX = -(int16_t)((raw[0] << 8) | raw[1]);
  data.accelY = (int16_t)((raw[2] << 8) | raw[3]);
  data.accelZ = (int16_t)((raw[4] << 8) | raw[5]);

  int16_t tempRaw = (int16_t)((raw[6] << 8) | raw[7]);
  data.temperatureC = (tempRaw / 340.0f) + 36.53f;

  data.gyroX = (int16_t)((raw[8] << 8) | raw[9]);
  data.gyroY = -(int16_t)((raw[10] << 8) | raw[11]);
  data.gyroZ = (int16_t)((raw[12] << 8) | raw[13]);
  data.timestampMs = millis();

  // temporarily comment this out while figuring axes
  applyImuCalibration(data);

  return true;
}

// --------------------------------------------------------------------
// Calibrate IMU biases (board must be still, flat-ish, FRD frame)
// --------------------------------------------------------------------
void calibrateImu()
{
  Serial.println(F("IMU calibration: keep the board still..."));

  const int N = 500; // number of samples to average
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;

  Mpu6050Data s;

  // Throw away a few samples to let things settle
  for (int i = 0; i < 50; ++i)
  {
    readMpu6050(s);
    delay(5);
  }

  for (int i = 0; i < N; ++i)
  {
    // Keep reading until we get a valid sample
    while (!readMpu6050(s))
    {
      delay(1);
    }

    // NOTE: s is already in FRD because readMpu6050() does the sign flips.
    sumAx += s.accelX;
    sumAy += s.accelY;
    sumAz += s.accelZ;

    sumGx += s.gyroX;
    sumGy += s.gyroY;
    sumGz += s.gyroZ;

    delay(5);
  }

  float avgAx = (float)sumAx / (float)N;
  float avgAy = (float)sumAy / (float)N;
  float avgAz = (float)sumAz / (float)N;

  float avgGx = (float)sumGx / (float)N;
  float avgGy = (float)sumGy / (float)N;
  float avgGz = (float)sumGz / (float)N;

  // In FRD at rest we *want*:
  //  accelX ≈ 0, accelY ≈ 0, accelZ ≈ +ACCEL_SENS_4G (1 g down)
  accelBiasX = (int16_t)(avgAx);                 // bring X -> 0
  accelBiasY = (int16_t)(avgAy);                 // bring Y -> 0
  accelBiasZ = (int16_t)(avgAz - ACCEL_SENS_4G); // bring Z -> +1 g

  // Gyros should be ≈ 0 at rest
  gyroBiasX = (int16_t)(avgGx);
  gyroBiasY = (int16_t)(avgGy);
  gyroBiasZ = (int16_t)(avgGz);

  Serial.println(F("IMU calibration done."));
  Serial.print(F("Accel bias [counts] = "));
  Serial.print(accelBiasX);
  Serial.print(F(", "));
  Serial.print(accelBiasY);
  Serial.print(F(", "));
  Serial.println(accelBiasZ);

  Serial.print(F("Gyro bias [counts]  = "));
  Serial.print(gyroBiasX);
  Serial.print(F(", "));
  Serial.print(gyroBiasY);
  Serial.print(F(", "));
  Serial.println(gyroBiasZ);
}

void loopFlightController(const Mpu6050Data &sample)
{
  static uint32_t lastUpdate = 0;
  const uint32_t dt_us = 2000; // 500 Hz = 2000 microseconds

  uint32_t now = micros();
  if (lastUpdate == 0)
  {
    // first call: just initialize timestamp
    lastUpdate = now;
    return;
  }

  if (now - lastUpdate < dt_us)
    return;

  float dt = (now - lastUpdate) * 1e-6f;
  lastUpdate = now;
  if (dt <= 0.0f)
    dt = 0.001f; // safety

  // 1) Get the attitude from gyro + accel (using madgwick filter here, assume FRD frame)
  madgwick.update(
      sample.gx_rad(), sample.gy_rad(), sample.gz_rad(),
      sample.ax_g(), sample.ay_g(), sample.az_g(),
      dt);

  madgwick.toEulerFRD(att.roll, att.pitch, att.yaw);
  att.rollRate = sample.gx_rad();
  att.pitchRate = sample.gy_rad();
  att.yawRate = sample.gz_rad();

  // roll: right-wing-down positive
  float rollAcc = atan2f(sample.ay_g(), sample.az_g()) * 180.0f / PI;

  // pitch: nose-down negative
  float pitchAcc = atan2f(-sample.ax_g(), sqrtf(sample.ay_g() * sample.ay_g() + sample.az_g() * sample.az_g())) * 180.0f / PI;

  // Serial.print("ACC R/P = ");
  // Serial.print(rollAcc);
  // Serial.print(", ");
  // Serial.print(pitchAcc);
  // Serial.print(" | FUSED R/P/Y = ");
  // Serial.print(att.roll * 180.0f / PI);
  // Serial.print(", ");
  // Serial.print(att.pitch * 180.0f / PI);
  // Serial.print(", ");
  // Serial.println(att.yaw * 180.0f / PI);
  // 2) Get pilot command (here: zero command for testing)
  // For testing: serial? command input can be added later
  Command cmd;
  cmd.roll = 0.0f;
  cmd.pitch = 0.0f;
  cmd.yaw = 0.0f;
  cmd.thrust = 0.2f; // 20% thrust

  // 3) Compute control from pilot command + attitude
  switch (controlMode)
  {
  case MODE_ANGLE_RATE:
  default:
    controlAngleRate(cmd, dt);
    break;
  }

  // 4) Mix to motors
  // 5) Send motors to hardware
}

// --------------------------------------------------------------------
// Setup
// --------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  for (int i = 0; i < 4; ++i)
  {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // I2C init (ESP32-C3: specify SDA/SCL pins)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // LEDs
  leds.begin();
  leds.setBrightness(50);
  ledColor(0, 0, 0); // all off

  Serial.println("MPU6050 + NeoPixel (modular) starting...");

  // Initialize MPU6050
  ledColor(0, 0, 255); // blue while initializing
  mpuOk = initMpu6050();

  if (mpuOk)
  {
    ledColor(0, 255, 0); // green if OK
    calibrateImu();
  }
  else
  {
    ledColor(255, 0, 0); // red if error
  }
}

// --------------------------------------------------------------------
// Main loop
// --------------------------------------------------------------------

void loop()
{

  if (!mpuOk)
  {
    // Blink red to indicate error
    ledColor(255, 0, 0);
    delay(200);
    ledColor(0, 0, 0);
    delay(200);
    return;
  }

  Mpu6050Data sample;
  if (readMpu6050(sample))
  {
    // For tuning/logging: scaled values
    // Serial.println(sample.toString());
    loopFlightController(sample);
  }
  else
  {
    Serial.println("MPU6050: Failed to read sensor");
    ledColor(255, 0, 0);
  }
}
