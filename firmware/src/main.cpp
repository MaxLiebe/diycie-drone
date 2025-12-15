/***********************************************************************
 *  LOLIN C3 Pico ESP32-C3 based DIY drone flight controller
 *
 *      * Gyro:  ±2000 Degr/s
 *      * Accel: ±4 g
 *      * DLPF:  42 Hz
 *      * Fs:    500 Hz output
 ***********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include <PID.h>         // PID controller
#include <MadgwickIMU.h> // Madgwick IMU filter
#include <MPU6050.h>     // MPU6050 driver

// --------------------------------------------------------------------
// Pin map
// --------------------------------------------------------------------

const int motorPin1 = 4;
const int motorPin2 = 0;
const int motorPin3 = 5;
const int motorPin4 = 6;
const int motorPins[4] = {motorPin4, motorPin1, motorPin2, motorPin3};
const int ledIndex[4] = {3, 0, 1, 2};
const char *armNames[4] = {"FL", "FR", "BR", "BL"};

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

// Mixer signs for an X quad in FRD:
//
// roll > 0 (right wing UP) -> increase thrust on left motors (FL, BL)
// pitch > 0 (nose up)          -> increase thrust on front motors (FL, FR)
// yaw sign depends on spin directions; assuming FL/BR CCW, FR/BL CW:
const float MIX_ROLL[4] = {+1.0f, -1.0f, -1.0f, +1.0f};  // [FL, FR, BR, BL]
const float MIX_PITCH[4] = {+1.0f, +1.0f, -1.0f, -1.0f}; // [FL, FR, BR, BL]
const float MIX_YAW[4] = {-1.0f, +1.0f, -1.0f, +1.0f};   // [FL, FR, BR, BL]

const float MAX_ANGLE_DEG = 18.0f;    // max commanded tilt
const float MAX_YAW_RATE_DPS = 90.0f; // max commanded yaw rate

// Angle-rate mode tuning
const float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG2RAD;
const float MAX_YAW_RATE_RAD = MAX_YAW_RATE_DPS * DEG2RAD;

// --------------------------------------------------------------------
// MPU6050 definitions & constants
// --------------------------------------------------------------------

bool mpuOk = false; // true if MPU6050 initialized OK
MPU6050 imu;

// --------------------------------------------------------------------
// Struct to hold one MPU6050 sample (raw) + helpers
// --------------------------------------------------------------------
struct AttitudeState
{
  float roll;  // rad
  float pitch; // rad
  float yaw;   // rad (relative, will drift without mag)

  float rollRate;  // rad/s
  float pitchRate; // rad/s
  float yawRate;   // rad/s
};
AttitudeState att; // global attitude state (rad / rad/s, FRD)

// --------------------------------------------------------------------
// Madgwick IMU filter (6-DOF, gyro + accel, FRD)
// --------------------------------------------------------------------

MadgwickIMU madgwick(0.03f); // global filter instance

// --------------------------------------------------------------------
// PID filters for angle-rate control
// --------------------------------------------------------------------

// PID controllers for roll, pitch, yaw
PID pidRoll(0.12f, 0.40f, 0.0020f);
PID pidPitch(0.12f, 0.40f, 0.0020f);
PID pidYaw(0.08f, 0.25f, 0.0000f);

// Outer P gain: angle error [rad] -> desired rate [rad/s]
const float ANGLE_TO_RATE_P = 5.0f; // tweak later

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

    uint16_t hueGreen = 21845; // approx 120deg
    uint16_t hueRed = 0;       // 0deg

    uint16_t hue = (uint16_t)((uint32_t)(255 - pwm) * hueGreen / 255); // green->red

    // Show on the corresponding LED instead of driving the motor
    // set led brightness
    uint32_t c = leds.gamma32(leds.ColorHSV(hue, 255, 255));
    leds.setPixelColor(ledIndex[i], c);
    leds.show();

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

void loopFlightController(const MPU6050::Data &sample)
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

  // 2) Get pilot command (here: zero command for testing)
  // For testing: serial? command input later
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
  mpuOk = imu.begin(Wire);

  if (mpuOk)
  {
    ledColor(0, 255, 0); // green if OK
    imu.calibrate(500);
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

  if (imu.update())
  {
    // For tuning/logging: scaled values
    // Serial.println(imu.get().toString());
    loopFlightController(imu.data()); // read only data from imu
  }
  else
  {
    Serial.println("MPU6050: Failed to read sensor");
    ledColor(255, 0, 0);
  }
}
