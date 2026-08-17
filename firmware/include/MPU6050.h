#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H
#include <Arduino.h>
#include <Wire.h>

// =============================================================================
//  MPU6050 — FRD body-frame IMU driver with level/bias calibration.
//
//  Changes vs previous revision (drift-related, no axis/sign changes):
//    • calibrate() now applies the accel scale factors BEFORE computing the
//      Rodrigues level rotation and the biases — exactly the same order that
//      applyCalibration() uses at run time. Previously scale was applied
//      after rotation, so the stored bias didn't match the run-time path.
//    • Z accel bias is now chosen so that a resting drone reads exactly
//      +1.000 g (8192 counts). Previously it rested at ~0.957 g.
//    • Gyro biases are rounded, not truncated (was up to 1 LSB = 0.06 dps off).
//    • New calibrateGyro(): gyro-only re-zero (fast, no level surface needed),
//      used at every boot because the MPU6050 gyro zero moves with temperature
//      while the level rotation R and accel bias are stable and can be saved.
//    • New setCalibration()/getters so main can persist R + accel bias in NVS.
//    • Sensor-frame ("silicon") accel calibration is now bias + scale per axis
//      (accelOffset[], accelScale[]) applied to the raw counts BEFORE the
//      mounting rotation — the ellipsoid → unit-sphere correction. The
//      offsets/scales come from the in-firmware 6-face routine (see main.cpp)
//      and are stored in NVS. Defaults = your old ACCEL_SCALE constants, offset 0.
//
//  Calibration pipeline (applyCalibration):
//      raw counts (sign-flipped to FRD by readRaw)
//        → (raw - accelOffset) * accelScale       sensor-frame ellipsoid → sphere
//        → R ·                                    mounting tilt (level surface)
//        → - accelBias                            tiny residual so rest = (0,0,+1 g)
// =============================================================================
struct MPU6050
{
    // Sensor-frame accel calibration (in the frame readRaw() outputs).
    // Fill from the 6-face routine; defaults = previous scale constants.
    float accelScale[3] = {0.997124f, 1.000907f, 0.956723f};
    float accelOffset[3] = {0.0f, 0.0f, 0.0f}; // LSB

    // ---- Public sample type ----
    struct Data
    {
        int16_t accelX = 0, accelY = 0, accelZ = 0;
        int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
        float temperatureC = 0.0f;
        unsigned long timestampMs = 0;

        static constexpr float ACCEL_SENS_4G = 8192.0f;
        static constexpr float GYRO_SENS_2000DPS = 16.4f;
        static constexpr float DEG2RAD = PI / 180.0f;

        float ax_g() const { return accelX / ACCEL_SENS_4G; }
        float ay_g() const { return accelY / ACCEL_SENS_4G; }
        float az_g() const { return accelZ / ACCEL_SENS_4G; }

        float gx_dps() const { return gyroX / GYRO_SENS_2000DPS; }
        float gy_dps() const { return gyroY / GYRO_SENS_2000DPS; }
        float gz_dps() const { return gyroZ / GYRO_SENS_2000DPS; }

        float gx_rad() const { return gx_dps() * DEG2RAD; }
        float gy_rad() const { return gy_dps() * DEG2RAD; }
        float gz_rad() const { return gz_dps() * DEG2RAD; }

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

    // ---- Public API ----
    bool begin(TwoWire &bus = Wire, uint8_t address = 0x68)
    {
        wire = &bus;
        addr = address;

        uint8_t who = 0;
        if (!readBytes(REG_WHOAMI, &who, 1))
            return false;
        if (!(who == 0x70 || who == 0x69 || who == 0x68))
            return false;

        if (!writeByte(REG_PWR, 0x00))
            return false; // wake
        if (!writeByte(REG_CONFIG, 0x03))
            return false; // DLPF 42 Hz
        if (!writeByte(REG_SMPLRT_DIV, 0))
            return false; // 1 kHz output (DLPF on)
        if (!writeByte(REG_GYRO_CFG, 0x18))
            return false; // ±2000 dps
        if (!writeByte(REG_ACCEL_CFG, 0x08))
            return false; // ±4 g

        return true;
    }

    // Read sensor -> updates internal state. Returns true on success.
    bool update()
    {
        if (!readRaw(sample))
            return false;
        applyCalibration(sample);
        return true;
    }

    // Access latest sample (read-only)
    const Data &data() const { return sample; }

    // get a copy of the latest sample
    Data get() const { return sample; }

    // Read raw sensor data into d (without applying calibration). Returns true on success.
    bool readRawSample(Data &d) { return readRaw(d); }

    // ── Full calibration: level rotation R + accel bias + gyro bias ──────────
    // Drone MUST rest on a level surface (this defines "level" for the FC).
    void calibrate(int N = 500)
    {
        long sumAx = 0, sumAy = 0, sumAz = 0;
        long sumGx = 0, sumGy = 0, sumGz = 0;
        Data s;

        // Warm-up: let the sensor settle
        for (int i = 0; i < 50; ++i)
        {
            readRaw(s);
            delay(5);
        }

        for (int i = 0; i < N; ++i)
        {
            while (!readRaw(s))
                delay(1);
            sumAx += s.accelX;
            sumAy += s.accelY;
            sumAz += s.accelZ;
            sumGx += s.gyroX;
            sumGy += s.gyroY;
            sumGz += s.gyroZ;
            delay(2);
        }

        // Sensor-frame offset+scale FIRST — same order as applyCalibration()
        const float sAx = ((float)sumAx / N - accelOffset[0]) * accelScale[0];
        const float sAy = ((float)sumAy / N - accelOffset[1]) * accelScale[1];
        const float sAz = ((float)sumAz / N - accelOffset[2]) * accelScale[2];

        computeLevelRotation(sAx, sAy, sAz);

        // Rotate the averaged (scaled) gravity vector into the body frame
        float cAx, cAy, cAz;
        rotate(sAx, sAy, sAz, cAx, cAy, cAz);

        // After rotation gravity lies on +Z. Bias so that rest reads (0, 0, +1 g).
        accelBiasX = (int16_t)roundf(cAx);
        accelBiasY = (int16_t)roundf(cAy);
        accelBiasZ = (int16_t)roundf(cAz - Data::ACCEL_SENS_4G);

        setGyroBiasFromRawAverage((float)sumGx / N, (float)sumGy / N, (float)sumGz / N);
    }

    // ── Gyro-only re-zero. Any orientation, must be still. ~1.1 s @ N=500 ────
    void calibrateGyro(int N = 500)
    {
        long sumGx = 0, sumGy = 0, sumGz = 0;
        Data s;
        for (int i = 0; i < 20; ++i)
        {
            readRaw(s);
            delay(2);
        }
        for (int i = 0; i < N; ++i)
        {
            while (!readRaw(s))
                delay(1);
            sumGx += s.gyroX;
            sumGy += s.gyroY;
            sumGz += s.gyroZ;
            delay(2);
        }
        setGyroBiasFromRawAverage((float)sumGx / N, (float)sumGy / N, (float)sumGz / N);
    }

    // ── Sensor-frame accel calibration (from 6-face routine) ─────────────────
    void setAccelSensorCal(const float scale[3], const float offset[3])
    {
        for (int i = 0; i < 3; ++i)
        {
            accelScale[i] = scale[i];
            accelOffset[i] = offset[i];
        }
    }

    // ── Persistence helpers (level rotation + accel bias are temperature-stable)
    void setCalibration(const float rot[9], int16_t abx, int16_t aby, int16_t abz)
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R[r][c] = rot[r * 3 + c];
        accelBiasX = abx;
        accelBiasY = aby;
        accelBiasZ = abz;
    }
    void getRotation(float rot[9]) const
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                rot[r * 3 + c] = R[r][c];
    }

    // Biases (public if you want to save/load them)
    int16_t accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
    int16_t gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

    // Mounting-offset rotation matrix (sensor frame -> body frame)
    // Identity by default; computed by calibrate()
    float R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    String calibrationToString() const
    {
        String s;
        s.reserve(260);
        s += "Accel Biases: X=";
        s += accelBiasX;
        s += ", Y=";
        s += accelBiasY;
        s += ", Z=";
        s += accelBiasZ;
        s += " | Gyro Biases: X=";
        s += gyroBiasX;
        s += ", Y=";
        s += gyroBiasY;
        s += ", Z=";
        s += gyroBiasZ;
        s += " | AccScale=(";
        s += String(accelScale[0], 5);
        s += ",";
        s += String(accelScale[1], 5);
        s += ",";
        s += String(accelScale[2], 5);
        s += ") AccOff=(";
        s += String(accelOffset[0], 1);
        s += ",";
        s += String(accelOffset[1], 1);
        s += ",";
        s += String(accelOffset[2], 1);
        s += ") | R row3=(";
        s += String(R[2][0], 4);
        s += ",";
        s += String(R[2][1], 4);
        s += ",";
        s += String(R[2][2], 4);
        s += ")";
        return s;
    }

private:
    // ---- MPU regs ----
    static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CFG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CFG = 0x1C;
    static constexpr uint8_t REG_PWR = 0x6B;
    static constexpr uint8_t REG_WHOAMI = 0x75;
    static constexpr uint8_t REG_ACCEL = 0x3B;

    TwoWire *wire = nullptr;
    uint8_t addr = 0x68;

    Data sample;

    void rotate(float x, float y, float z, float &ox, float &oy, float &oz) const
    {
        ox = R[0][0] * x + R[0][1] * y + R[0][2] * z;
        oy = R[1][0] * x + R[1][1] * y + R[1][2] * z;
        oz = R[2][0] * x + R[2][1] * y + R[2][2] * z;
    }

    void setGyroBiasFromRawAverage(float avgGx, float avgGy, float avgGz)
    {
        // Gyro bias is a pure offset in the sensor frame; rotate it into the
        // body frame so it matches the rotated readings in applyCalibration().
        float cGx, cGy, cGz;
        rotate(avgGx, avgGy, avgGz, cGx, cGy, cGz);
        gyroBiasX = (int16_t)roundf(cGx);
        gyroBiasY = (int16_t)roundf(cGy);
        gyroBiasZ = (int16_t)roundf(cGz);
    }

    // ── Rodrigues: rotate measured gravity vector onto (0,0,1) ──────────────
    //
    // gS = measured gravity in sensor frame (normalised)
    // zB = target gravity direction in body frame = (0, 0, 1)  [FRD, down]
    // rotation axis  v = gS × zB = (gy, -gx, 0),  cos = gz, sin = |v|
    void computeLevelRotation(float ax, float ay, float az)
    {
        const float mag = sqrtf(ax * ax + ay * ay + az * az);
        const float gx = ax / mag;
        const float gy = ay / mag;
        const float gz = az / mag;

        const float vx = gy;
        const float vy = -gx;
        const float s_mag = sqrtf(vx * vx + vy * vy); // sin(theta)
        const float c_val = gz;                       // cos(theta)

        if (s_mag < 1e-6f)
        {
            R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
            R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
            R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;
            return;
        }

        // Rodrigues' formula:  R = I + sin(θ)[n]× + (1-cos θ)[n]×²   (n = v/|v|, nz = 0)
        const float nvx = vx / s_mag;
        const float nvy = vy / s_mag;
        const float sin_t = s_mag;
        const float k = 1.0f - c_val;

        R[0][0] = 1.0f - k * nvy * nvy;
        R[0][1] = k * nvx * nvy;
        R[0][2] = sin_t * nvy;
        R[1][0] = k * nvx * nvy;
        R[1][1] = 1.0f - k * nvx * nvx;
        R[1][2] = -sin_t * nvx;
        R[2][0] = -sin_t * nvy;
        R[2][1] = sin_t * nvx;
        R[2][2] = 1.0f - k * (nvx * nvx + nvy * nvy);
    }

    bool writeByte(uint8_t reg, uint8_t value)
    {
        wire->beginTransmission(addr);
        wire->write(reg);
        wire->write(value);
        return (wire->endTransmission() == 0);
    }

    bool readBytes(uint8_t startReg, uint8_t *buffer, uint8_t length)
    {
        wire->beginTransmission(addr);
        wire->write(startReg);
        if (wire->endTransmission(false) != 0)
            return false;

        uint8_t got = (uint8_t)wire->requestFrom((uint8_t)addr, (uint8_t)length, (uint8_t)1);
        if (got != length)
            return false;

        for (uint8_t i = 0; i < length; i++)
            buffer[i] = wire->read();
        return true;
    }

    // Raw read: sign flips -> FRD, no calibration applied.  (UNCHANGED — axes verified)
    bool readRaw(Data &d)
    {
        uint8_t raw[14];
        if (!readBytes(REG_ACCEL, raw, sizeof(raw)))
            return false;

        d.accelX = (int16_t)((raw[0] << 8) | raw[1]);
        d.accelY = -(int16_t)((raw[2] << 8) | raw[3]); // solved
        d.accelZ = (int16_t)((raw[4] << 8) | raw[5]);

        int16_t tempRaw = (int16_t)((raw[6] << 8) | raw[7]);
        d.temperatureC = (tempRaw / 340.0f) + 36.53f;

        d.gyroX = -(int16_t)((raw[8] << 8) | raw[9]);
        d.gyroY = (int16_t)((raw[10] << 8) | raw[11]); // solved
        d.gyroZ = -(int16_t)((raw[12] << 8) | raw[13]);

        d.timestampMs = millis();
        return true;
    }

    void applyCalibration(Data &d)
    {
        // Step 0: sensor-frame ellipsoid → sphere (offset, then scale; float)
        const float sx = (d.accelX - accelOffset[0]) * accelScale[0];
        const float sy = (d.accelY - accelOffset[1]) * accelScale[1];
        const float sz = (d.accelZ - accelOffset[2]) * accelScale[2];

        // Step 1: rotate into body frame
        float ax, ay, az, gx, gy, gz;
        rotate(sx, sy, sz, ax, ay, az);
        rotate(d.gyroX, d.gyroY, d.gyroZ, gx, gy, gz);

        // Step 2: subtract bias (single int16 conversion at the end)
        d.accelX = (int16_t)roundf(ax) - accelBiasX;
        d.accelY = (int16_t)roundf(ay) - accelBiasY;
        d.accelZ = (int16_t)roundf(az) - accelBiasZ;

        d.gyroX = (int16_t)roundf(gx) - gyroBiasX;
        d.gyroY = (int16_t)roundf(gy) - gyroBiasY;
        d.gyroZ = (int16_t)roundf(gz) - gyroBiasZ;
    }
};

#endif // IMU_MPU6050_H
