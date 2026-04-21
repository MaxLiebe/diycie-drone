#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H
#include <Arduino.h>
#include <Wire.h>

struct MPU6050
{
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
        if (!writeByte(REG_SMPLRT_DIV, 1))
            return false; // 500 Hz output
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

        // Accumulate N samples
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

        const float avgAx = (float)sumAx / N;
        const float avgAy = (float)sumAy / N;
        const float avgAz = (float)sumAz / N;

        // ── Rodrigues: rotate measured gravity vector onto (0,0,1) ──────────────
        //
        // gS = measured gravity in sensor frame (normalised)
        // zB = target gravity direction in body frame = (0, 0, 1)  [FRD, down]
        //
        // rotation axis  v = gS × zB
        // cross product of (gx,gy,gz) × (0,0,1):
        //   vx =  gy*1 - gz*0 =  gy
        //   vy =  gz*0 - gx*1 = -gx
        //   vz =  gx*0 - gy*0 =  0
        //
        // cos(angle) c = gS · zB = gz  (after normalisation)
        // sin(angle) s_mag = |v|

        float mag = sqrtf(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);
        float gx = avgAx / mag;
        float gy = avgAy / mag;
        float gz = avgAz / mag;

        float vx = gy;
        float vy = -gx;
        float vz = 0.0f;
        float s_mag = sqrtf(vx * vx + vy * vy); // vz is always 0
        float c_val = gz;                       // dot product with (0,0,1)

        if (s_mag < 1e-6f)
        {
            // Already (nearly) aligned — keep identity
            R[0][0] = 1;
            R[0][1] = 0;
            R[0][2] = 0;
            R[1][0] = 0;
            R[1][1] = 1;
            R[1][2] = 0;
            R[2][0] = 0;
            R[2][1] = 0;
            R[2][2] = 1;
        }
        else
        {
            // Rodrigues' formula:  R = I + [v]× + [v]×² · (1-c)/s²
            //
            // [v]× (skew-symmetric):          [v]×²:
            //  [ 0  -vz  vy ]                 [-vz²-vy²   vx·vy   vx·vz]
            //  [ vz   0  -vx]                 [ vx·vy  -vz²-vx²   vy·vz]
            //  [-vy  vx   0 ]                 [ vx·vz   vy·vz  -vy²-vx²]

            float k = (1.0f - c_val) / (s_mag * s_mag);

            R[0][0] = 1.0f + k * (-vz * vz - vy * vy);
            R[0][1] = k * (vx * vy) - vz;
            R[0][2] = k * (vx * vz) + vy;
            R[1][0] = k * (vx * vy) + vz;
            R[1][1] = 1.0f + k * (-vz * vz - vx * vx);
            R[1][2] = k * (vy * vz) - vx;
            R[2][0] = k * (vx * vz) - vy;
            R[2][1] = k * (vy * vz) + vx;
            R[2][2] = 1.0f + k * (-vy * vy - vx * vx);
        }

        // ── Biases: residual offsets measured AFTER the rotation is applied ──────
        // Rotate the averaged raw vector by R, then subtract the expected (0,0,+g)
        float cAx = R[0][0] * avgAx + R[0][1] * avgAy + R[0][2] * avgAz;
        float cAy = R[1][0] * avgAx + R[1][1] * avgAy + R[1][2] * avgAz;
        float cAz = R[2][0] * avgAx + R[2][1] * avgAy + R[2][2] * avgAz;

        accelBiasX = (int16_t)cAx;
        accelBiasY = (int16_t)cAy;
        accelBiasZ = (int16_t)(cAz - Data::ACCEL_SENS_4G);

        // Gyro bias is a pure offset (direction-invariant), rotate it too
        // so it lives in the same body frame as the corrected readings
        float avgGx = (float)sumGx / N;
        float avgGy = (float)sumGy / N;
        float avgGz = (float)sumGz / N;

        float cGx = R[0][0] * avgGx + R[0][1] * avgGy + R[0][2] * avgGz;
        float cGy = R[1][0] * avgGx + R[1][1] * avgGy + R[1][2] * avgGz;
        float cGz = R[2][0] * avgGx + R[2][1] * avgGy + R[2][2] * avgGz;

        gyroBiasX = (int16_t)cGx;
        gyroBiasY = (int16_t)cGy;
        gyroBiasZ = (int16_t)cGz;
    }

    // Biases (public if you want to save/load them)
    int16_t accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
    int16_t gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

    // Mounting-offset rotation matrix (sensor frame -> body frame)
    // Identity by default; computed by calibrate()
    float R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

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

    // Raw read: sign flips -> FRD, no calibration applied.
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
        // Step 1 — rotate from sensor frame into body frame
        float ax = R[0][0] * d.accelX + R[0][1] * d.accelY + R[0][2] * d.accelZ;
        float ay = R[1][0] * d.accelX + R[1][1] * d.accelY + R[1][2] * d.accelZ;
        float az = R[2][0] * d.accelX + R[2][1] * d.accelY + R[2][2] * d.accelZ;

        float gx = R[0][0] * d.gyroX + R[0][1] * d.gyroY + R[0][2] * d.gyroZ;
        float gy = R[1][0] * d.gyroX + R[1][1] * d.gyroY + R[1][2] * d.gyroZ;
        float gz = R[2][0] * d.gyroX + R[2][1] * d.gyroY + R[2][2] * d.gyroZ;

        // Step 2 — subtract residual bias (already in body frame)
        d.accelX = (int16_t)(ax)-accelBiasX;
        d.accelY = (int16_t)(ay)-accelBiasY;
        d.accelZ = (int16_t)(az)-accelBiasZ;

        d.gyroX = (int16_t)(gx)-gyroBiasX;
        d.gyroY = (int16_t)(gy)-gyroBiasY;
        d.gyroZ = (int16_t)(gz)-gyroBiasZ;
    }
};

#endif // IMU_MPU6050_H