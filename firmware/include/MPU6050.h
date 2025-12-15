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
        // Average raw readings while still
        long sumAx = 0, sumAy = 0, sumAz = 0;
        long sumGx = 0, sumGy = 0, sumGz = 0;

        Data s;

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

            delay(5);
        }

        const float avgAx = (float)sumAx / N;
        const float avgAy = (float)sumAy / N;
        const float avgAz = (float)sumAz / N;

        const float avgGx = (float)sumGx / N;
        const float avgGy = (float)sumGy / N;
        const float avgGz = (float)sumGz / N;

        // FRD at rest should be: ax≈0, ay≈0, az≈ +1g (down)
        accelBiasX = (int16_t)avgAx;
        accelBiasY = (int16_t)avgAy;
        accelBiasZ = (int16_t)(avgAz - Data::ACCEL_SENS_4G);

        gyroBiasX = (int16_t)avgGx;
        gyroBiasY = (int16_t)avgGy;
        gyroBiasZ = (int16_t)avgGz;
    }

    // Biases (public if you want to save/load them)
    int16_t accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
    int16_t gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

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

        uint8_t got = wire->requestFrom(addr, length, true);
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

        d.accelX = -(int16_t)((raw[0] << 8) | raw[1]);
        d.accelY = (int16_t)((raw[2] << 8) | raw[3]);
        d.accelZ = (int16_t)((raw[4] << 8) | raw[5]);

        int16_t tempRaw = (int16_t)((raw[6] << 8) | raw[7]);
        d.temperatureC = (tempRaw / 340.0f) + 36.53f;

        d.gyroX = (int16_t)((raw[8] << 8) | raw[9]);
        d.gyroY = -(int16_t)((raw[10] << 8) | raw[11]);
        d.gyroZ = (int16_t)((raw[12] << 8) | raw[13]);

        d.timestampMs = millis();
        return true;
    }

    void applyCalibration(Data &d)
    {
        d.accelX -= accelBiasX;
        d.accelY -= accelBiasY;
        d.accelZ -= accelBiasZ;
        d.gyroX -= gyroBiasX;
        d.gyroY -= gyroBiasY;
        d.gyroZ -= gyroBiasZ;
    }
};

#endif // IMU_MPU6050_H