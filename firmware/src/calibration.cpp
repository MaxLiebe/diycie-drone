// /***********************************************************************
//  *  MPU6050 6-Point Accelerometer Calibration Sketch  —  rev-1
//  *  LOLIN C3 Pico / ESP32-C3
//  *
//  *  Stage-1 hardware calibration (IMU frame).
//  *  Outputs ONLY the three ACCEL_SCALE constants for MPU6050-2.h.
//  *  Stage-2 drone-frame alignment (Rodrigues + per-boot bias) stays
//  *  inside MPU6050::calibrate(500) and is NOT touched by this sketch.
//  *
//  *  ── What the original sketch got wrong, and how this fixes it ────────
//  *  1. include name was "MPU6050.h"             → now "MPU6050-2.h"
//  *  2. used imu.update() which already applied  → now uses imu.readRawSample()
//  *     ACCEL_SCALE — produced wrong scale          (truly raw data)
//  *  3. printed application snippet referenced    → snippet removed; the
//  *     undefined GRAV and double-applied scale      driver does it correctly
//  *                                                  with just the SCALE consts
//  *  4. only six of eighteen numbers were used    → 6×6 ellipsoid solve
//  *     (three independent two-point cals)           uses all 18 axis-readings
//  *  5. no settling delay after waitForEnter      → added 1500 ms delay
//  *  6. no residual / sanity check                → added both
//  *  7. loop() empty — board appeared hung        → added 5 s heartbeat
//  *
//  *  ── Driver requirement (one-line addition) ──────────────────────────
//  *  In MPU6050-2.h, public section, add:
//  *      bool readRawSample(Data &d) { return readRaw(d); }
//  *  (already done if you took the patched driver alongside this sketch)
//  *
//  *  ── Procedure ────────────────────────────────────────────────────────
//  *  1. Flash THIS sketch (not the flight controller).
//  *  2. Open Serial Monitor at 115200 baud.
//  *  3. Place the IMU (in the precision mount block) in each of six
//  *     orientations as prompted, send any char + ENTER each time.
//  *  4. At the end, copy the three ACCEL_SCALE_ lines into MPU6050-2.h
//  *     replacing the existing constants. That's all — bias/rotation
//  *     are still handled per boot by calibrate(500).
//  *
//  *  ── Orientation guide (FRD body, +X fwd, +Y right, +Z down) ─────────
//  *  Note: the driver sign-flips raw accelY, so the Y readings appear with
//  *  inverted sign vs. the register. The ellipsoid math is sign-independent
//  *  (uses mx², my², mz²) — labels below show what you'll actually see.
//  *
//  *    Face 1  +X up   nose at ceiling          accelX ≈ +8192
//  *    Face 2  -X up   nose at floor            accelX ≈ -8192
//  *    Face 3  +Y up   right arm at ceiling     accelY ≈ -8192  (driver flip)
//  *    Face 4  -Y up   left arm at ceiling      accelY ≈ +8192  (driver flip)
//  *    Face 5  +Z up   flat, right-side up      accelZ ≈ +8192
//  *    Face 6  -Z up   flat, upside down        accelZ ≈ -8192
//  ***********************************************************************/

// #include <Arduino.h>
// #include <Wire.h>
// #include "MPU6050.h"

// // ── Pins ──────────────────────────────────────────────────────────────
// static constexpr uint8_t I2C_SDA_PIN = 10;
// static constexpr uint8_t I2C_SCL_PIN = 20;

// // ── Cal config ────────────────────────────────────────────────────────
// static constexpr int N_SAMPLES = 1000; // ~2 s of data per face
// static constexpr float SENS = 8192.0f; // LSB/g for ±4 g range
// static constexpr float G_REF = 9.81f;  // m/s², display only

// MPU6050 imu;

// // ─────────────────────────────────────────────────────────────────────
// //  6×6 Gaussian elimination with partial pivoting.
// //  Solves  A·x = b  in-place. Returns false on singular A.
// // ─────────────────────────────────────────────────────────────────────
// static bool solve6(float A[6][6], float b[6], float x[6])
// {
//     float M[6][7];
//     for (int i = 0; i < 6; ++i)
//     {
//         for (int j = 0; j < 6; ++j)
//             M[i][j] = A[i][j];
//         M[i][6] = b[i];
//     }
//     for (int c = 0; c < 6; ++c)
//     {
//         int piv = c;
//         for (int r = c + 1; r < 6; ++r)
//             if (fabsf(M[r][c]) > fabsf(M[piv][c]))
//                 piv = r;
//         if (fabsf(M[piv][c]) < 1e-8f)
//             return false;
//         if (piv != c)
//             for (int j = 0; j <= 6; ++j)
//             {
//                 float t = M[c][j];
//                 M[c][j] = M[piv][j];
//                 M[piv][j] = t;
//             }
//         const float inv = 1.0f / M[c][c];
//         for (int r = 0; r < 6; ++r)
//         {
//             if (r == c)
//                 continue;
//             const float f = M[r][c] * inv;
//             for (int j = c; j <= 6; ++j)
//                 M[r][j] -= f * M[c][j];
//         }
//     }
//     for (int i = 0; i < 6; ++i)
//         x[i] = M[i][6] / M[i][i];
//     return true;
// }

// // ─────────────────────────────────────────────────────────────────────
// //  Wait for any keypress, then settle for 1.5 s
// // ─────────────────────────────────────────────────────────────────────
// static void waitKey(const char *msg)
// {
//     Serial.println();
//     Serial.print("  >>> ");
//     Serial.println(msg);
//     Serial.println("      Hold steady, then send any char + ENTER.");
//     Serial.flush();
//     while (Serial.available())
//         Serial.read();
//     while (!Serial.available())
//         delay(10);
//     while (Serial.available())
//         Serial.read();
//     delay(1500); // settle out user's hand vibration
// }

// // ─────────────────────────────────────────────────────────────────────
// //  Average N raw samples (sign-flip only — no scale, rotation or bias)
// // ─────────────────────────────────────────────────────────────────────
// static void takeFace(float out[3])
// {
//     Serial.println("      Collecting...");
//     MPU6050::Data d;

//     // discard 100 samples for sensor-side settling
//     for (int i = 0; i < 100; ++i)
//     {
//         imu.readRawSample(d);
//         delay(2);
//     }

//     double sx = 0, sy = 0, sz = 0;
//     int n = 0;
//     while (n < N_SAMPLES)
//     {
//         if (imu.readRawSample(d))
//         {
//             sx += d.accelX;
//             sy += d.accelY;
//             sz += d.accelZ;
//             ++n;
//         }
//         delay(2);
//     }
//     out[0] = (float)(sx / N_SAMPLES);
//     out[1] = (float)(sy / N_SAMPLES);
//     out[2] = (float)(sz / N_SAMPLES);
//     Serial.printf("      Mean raw: X=%8.1f  Y=%8.1f  Z=%8.1f LSB\n",
//                   out[0], out[1], out[2]);
// }

// // ─────────────────────────────────────────────────────────────────────
// void setup()
// {
//     Serial.begin(115200);
//     delay(2000);
//     Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
//     Wire.setClock(400000);

//     Serial.println();
//     Serial.println();
//     Serial.println("================================================");
//     Serial.println("  MPU6050 6-Point Ellipsoid Calibration  rev-1");
//     Serial.println("================================================");

//     if (!imu.begin(Wire))
//     {
//         Serial.println("ERROR: MPU6050 not found - check I2C wiring.");
//         for (;;)
//             delay(500);
//     }
//     // NB: we deliberately do NOT call imu.calibrate() here.
//     //     Stage-1 cal (this sketch) wants raw IMU-frame data, not
//     //     drone-frame gravity-aligned data.

//     Serial.println("  Sensor OK. Use your precision mount block for each face.");
//     Serial.println("  ~2 s of data is collected automatically after each keypress.");

//     // ── 1. Collect six face means in raw LSB ─────────────────────────
//     float F[6][3];
//     waitKey("FACE 1/6  Nose UP        (+X to ceiling)");
//     takeFace(F[0]);
//     waitKey("FACE 2/6  Nose DOWN      (+X to floor)");
//     takeFace(F[1]);
//     waitKey("FACE 3/6  Right arm UP   (+Y to ceiling)");
//     takeFace(F[2]);
//     waitKey("FACE 4/6  Left arm UP    (-Y to ceiling)");
//     takeFace(F[3]);
//     waitKey("FACE 5/6  Flat, RIGHT-SIDE UP   (normal flight pos)");
//     takeFace(F[4]);
//     waitKey("FACE 6/6  Flat, UPSIDE DOWN");
//     takeFace(F[5]);

//     // ── 2. Ellipsoid fit (closed-form 6×6 linear solve) ──────────────
//     //
//     // Diagonal-ellipsoid model.  For each face i with mean (mx,my,mz):
//     //
//     //   sx²(mx-bx)² + sy²(my-by)² + sz²(mz-bz)² = SENS²
//     //
//     // Expanded and rearranged (small K = sx²bx² + sy²by² + sz²bz²
//     // is < 0.5 % for any reasonable MPU6050 — absorbed into RHS):
//     //
//     //   u·mx² + v·my² + w·mz² + p·mx + q·my + r·mz = SENS²
//     //
//     // with  u = sx²,  v = sy²,  w = sz²,
//     //       p = -2u·bx,  q = -2v·by,  r = -2w·bz.
//     //
//     // Six face means → 6 linear equations in (u,v,w,p,q,r).
//     // Uses all 18 axis-readings (not just the 6 dominant ones).

//     const float S2 = SENS * SENS; // 8192² = 67 108 864
//     float H[6][6], rhs[6], th[6];
//     for (int i = 0; i < 6; ++i)
//     {
//         const float mx = F[i][0], my = F[i][1], mz = F[i][2];
//         H[i][0] = mx * mx;
//         H[i][1] = my * my;
//         H[i][2] = mz * mz;
//         H[i][3] = mx;
//         H[i][4] = my;
//         H[i][5] = mz;
//         rhs[i] = S2;
//     }

//     if (!solve6(H, rhs, th))
//     {
//         Serial.println("\nERROR: singular matrix - duplicate or swapped faces?");
//         for (;;)
//             delay(500);
//     }

//     const float u = th[0], v = th[1], w = th[2];
//     const float p = th[3], q = th[4], r = th[5];

//     if (u <= 0.0f || v <= 0.0f || w <= 0.0f)
//     {
//         Serial.println("\nERROR: negative scale^2 - face order wrong.");
//         for (;;)
//             delay(500);
//     }

//     const float sx = sqrtf(u);
//     const float sy = sqrtf(v);
//     const float sz = sqrtf(w);
//     const float bx = -p / (2.0f * u); // raw-LSB bias (not needed for driver)
//     const float by = -q / (2.0f * v);
//     const float bz = -r / (2.0f * w);

//     // ── 3. Residuals: every face must lie on the unit sphere ─────────
//     const char *lbl[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
//     float maxErr = 0.0f;

//     Serial.println();
//     Serial.println("================================================");
//     Serial.println("  RESIDUALS  (target |a| = 1.00000 g per face)");
//     Serial.println("================================================");
//     for (int i = 0; i < 6; ++i)
//     {
//         const float ax = (F[i][0] - bx) * sx / SENS;
//         const float ay = (F[i][1] - by) * sy / SENS;
//         const float az = (F[i][2] - bz) * sz / SENS;
//         const float mag = sqrtf(ax * ax + ay * ay + az * az);
//         const float err = fabsf(mag - 1.0f);
//         if (err > maxErr)
//             maxErr = err;
//         Serial.printf("  Face %s: |a| = %.5f g   err = %.5f g%s\n",
//                       lbl[i], mag, err, err > 0.015f ? "  <-- HIGH" : "");
//     }
//     const char *grade =
//         maxErr > 0.015f ? "POOR (redo cal)" : maxErr > 0.005f ? "ACCEPTABLE"
//                                                               : "EXCELLENT";
//     Serial.printf("  Overall: %s   (max err %.5f g)\n", grade, maxErr);

//     // ── 4. Sanity check ─────────────────────────────────────────────
//     Serial.println();
//     Serial.println("================================================");
//     Serial.println("  SANITY CHECK  (scale within 8% of 1.0,");
//     Serial.println("                 bias within 800 LSB)");
//     Serial.println("================================================");
//     bool warn = false;
//     auto chk = [&](const char *nm, float sc, float bb)
//     {
//         const bool sOk = (fabsf(sc - 1.0f) < 0.08f);
//         const bool bOk = (fabsf(bb) < 800.0f);
//         if (!sOk || !bOk)
//             warn = true;
//         Serial.printf("  %s  scale=%.6f%s  bias=%+8.1f LSB%s\n",
//                       nm, sc, sOk ? "" : " [!]",
//                       bb, bOk ? "" : " [!]");
//     };
//     chk("X", sx, bx);
//     chk("Y", sy, by);
//     chk("Z", sz, bz);
//     if (warn)
//         Serial.println("  WARNING: out-of-range result. Verify positions and redo.");

//     // ── 5. Output: ONLY the three constants for MPU6050-2.h ─────────
//     Serial.println();
//     Serial.println("================================================");
//     Serial.println("  PASTE INTO MPU6050-2.h   (replace the three");
//     Serial.println("  ACCEL_SCALE_ lines near the top of struct MPU6050)");
//     Serial.println("================================================");
//     Serial.printf("    const float ACCEL_SCALE_X = %.6ff;\n", sx);
//     Serial.printf("    const float ACCEL_SCALE_Y = %.6ff;\n", sy);
//     Serial.printf("    const float ACCEL_SCALE_Z = %.6ff;\n", sz);
//     Serial.println();
//     Serial.println("  (No bias paste needed - calibrate(500) on boot");
//     Serial.println("   recomputes accel & gyro bias and Rodrigues R");
//     Serial.println("   in drone-frame, using these new scale factors.)");
//     Serial.println();
//     Serial.println("  Hardware bias (informational; m/s^2):");
//     Serial.printf("     X=%+.4f  Y=%+.4f  Z=%+.4f\n",
//                   bx / SENS * G_REF, by / SENS * G_REF, bz / SENS * G_REF);
//     Serial.println();
//     Serial.println("================================================");
//     Serial.println("  DONE. Edit MPU6050-2.h and re-flash the FC.");
//     Serial.println("================================================");
// }

// void loop()
// {
//     // Heartbeat so the user knows the board didn't crash.
//     static uint32_t last = 0;
//     if (millis() - last >= 5000)
//     {
//         last = millis();
//         Serial.println("[Calibration finished - safe to disconnect]");
//     }
// }
