// #ifndef VEC_MATH_H
// #define VEC_MATH_H

// #include <Arduino.h>

// // --------------------------------------------------------------------
// // Simple 3D vector of floats
// // --------------------------------------------------------------------
// struct Vec3f
// {
//     float x, y, z;

//     Vec3f() : x(0), y(0), z(0) {}
//     Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

//     // basic ops
//     Vec3f operator+(const Vec3f &o) const { return Vec3f(x + o.x, y + o.y, z + o.z); }
//     Vec3f operator-(const Vec3f &o) const { return Vec3f(x - o.x, y - o.y, z - o.z); }
//     Vec3f operator*(float s) const { return Vec3f(x * s, y * s, z * s); }
//     Vec3f operator/(float s) const { return Vec3f(x / s, y / s, z / s); }

//     Vec3f &operator+=(const Vec3f &o)
//     {
//         x += o.x;
//         y += o.y;
//         z += o.z;
//         return *this;
//     }
//     Vec3f &operator-=(const Vec3f &o)
//     {
//         x -= o.x;
//         y -= o.y;
//         z -= o.z;
//         return *this;
//     }
//     Vec3f &operator*=(float s)
//     {
//         x *= s;
//         y *= s;
//         z *= s;
//         return *this;
//     }
//     Vec3f &operator/=(float s)
//     {
//         x /= s;
//         y /= s;
//         z /= s;
//         return *this;
//     }

//     float dot(const Vec3f &o) const { return x * o.x + y * o.y + z * o.z; }

//     float length() const { return sqrtf(x * x + y * y + z * z); }

//     Vec3f normalized() const
//     {
//         float len = length();
//         if (len <= 1e-6f)
//             return Vec3f(0, 0, 0);
//         return *this / len;
//     }

//     Vec3f clamped(float minVal, float maxVal) const
//     {
//         Vec3f r;
//         r.x = constrain(x, minVal, maxVal);
//         r.y = constrain(y, minVal, maxVal);
//         r.z = constrain(z, minVal, maxVal);
//         return r;
//     }
// };

// // --------------------------------------------------------------------
// // Simple 4D "corner" vector (FL, FR, BR, BL)
// // --------------------------------------------------------------------
// struct Vec4f
// {
//     // Corner layout: FL, FR, BR, BL
//     float fl, fr, br, bl;

//     Vec4f() : fl(0), fr(0), br(0), bl(0) {}
//     Vec4f(float fl_, float fr_, float br_, float bl_) : fl(fl_), fr(fr_), br(br_), bl(bl_) {}

//     Vec4f operator+(const Vec4f &o) const
//     {
//         return Vec4f(fl + o.fl, fr + o.fr, br + o.br, bl + o.bl);
//     }
//     Vec4f operator-(const Vec4f &o) const
//     {
//         return Vec4f(fl - o.fl, fr - o.fr, br - o.br, bl - o.bl);
//     }
//     Vec4f operator*(float s) const
//     {
//         return Vec4f(fl * s, fr * s, br * s, bl * s);
//     }

//     Vec4f &operator+=(const Vec4f &o)
//     {
//         fl += o.fl;
//         fr += o.fr;
//         br += o.br;
//         bl += o.bl;
//         return *this;
//     }
//     Vec4f &operator-=(const Vec4f &o)
//     {
//         fl -= o.fl;
//         fr -= o.fr;
//         br -= o.br;
//         bl -= o.bl;
//         return *this;
//     }
//     Vec4f &operator*=(float s)
//     {
//         fl *= s;
//         fr *= s;
//         br *= s;
//         bl *= s;
//         return *this;
//     }

//     float &operator[](int i)
//     {
//         switch (i)
//         {
//         default:
//         case 0:
//             return fl;
//         case 1:
//             return fr;
//         case 2:
//             return br;
//         case 3:
//             return bl;
//         }
//     }

//     const float &operator[](int i) const
//     {
//         switch (i)
//         {
//         default:
//         case 0:
//             return fl;
//         case 1:
//             return fr;
//         case 2:
//             return br;
//         case 3:
//             return bl;
//         }
//     }

//     Vec4f clamped(float minVal, float maxVal) const
//     {
//         Vec4f r;
//         r.fl = constrain(fl, minVal, maxVal);
//         r.fr = constrain(fr, minVal, maxVal);
//         r.br = constrain(br, minVal, maxVal);
//         r.bl = constrain(bl, minVal, maxVal);
//         return r;
//     }
// };

// // --------------------------------------------------------------------
// // Quaternion (w + x i + y j + z k)
// // --------------------------------------------------------------------
// struct Quatf
// {
//     float w, x, y, z;

//     Quatf() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
//     Quatf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

//     static Quatf identity() { return Quatf(); }

//     float norm() const
//     {
//         return sqrtf(w * w + x * x + y * y + z * z);
//     }

//     void normalize()
//     {
//         float n = norm();
//         if (n <= 1e-9f)
//         {
//             w = 1.0f;
//             x = y = z = 0.0f;
//             return;
//         }
//         float inv = 1.0f / n;
//         w *= inv;
//         x *= inv;
//         y *= inv;
//         z *= inv;
//     }

//     Quatf normalized() const
//     {
//         Quatf q = *this;
//         q.normalize();
//         return q;
//     }

//     Quatf conjugate() const
//     {
//         return Quatf(w, -x, -y, -z);
//     }

//     Quatf operator*(const Quatf &q) const
//     {
//         return Quatf(
//             w * q.w - x * q.x - y * q.y - z * q.z,
//             w * q.x + x * q.w + y * q.z - z * q.y,
//             w * q.y - x * q.z + y * q.w + z * q.x,
//             w * q.z + x * q.y - y * q.x + z * q.w);
//     }

//     Quatf operator+(const Quatf &q) const
//     {
//         return Quatf(w + q.w, x + q.x, y + q.y, z + q.z);
//     }

//     Quatf &operator+=(const Quatf &q)
//     {
//         w += q.w;
//         x += q.x;
//         y += q.y;
//         z += q.z;
//         return *this;
//     }

//     Quatf operator*(float s) const
//     {
//         return Quatf(w * s, x * s, y * s, z * s);
//     }

//     Quatf &operator*=(float s)
//     {
//         w *= s;
//         x *= s;
//         y *= s;
//         z *= s;
//         return *this;
//     }

//     // Rotate a vector: v' = q * (0,v) * q*
//     Vec3f rotate(const Vec3f &v) const
//     {
//         Quatf vq(0.0f, v.x, v.y, v.z);
//         Quatf rq = (*this) * vq * this->conjugate();
//         return Vec3f(rq.x, rq.y, rq.z);
//     }

//     // Convert to roll/pitch/yaw (radians), Z-Y-X convention
//     void toEulerRPY(float &roll, float &pitch, float &yaw) const
//     {
//         float qw = w, qx = x, qy = y, qz = z;

//         // roll (x-axis)
//         float sinr_cosp = 2.0f * (qw * qx + qy * qz);
//         float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
//         roll = atan2f(sinr_cosp, cosr_cosp);

//         // pitch (y-axis)
//         float sinp = 2.0f * (qw * qy - qz * qx);
//         if (fabsf(sinp) >= 1.0f)
//             pitch = copysignf(PI / 2.0f, sinp);
//         else
//             pitch = asinf(sinp);

//         // yaw (z-axis)
//         float siny_cosp = 2.0f * (qw * qz + qx * qy);
//         float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
//         yaw = atan2f(siny_cosp, cosy_cosp);
//     }
// };

// #endif // VEC_MATH_H
