#include <cmath>
#include <cstdint>
#include <cstring>

#include <algorithm>

#include "math.h"

#define MAT(x, y) e[((x) * 4) + (y)]


namespace math {

vec3f_t vec3f_t::normalize(const vec3f_t &v) {
  const float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
  return vec3f_t{v.x / len,
                 v.y / len,
                 v.z / len};
}

void matrix_t::identity() {
  memset(e, 0, sizeof(e));
  e[0x0] = e[0x5] = e[0xa] = e[0xf] = 1.f;
}

// OpenGL perspective projection matrix
void matrix_t::frustum(
    const float l,
    const float r,
    const float b,
    const float t,
    const float n,
    const float f)
{
  MAT(0, 0) = (2 * n) / (r - l);
  MAT(0, 1) = 0;
  MAT(0, 2) = 0;
  MAT(0, 3) = 0;

  MAT(1, 0) = 0;
  MAT(1, 1) = (2 * n) / (t - b);
  MAT(1, 2) = 0;
  MAT(1, 3) = 0;

  MAT(2, 0) = (r + l) / (r - l);
  MAT(2, 1) = (t + b) / (t - b);
  MAT(2, 2) =-(f + n) / (f - n);
  MAT(2, 3) =-1;

  MAT(3, 0) = 0;
  MAT(3, 1) = 0;
  MAT(3, 2) =-(2 * f * n) / (f - n);
  MAT(3, 3) = 0;
}

/* generate a rotation matrix */
void matrix_t::rotate(
    const float a,
    const float b,
    const float c)
{
  const float sa = sinf(a), ca = cosf(a);
  const float sb = sinf(b), cb = cosf(b);
  const float sc = sinf(c), cc = cosf(c);

  MAT(0, 0) = cc * cb;
  MAT(0, 1) = ca * sc * cb + sa * sb;
  MAT(0, 2) = sa * sc * cb - ca * sb;
  MAT(0, 3) = 0.f;

  MAT(1, 0) =-sc;
  MAT(1, 1) = ca * cc;
  MAT(1, 2) = sa * cc;
  MAT(1, 3) = 0.f;

  MAT(2, 0) = cc * sb;
  MAT(2, 1) = ca * sc * sb - sa * cb;
  MAT(2, 2) = sa * sc * sb + ca * cb;
  MAT(2, 3) = 0.f;

  MAT(3, 0) = 0.f;
  MAT(3, 1) = 0.f;
  MAT(3, 2) = 0.f;
  MAT(3, 3) = 1.f;
}

void matrix_t::translate(const vec3f_t &p) {
  MAT(3, 0) = p.x;
  MAT(3, 1) = p.y;
  MAT(3, 2) = p.z;
  MAT(3, 3) = 1.f;
}

/* transform an array of vectors by a matrix */
void matrix_t::transform(const uint32_t num_verts,
                         const vec4f_t *in,
                         vec4f_t *out) {
  for (uint32_t q = 0; q < num_verts; ++q) {
    // todo: unroll and use SIMD instructions
    const vec4f_t &s = in[q];
    out[q] = vec4f_t{
      s.x * MAT(0, 0) + s.y * MAT(1, 0) + s.z * MAT(2, 0) + s.w * MAT(3, 0),
      s.x * MAT(0, 1) + s.y * MAT(1, 1) + s.z * MAT(2, 1) + s.w * MAT(3, 1),
      s.x * MAT(0, 2) + s.y * MAT(1, 2) + s.z * MAT(2, 2) + s.w * MAT(3, 2),
      s.x * MAT(0, 3) + s.y * MAT(1, 3) + s.z * MAT(2, 3) + s.w * MAT(3, 3),
    };
  } // for
}

/* transform an array of vectors by a matrix */
void matrix_t::transform(const uint32_t num_verts,
                         const vec3f_t *in,
                         vec3f_t *out) {
  for (uint32_t q = 0; q < num_verts; ++q) {
    // todo: unroll and use SIMD instructions
    const vec3f_t &s = in[q];
    out[q] = vec3f_t{
      s.x * MAT(0, 0) + s.y * MAT(1, 0) + s.z * MAT(2, 0),
      s.x * MAT(0, 1) + s.y * MAT(1, 1) + s.z * MAT(2, 1),
      s.x * MAT(0, 2) + s.y * MAT(1, 2) + s.z * MAT(2, 2),
    };
  } // for
}

bool matrix_t::invert(matrix_t &out)
{
    float inv[16];

    inv[0] = e[5] * e[10] * e[15] - e[5]  * e[11] * e[14] - e[9]  * e[6] * e[15] +
             e[9] * e[7]  * e[14] + e[13] * e[6]  * e[11] - e[13] * e[7] * e[10];

    inv[4] = -e[4] * e[10] * e[15] + e[4]  * e[11] * e[14] + e[8]  * e[6] * e[15] -
              e[8] * e[7]  * e[14] - e[12] * e[6]  * e[11] + e[12] * e[7] * e[10];

    inv[8] = e[4] * e[9] * e[15] - e[4] * e[11] * e[13] - e[8] * e[5] * e[15] +
             e[8] * e[7] * e[13] + e[12] * e[5] * e[11] - e[12] * e[7] * e[9];

    inv[12] = -e[4] * e[9] * e[14] + e[4]  * e[10] * e[13] + e[8]  * e[5] * e[14] -
               e[8] * e[6] * e[13] - e[12] * e[5]  * e[10] + e[12] * e[6] * e[9];

    inv[1] = -e[1] * e[10] * e[15] + e[1]  * e[11] * e[14] + e[9]  * e[2] * e[15] -
              e[9] * e[3]  * e[14] - e[13] * e[2]  * e[11] + e[13] * e[3] * e[10];

    inv[5] =  e[0] * e[10] * e[15] - e[0]  * e[11] * e[14] - e[8]  * e[2] * e[15] +
              e[8] * e[3]  * e[14] + e[12] * e[2]  * e[11] - e[12] * e[3] * e[10];

    inv[9] = -e[0] * e[9] * e[15] + e[0]  * e[11] * e[13] + e[8]  * e[1] * e[15] -
              e[8] * e[3] * e[13] - e[12] * e[1]  * e[11] + e[12] * e[3] * e[9];

    inv[13] = e[0] * e[9] * e[14] - e[0]  * e[10] * e[13] - e[8]  * e[1] * e[14] +
              e[8] * e[2] * e[13] + e[12] * e[1]  * e[10] - e[12] * e[2] * e[9];

    inv[2] = e[1] * e[6] * e[15] - e[1] * e[7] * e[14] - e[5] * e[2] * e[15] +
             e[5] * e[3] * e[14] + e[13] * e[2] * e[7] - e[13] * e[3] * e[6];

    inv[6] = -e[0] * e[6] * e[15] + e[0] * e[7] * e[14] + e[4] * e[2] * e[15] -
             e[4] * e[3] * e[14] - e[12] * e[2] * e[7] + e[12] * e[3] * e[6];

    inv[10] = e[0] * e[5] * e[15] - e[0] * e[7] * e[13] - e[4] * e[1] * e[15] +
              e[4] * e[3] * e[13] + e[12] * e[1] * e[7] - e[12] * e[3] * e[5];

    inv[14] = -e[0] * e[5] * e[14] + e[0] * e[6] * e[13] + e[4] * e[1] * e[14] -
              e[4] * e[2] * e[13] - e[12] * e[1] * e[6] + e[12] * e[2] * e[5];

    inv[3] = -e[1] * e[6] * e[11] + e[1] * e[7] * e[10] + e[5] * e[2] * e[11] -
             e[5] * e[3] * e[10] - e[9] * e[2] * e[7] + e[9] * e[3] * e[6];

    inv[7] = e[0] * e[6] * e[11] - e[0] * e[7] * e[10] - e[4] * e[2] * e[11] +
             e[4] * e[3] * e[10] + e[8] * e[2] * e[7] - e[8] * e[3] * e[6];

    inv[11] = -e[0] * e[5] * e[11] + e[0] * e[7] * e[9] + e[4] * e[1] * e[11] -
              e[4] * e[3] * e[9] - e[8] * e[1] * e[7] + e[8] * e[3] * e[5];

    inv[15] = e[0] * e[5] * e[10] - e[0] * e[6] * e[9] - e[4] * e[1] * e[10] +
              e[4] * e[2] * e[9] + e[8] * e[1] * e[6] - e[8] * e[2] * e[5];

    float det = e[0] * inv[0] + e[1] * inv[4] + e[2] * inv[8] + e[3] * inv[12];
    if (det == 0)
      return false;
    det = 1.0f / det;

    for (int i = 0; i < 16; i++)
      out.e[i] = inv[i] * det;

    return true;
}

void matrix_t::transpose() {
  std::swap(e[0x1], e[0x4]);
  std::swap(e[0x2], e[0x8]);
  std::swap(e[0x3], e[0xc]);
  std::swap(e[0x6], e[0x9]);
  std::swap(e[0x7], e[0xd]);
  std::swap(e[0xb], e[0xe]);
}


// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

vec3f_t vec3(const vec2f_t &v, float z) {
  return vec3f_t{v.x, v.y, z};
}

vec3f_t vec3(const vec4f_t &v) {
  const float iw = 1.f / v.w;
  return vec3f_t {v.x * iw, v.y * iw, v.z * iw};
}

vec4f_t vec4(const vec2f_t &v, float z, float w) {
  return vec4f_t{v.x, v.y, z, w};
}

vec4f_t vec4(const vec3f_t &v, float w) {
  return vec4f_t{v.x, v.y, v.z, w};
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector component construction

vec2f_t vec2(const float x,
             const float y) {
  return vec2f_t {x, y};
}

vec3f_t vec3(const float x,
             const float y,
             const float z) {
  return vec3f_t {x, y, z};
}

vec4f_t vec4(const float x,
             const float y,
             const float z,
             const float w) {
  return vec4f_t {x, y, z, w};
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector addition

vec2f_t operator+(const vec2f_t &a,
                  const vec2f_t &b) {
  return vec2f_t{
    a.x + b.x,
    a.y + b.y
  };
}

vec3f_t operator+(const vec3f_t &a,
                  const vec3f_t &b) {
  return vec3f_t{
    a.x + b.x,
    a.y + b.y,
    a.z + b.z
  };
}

vec4f_t operator+(const vec4f_t &a,
                  const vec4f_t &b) {
  return vec4f_t{
    a.x + b.x,
    a.y + b.y,
    a.z + b.z,
    a.w + b.w
  };
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector subtraction

vec2f_t operator-(const vec2f_t &a,
                  const vec2f_t &b) {
  return vec2f_t{
    a.x - b.x,
    a.y - b.y
  };
}

vec3f_t operator-(const vec3f_t &a,
                  const vec3f_t &b) {
  return vec3f_t{
    a.x - b.x,
    a.y - b.y,
    a.z - b.z
  };
}

vec4f_t operator-(const vec4f_t &a,
                  const vec4f_t &b) {
  return vec4f_t{
    a.x - b.x,
    a.y - b.y,
    a.z - b.z,
    a.w - b.w
  };
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector in place addition

void operator+=(vec2f_t &a,
                const vec2f_t &b) {
  a.x += b.x;
  a.y += b.y;
}

void operator+=(vec3f_t &a,
                const vec3f_t &b) {
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
}

void operator+=(vec4f_t &a,
                const vec4f_t &b) {
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  a.w += b.w;
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector subtraction in place

void operator-=(vec2f_t &a,
                const vec2f_t &b) {
  a.x -= b.x;
  a.y -= b.y;
}

void operator-=(vec3f_t &a,
                const vec3f_t &b) {
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;
}

void operator-=(vec4f_t &a,
                const vec4f_t &b) {
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;
  a.w -= b.w;
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector scale

vec2f_t operator*(const vec2f_t &a,
                  const float s) {
  return vec2f_t{a.x * s, a.y * s};
}

vec3f_t operator*(const vec3f_t &a,
                  const float s) {
  return vec3f_t{a.x * s, a.y * s, a.z * s};
}

vec4f_t operator*(const vec4f_t &a,
                  const float s) {
  return vec4f_t{a.x * s, a.y * s, a.z * s, a.w * s};
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector dot product

float operator*(const vec2f_t &a,
                const vec2f_t &b) {
  return a.x * b.x + a.y * b.y;
}

float operator*(const vec3f_t &a,
                const vec3f_t &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

float operator*(const vec4f_t &a,
                const vec4f_t &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

vec3f_t operator/(const vec3f_t &a,
                  const float s) {
  return vec3f_t{a.x / s, a.y / s, a.z / s};
}

} // namespace math
