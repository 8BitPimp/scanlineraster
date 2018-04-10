#pragma once

namespace math {

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

struct vec2f_t;
struct vec3f_t;
struct vec4f_t;
struct matrix_t;

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

static const float n3d_pi = 3.14159265359f;
static const float n3d_pi2 = n3d_pi * 2.f;

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

struct util_t {
  static float lerp(const float &a,
             const float &b,
             const float k) {
    return ((1.f - k) * a) + (k * b);
  }
};

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

struct vec2f_t {

  union { float e[2]; struct { float x, y; }; };

  static vec2f_t lerp(const vec2f_t &a, const vec2f_t &b, float i) {
    //(todo) SIMD lerp
    return vec2f_t{
      util_t::lerp(a.x, b.x, i),
      util_t::lerp(a.y, b.y, i)
    };
  }
};

struct vec3f_t {

  union { float e[3]; struct { float x, y, z; }; };

  static vec3f_t normalize(const vec3f_t &v);

  static vec3f_t cross(const vec3f_t &a, const vec3f_t &b) {
    return vec3f_t{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
    };
  }

  static vec3f_t lerp(const vec3f_t &a, const vec3f_t &b, float i) {
    //(todo) SIMD lerp
    return vec3f_t{
      util_t::lerp(a.x, b.x, i),
      util_t::lerp(a.y, b.y, i),
      util_t::lerp(a.z, b.z, i)
    };
  }
};

struct vec4f_t {

  union { float e[4]; struct { float x, y, z, w; }; };

  static vec4f_t lerp(const vec4f_t &a, const vec4f_t &b, float i) {
    //(todo) SIMD lerp
    return vec4f_t{
      util_t::lerp(a.x, b.x, i),
      util_t::lerp(a.y, b.y, i),
      util_t::lerp(a.z, b.z, i),
      util_t::lerp(a.w, b.w, i)
    };
  }
};

struct matrix_t {

  void translate(const vec3f_t &p);

  bool invert(matrix_t &out);

  void rotate(const float x,
              const float y,
              const float z);

  void frustum(const float left,
               const float right,
               const float bottom,
               const float top,
               const float near,
               const float far);

  void transform(const uint32_t num_verts,
                 const vec3f_t *in,
                 vec3f_t *out);

  void transform(const uint32_t num_verts,
                 const vec4f_t *in,
                 vec4f_t *out);

  void transpose();

  void identity();

protected:
  float e[16];
};


// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

vec3f_t vec3(const vec2f_t &v, float z = 0.f);

vec3f_t vec3(const vec4f_t &v);

vec4f_t vec4(const vec2f_t &v, float z = 0.f, float w = 1.f);

vec4f_t vec4(const vec3f_t &v, float w = 1.f);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector component construction

vec2f_t vec2(const float x,
             const float y);

vec3f_t vec3(const float x,
             const float y,
             const float z);

vec4f_t vec4(const float x,
             const float y,
             const float z,
             const float w);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector addition

vec2f_t operator+(const vec2f_t &a,
                  const vec2f_t &b);

vec3f_t operator+(const vec3f_t &a,
                  const vec3f_t &b);

vec4f_t operator+(const vec4f_t &a,
                  const vec4f_t &b);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector subtraction

vec2f_t operator-(const vec2f_t &a,
                  const vec2f_t &b);

vec3f_t operator-(const vec3f_t &a,
                  const vec3f_t &b);

vec4f_t operator-(const vec4f_t &a,
                  const vec4f_t &b);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector in place addition

void operator+=(vec2f_t &a,
                const vec2f_t &b);

void operator+=(vec3f_t &a,
                const vec3f_t &b);

void operator+=(vec4f_t &a,
                const vec4f_t &b);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector subtraction in place

void operator-=(vec2f_t &a,
                const vec2f_t &b);

void operator-=(vec3f_t &a,
                const vec3f_t &b);

void operator-=(vec4f_t &a,
                const vec4f_t &b);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector scale

vec2f_t operator*(const vec2f_t &a,
                  const float s);

vec3f_t operator*(const vec3f_t &a,
                  const float s);

vec4f_t operator*(const vec4f_t &a,
                  const float s);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// vector dot product

float operator*(const vec2f_t &a,
                const vec2f_t &b);

float operator*(const vec3f_t &a,
                const vec3f_t &b);

float operator*(const vec4f_t &a,
                const vec4f_t &b);

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

vec3f_t operator/(const vec3f_t &a,
                  const float s);

} // namespace math
