#define _SDL_main_h
#include <SDL/SDL.h>

#include <array>
#include <cassert>
#include <cstdint>

#include "math.h"

using namespace math;

// return true if a triangle is backfacing
bool is_backface(const vec2f_t &a, const vec2f_t &b, const vec2f_t &c) {
  const float v1 = a.x - b.x;
  const float v2 = a.y - b.y;
  const float w1 = c.x - b.x;
  const float w2 = c.y - b.y;
  return 0.f > (v1 * w2 - v2 * w1);
}

// return true if a triangle intersect unit square [0,0] -> [1,1]
bool tri_vis(const vec2f_t &a, const vec2f_t &b, const vec2f_t &c) {

  // cohen-sutherland style trivial clipping
  {
    const auto classify = [](const vec2f_t &p) -> int {
      return (p.x < 0.f ? 1 : 0) | (p.x > 1.f ? 2 : 0) | (p.y < 0.f ? 4 : 0) |
             (p.y > 1.f ? 8 : 0);
    };

    const int ca = classify(a), cb = classify(b), cc = classify(c);

    if (0 == (ca | cb | cc)) {
      // all in center, no clipping
      return true;
    }

    const int code = ca & cb & cc;
    if ((code & 1) || (code & 2) || (code & 4) || (code & 8)) {
      // all outside one plane
      return false;
    }
  }

  // reject backfaces
  if (is_backface(a, b, c)) {
    return false;
  }

  // plane distance rejection
  {
    struct plane_t {

      plane_t(const vec2f_t &a, const vec2f_t &b)
          : nx(b.y - a.y), ny(a.x - b.x), d(a.x * nx + a.y * ny) {}

      bool test(const vec2f_t &p) const { return d > (p.x * nx + p.y * ny); }

      float nx, ny, d;
    };

    const vec2f_t ndc[4] = {{0.f, 0.f}, {1.f, 0.f}, {0.f, 1.f}, {1.f, 1.f}};

    // XXX: we can do trivial rejects with closest vertex first

    const plane_t pab{a, b};
    if (pab.test(ndc[0]) && pab.test(ndc[1]) && pab.test(ndc[2]) &&
        pab.test(ndc[3])) {
      return false;
    }

    const plane_t pbc{b, c};
    if (pbc.test(ndc[0]) && pbc.test(ndc[1]) && pbc.test(ndc[2]) &&
        pbc.test(ndc[3])) {
      return false;
    }

    const plane_t pca{c, a};
    if (pca.test(ndc[0]) && pca.test(ndc[1]) && pca.test(ndc[2]) &&
        pca.test(ndc[3])) {
      return false;
    }
  }

  // in, but needs clipping
  return true;
}

// dot product between two 2d vectors
constexpr float dot(const vec2f_t &a, const vec2f_t &b) {
  return a.x * b.x + a.y * b.y;
}

// plot a pixel to the screen
void plot(SDL_Surface *surf, float x, float y, uint32_t rgb = 0xdadada) {
  assert(surf);
  if (x < 0.f || y < 0.f || x >= surf->w || y >= surf->h) {
    return;
  }
  uint32_t *pix = (uint32_t *)surf->pixels;
  pix[int(x) + int(y) * surf->w] = rgb;
}

constexpr int32_t minv(int32_t a, int32_t b) { return a < b ? a : b; }

constexpr int32_t maxv(int32_t a, int32_t b) { return a > b ? a : b; }

constexpr int32_t clampv(int32_t lo, int32_t v, int32_t hi) {
  return minv(hi, maxv(lo, v));
}

enum clip_span_t { CLIP_SPAN_MIN_X, CLIP_SPAN_MAX_X };

template <clip_span_t CLIP, size_t SIZE>
void scan_convert(vec2f_t a, vec2f_t b, std::array<int32_t, SIZE> &span) {

  // XXX: move into global attribute
  static const int32_t screen_w = 511;
  static const int32_t screen_h = 511;

  // assume our vertices are pre-sorted
  __assume(a.y < b.y);

  // scanline rejection
  if (b.y < 0.f || a.y > screen_h) {
    return;
  }

  // reject if vertices are co-linear
  if (b.y <= a.y) {
    return;
  }
  const float dx = (b.x - a.x) / (b.y - a.y);

  // clip to top of window edge
  if (a.y < 0.f) {
    a.x += dx * (0.f - a.y);
    a.y = 0.f;
  } else {
    // align to start of scanline
    const float ceily = ceilf(a.y);
    a.x += dx * (ceily - a.y);
    a.y = ceily;
  }

  const int32_t iay = maxv(int32_t(a.y), 0);
  const int32_t iby = minv(int32_t(b.y), screen_h);

  int32_t x = int32_t(a.x * float(0x10000));
  const int32_t idx = int32_t(dx * float(0x10000));

  switch (CLIP) {
  case CLIP_SPAN_MAX_X:
    for (uint32_t y = iay; y <= iby; ++y, x += idx) {
      span[y] = std::max<int32_t>(x >> 16, 0);
    }
    break;
  case CLIP_SPAN_MIN_X:
    for (uint32_t y = iay; y <= iby; ++y, x += idx) {
      span[y] = std::min<int32_t>(x >> 16, screen_w);
    }
    break;
  }
}

// scan convert a triangle
bool scan_triangle(SDL_Surface *surf, std::array<vec2f_t, 3> v, uint32_t rgb) {

  // sort vertices: top (0), mid (1), bottom (2)
  if (v[1].y < v[0].y)
    std::swap(v[1], v[0]);
  if (v[2].y < v[0].y)
    std::swap(v[2], v[0]);
  if (v[2].y < v[1].y)
    std::swap(v[2], v[1]);

  // check mid vertex side
  const float nx = v[2].y - v[0].y;
  const float ny = v[0].x - v[2].x;
  const float d1 = v[0].x * nx + v[0].y * ny;
  const float d2 = v[1].x * nx + v[1].y * ny;

  if (d1 == d2) {
    // coplanar, so reject triangle
    return false;
  }

  // our y axis span buffers
  std::array<int32_t, 512> lo, hi;

  // scan convert edges
  if (d1 > d2) {
    scan_convert<CLIP_SPAN_MIN_X>(v[0], v[2], hi);
    scan_convert<CLIP_SPAN_MAX_X>(v[0], v[1], lo);
    scan_convert<CLIP_SPAN_MAX_X>(v[1], v[2], lo);
  } else {
    scan_convert<CLIP_SPAN_MAX_X>(v[0], v[2], lo);
    scan_convert<CLIP_SPAN_MIN_X>(v[0], v[1], hi);
    scan_convert<CLIP_SPAN_MIN_X>(v[1], v[2], hi);
  }

  // fill triangle
  {
    const int32_t y0 = std::max(int32_t(ceilf(v[0].y)), 0);
    const int32_t y1 = std::min(int32_t(v[2].y), 511);

    uint32_t *py = (uint32_t *)surf->pixels;
    py += (y0 * surf->pitch) / 4;
    for (int32_t y = y0; y <= y1; ++y) {
      // step to edge
      uint32_t *px = py + lo[y];
      // raster scanline
      for (int32_t x = lo[y]; x < hi[y]; ++x, ++px) {
        *px = rgb;
      }
      // step scanline
      py += surf->pitch / 4;
    }
  }

  return true;
}

// fast fixed point line drawing
void draw_line(SDL_Surface *surf, math::vec2f_t a, math::vec2f_t b,
               uint32_t rgb) {
  const float dx = b.x - a.x, dy = b.y - a.y;
  const float adx = fabsf(dx), ady = fabs(dy);
  if (fabsf(dx) > fabsf(dy)) {
    if (b.x < a.x)
      std::swap(a, b);
    const int32_t iy = int32_t(float(0x10000) * ((b.y - a.y) / adx));
    int32_t y = int32_t(a.y * float(0x10000));
    for (int32_t x = a.x; x < b.x; ++x, y += iy) {
      plot(surf, x, y >> 16, rgb);
    }
  } else {
    if (b.y < a.y)
      std::swap(a, b);
    const int32_t ix = int32_t(float(0x10000) * ((b.x - a.x) / ady));
    int32_t x = int32_t(a.x * float(0x10000));
    for (int32_t y = a.y; y < b.y; ++y, x += ix) {
      plot(surf, x >> 16, y, rgb);
    }
  }
}

// draw a wireframe triangle
void draw_tri(SDL_Surface *surf, const std::array<math::vec4f_t, 3> &t,
              uint32_t rgb) {

  const std::array<vec2f_t, 3> tri = {
      vec2f_t{t[0].x, t[0].y},
      vec2f_t{t[1].x, t[1].y},
      vec2f_t{t[2].x, t[2].y},
  };

  if (!is_backface(tri[0], tri[2], tri[1])) {

    scan_triangle(surf, tri, 0x335577);
#if 1
    for (uint32_t j = 0; j < 3; ++j) {
      const math::vec4f_t &a = t[j];
      const math::vec4f_t &b = t[j == 2 ? 0 : j + 1];
      draw_line(surf, math::vec2(a.x, a.y), math::vec2(b.x, b.y), 0x446688);
    }
#endif
  }
}
