#define _SDL_main_h
#include <SDL.h>

#include <cassert>
#include <cstdint>
#include <array>

#include "math.h"

using namespace math;

void draw_line(SDL_Surface *, math::vec2f_t, math::vec2f_t, uint32_t rgb);
void draw_tri(SDL_Surface *, const std::array<math::vec4f_t, 3> &, uint32_t rgb);

struct app_t {

  vec3f_t rot_;
  SDL_Surface *surf_;
  matrix_t mat_;

  app_t(SDL_Surface *surf)
    : rot_{0.f, 0.f, 0.f}
    , surf_(surf)
  {
    mat_.identity();
  }

  // plot a pixel to the screen
  void plot(float x, float y, uint32_t rgb = 0xdadada) {
    assert(surf_);
    if (x < 0.f || y < 0.f || x >= surf_->w || y >= surf_->h) {
      return;
    }
    uint32_t *pix = (uint32_t *)surf_->pixels;
    pix[int(x) + int(y) * surf_->w] = rgb;
  }

  void render() {
    extern const float obj_vertex[];
    extern const uint32_t obj_index[];
    extern const uint32_t obj_num_vertex;
    extern const uint32_t obj_num_index;

    std::array<vec4f_t, 3> pre;
    std::array<vec4f_t, 3> post;
    for (uint32_t i = 0; i < obj_num_index; i += 3) {

      const std::array<uint32_t, 3> index = {
        obj_index[i + 0],
        obj_index[i + 1],
        obj_index[i + 2],
      };

      const vec3f_t *bunny = (const vec3f_t*)obj_vertex;
      pre[0] = vec4(bunny[index[0]], 1.f);
      pre[1] = vec4(bunny[index[1]], 1.f);
      pre[2] = vec4(bunny[index[2]], 1.f);

      mat_.transform(3, pre.data(), post.data());

      for (auto &v : post) {
        v.x = 256 + v.x * 4.5f;
        v.y = 256 + v.y * 4.5f;
#if 0
        plot(v.x, v.y);
#endif
      }

      draw_tri(surf_, post, 0x88aabb);
    }
  }

  void tick() {
    // update cube rotation
    mat_.rotate(rot_.x, rot_.y, rot_.z);
    rot_ += math::vec3f_t{0.7032f, 0.2345f, 1.2444f} * 0.003f;
    render();
  }
};

// program entry
int main(const int argc, const char **args) {

  if (SDL_Init(SDL_INIT_VIDEO)) {
    return 1;
  }

  SDL_Surface *surf = SDL_SetVideoMode(512, 512, 32, 0);
  if (!surf) {
    return 2;
  }

  app_t app{surf};

  bool active = true;
  while (active) {

    SDL_FillRect(surf, nullptr, 0x101010);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_QUIT:
        active = false;
        break;
      }
    }

    app.tick();

    SDL_Flip(surf);
    SDL_Delay(1);
  }

  return 0;
}
