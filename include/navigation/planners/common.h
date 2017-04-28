#ifndef PLANNERS_COMMON_H
#define PLANNERS_COMMON_H
#include "ssl_common/geometry.hpp"

// some common definitions required for all the planners
namespace Navigation
{
  typedef struct
  {
    float x, y, x2, y2;
    float radius;
  } obstacle;
  const int MAX_OBSTACLES = 20;
}
#endif