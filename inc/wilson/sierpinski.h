#include <vector>

#include "s2/s2edge_distances.h"
#include "s2/s2loop.h"
#include "s2/s2polygon.h"
#include "s2/s2point.h"
#include "s2/s2text_format.h"

namespace w {

static inline void SierpinskiHole(std::vector<std::unique_ptr<S2Loop>>& loops,
  const S2Point& a, const S2Point& b, const S2Point& c, int max_level, int level = 0) {
  if (level == max_level) {
    return;
  }

  S2Point abm = S2::Interpolate(a, b, 0.5);
  S2Point bcm = S2::Interpolate(b, c, 0.5);
  S2Point cam = S2::Interpolate(c, a, 0.5);

  // Move points towards the center slightly.
  const S2Point center = ((abm+bcm+cam)/3).Normalize();
  S2Point abm2 = S2::Interpolate(abm, center, 1e-3);
  S2Point bcm2 = S2::Interpolate(bcm, center, 1e-3);
  S2Point cam2 = S2::Interpolate(cam, center, 1e-3);

  loops.emplace_back(
    std::make_unique<S2Loop>(std::vector<S2Point>{abm2, cam2, bcm2}));

  SierpinskiHole(loops, a, abm, cam, max_level, level+1);
  SierpinskiHole(loops, b, bcm, abm, max_level, level+1);
  SierpinskiHole(loops, c, cam, bcm, max_level, level+1);
}

static inline S2Polygon Sierpinski(int max_level = 4) { //
  const S2Point a = S2LatLng::FromDegrees(-40, -40).ToPoint();
  const S2Point b = S2LatLng::FromDegrees(-40, +40).ToPoint();
  const S2Point c = S2LatLng::FromDegrees(+40, 0).ToPoint();

  std::vector<std::unique_ptr<S2Loop>> loops;
  loops.emplace_back(std::make_unique<S2Loop>(std::vector<S2Point>{a, b, c}));

  SierpinskiHole(loops, a, b, c, max_level);

  S2Polygon fractal;
  fractal.InitOriented(std::move(loops));
  return fractal;
}

}  // namespace w
