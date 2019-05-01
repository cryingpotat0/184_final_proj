#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  double a = r.d.norm2(), b = 2*dot(r.o - o, r.d), c = (r.o - o).norm2() - r2;
  auto term1 = -b / (2 * a), term2 = sqrt(b*b - 4*a*c) / (2*a);
  if (isnan(term2)) return false;
  t1 = term1 - term2; t2 = term1 + term2;
  if (t1 > t2) { std::swap(t1, t2); }

  return t1 >= 0 && t2 >= 0;


}

bool Sphere::intersect(const Ray& r) const {
  return false;

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  bool intersects = test(r, t1, t2);
  if (intersects && t1 <= r.max_t && t1 >= r.min_t) {
    r.max_t = t1;
    return true;
  }

  return false;


}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  return false;
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
  bool intersects = test(r, t1, t2);
  if (intersects && t1 <= r.max_t && t1 >= r.min_t) {
    i->bsdf = get_bsdf();
    i->primitive = this;
    i->t = t1;
    i -> n = (r.o + t1 * r.d - o);
    i -> n /= i -> n.norm();
    r.max_t = t1;
    return true;
  }

  return false;

  
}

void Sphere::draw(const Color& c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c, float alpha) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
