#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>
//#include <CoreImage/CoreImage.h>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
  return true;

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  double
          t_min_x = (min.x - r.o.x) / r.d.x,
          t_max_x = (max.x - r.o.x) / r.d.x;

  if (t_min_x > t_max_x) std::swap(t_min_x, t_max_x);

  double
          t_min_y = (min.y - r.o.y) / r.d.y,
          t_max_y = (max.y - r.o.y) / r.d.y;

  if (t_min_y > t_max_y) std::swap(t_min_y, t_max_y);
  if ((t_min_x > t_max_y) || (t_min_y > t_max_x)) return false;
  if (t_min_y > t_min_x) t_min_x = t_min_y;
  if (t_max_y < t_max_x) t_max_x = t_max_y;

  double
          t_min_z = (min.z - r.o.z) / r.d.z,
          t_max_z = (max.z - r.o.z) / r.d.z;

  if (t_min_z > t_max_z) std::swap(t_min_z, t_max_z);
  if ((t_min_x > t_max_z) || (t_min_z > t_max_x)) return false;
  if (t_min_z > t_min_x) t_min_x = t_min_z;
  if (t_max_x > t_max_z) t_max_x = t_max_z;

  double
          new_t0 = std::max({t_min_x, t_min_y, t_min_z}),
          new_t1 = std::min({t_max_x, t_max_y, t_max_z});
  t0 = new_t0;
  t1 = new_t1;
  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
