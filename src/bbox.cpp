#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>
#include <CoreImage/CoreImage.h>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
  return true;

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  if (r.curved) {
	  double t_min_x_1 = (r.b.x + sqrt(r.b.x*r.b.x - 4 * r.a.x * (r.c.x + min.x))) / (2.0 * r.a.x);
	  double t_min_x_2 = (r.b.x - sqrt(r.b.x*r.b.x - 4 * r.a.x * (r.c.x + min.x))) / (2.0 * r.a.x);
	  double t_max_x_1 = (r.b.x + sqrt(r.b.x*r.b.x - 4 * r.a.x * (r.c.x + max.x))) / (2.0 * r.a.x);
	  double t_max_x_2 = (r.b.x - sqrt(r.b.x*r.b.x - 4 * r.a.x * (r.c.x + max.x))) / (2.0 * r.a.x);
	  double t_min_x = std::min(t_min_x_1, t_min_x_2);
	  double t_max_x = std::min(t_max_x_1, t_max_x_2);

	  if (t_min_x > t_max_x) std::swap(t_min_x, t_max_x);

	  double t_min_y_1 = (r.b.y + sqrt(r.b.y*r.b.y - 4 * r.a.y * (r.c.y + min.y))) / (2.0 * r.a.y);
	  double t_min_y_2 = (r.b.y - sqrt(r.b.y*r.b.y - 4 * r.a.y * (r.c.y + min.y))) / (2.0 * r.a.y);
	  double t_max_y_1 = (r.b.y + sqrt(r.b.y*r.b.y - 4 * r.a.y * (r.c.y + max.y))) / (2.0 * r.a.y);
	  double t_max_y_2 = (r.b.y - sqrt(r.b.y*r.b.y - 4 * r.a.y * (r.c.y + max.y))) / (2.0 * r.a.y);
	  double t_min_y = std::min(t_min_y_1, t_min_y_2);
	  double t_max_y = std::min(t_max_y_1, t_max_y_2);

	  if (t_min_y > t_max_y) std::swap(t_min_y, t_max_y);
	  if ((t_min_x > t_max_y) || (t_min_y > t_max_x)) return false;
	  if (t_min_y > t_min_x) t_min_x = t_min_y;
	  if (t_max_y < t_max_x) t_max_x = t_max_y;

	  double t_min_z_1 = (r.b.z + sqrt(r.b.z*r.b.z - 4 * r.a.z * (r.c.z + min.z))) / (2.0 * r.a.z);
	  double t_min_z_2 = (r.b.z - sqrt(r.b.z*r.b.z - 4 * r.a.z * (r.c.z + min.z))) / (2.0 * r.a.z);
	  double t_max_z_1 = (r.b.z + sqrt(r.b.z*r.b.z - 4 * r.a.z * (r.c.z + max.z))) / (2.0 * r.a.z);
	  double t_max_z_2 = (r.b.z - sqrt(r.b.z*r.b.z - 4 * r.a.z * (r.c.z + max.z))) / (2.0 * r.a.z);
	  double t_min_z = std::min(t_min_z_1, t_min_z_2);
	  double t_max_z = std::min(t_max_z_1, t_max_z_2);

	  if (t_min_z > t_max_z) std::swap(t_min_z, t_max_z);
	  if ((t_min_x > t_max_z) || (t_min_z > t_max_x)) return false;
	  if (t_min_z > t_min_x) t_min_x = t_min_z;
	  if (t_max_x > t_max_z) t_max_x = t_max_z;

	  double
		  new_t0 = std::max({ t_min_x, t_min_y, t_min_z }),
		  new_t1 = std::min({ t_max_x, t_max_y, t_max_z });
	  t0 = new_t0;
	  t1 = new_t1;
	  return true;
  }
  else {

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
		  new_t0 = std::max({ t_min_x, t_min_y, t_min_z }),
		  new_t1 = std::min({ t_max_x, t_max_y, t_max_z });
	  t0 = new_t0;
	  t1 = new_t1;
	  return true;
  }
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
