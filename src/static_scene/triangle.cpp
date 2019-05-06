#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  BBox bb(p1);
  bb.expand(p2); 
  bb.expand(p3);
  return bb;

}

bool in_(double val, double floor, double roof, double tolerance) {
  return val > floor - tolerance && val < roof + tolerance;
}

bool Triangle::intersect(const Ray& r) const {


  /*
Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p0(mesh->positions[v3]);

  auto e1 = p1 - p0, e2 = p2 - p0, s = r.o - p0, s1 = cross(r.d, e2), s2 = cross(s, e1);
  auto vec = (1 / (dot(s1, e1)) * Vector3D(
          dot(s2, e2),
          dot(s1, s),
          dot(s2, r.d)
  ));
  auto t = vec.x, b1 = vec.y, b2 = vec.z, b0 = 1 - b1 - b2;

  if (
          in_(b1, 0, 1, EPS_D) &&
          in_(b2, 0, 1, EPS_D) &&
          in_(b0, 0, 1, EPS_D) &&
          in_(t, r.min_t, r.max_t, 0.01)
          ) {
    r.max_t = t;
    return true;
  }
  return false;
   */

  // TODO (Part 1.3):
  // implement ray-triangle intersection

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p0(mesh->positions[v3]);
  auto e1 = p1 - p0, e2 = p2 - p0, s = r.o - p0, s1 = cross(r.d, e2), s2 = cross(s, e1);
  auto vec = (1 / (dot(s1, e1)) * Vector3D(
          dot(s2, e2),
          dot(s1, s),
          dot(s2, r.d)
          ));
  double t = vec.x, b1 = vec.y, b2 = vec.z, b3 = 1 - b1 - b2;
  if (b1 <= 1 && b1 >= 0 && b2 <= 1 && b2 >= 0 && b3 <= 1 && b3 >= 0 && t <= r.max_t && t >= r.min_t) {
    r.max_t = t;
    return true;
  }

  return false;


}



bool Triangle::intersect(const Ray& r, Intersection *isect) const {

  const Ray* old = &r;
  Ray ray = old->surface_tangent();
  //printf("r: %f, %f, %f\n", r.d.x, r.d.y, r.d.z);
  //printf("ray: %f, %f, %f\n\n", ray.d.x, ray.d.y, ray.d.z);
  //ray.d = r.d;
  ray.min_t = r.min_t; // old->min_t;
  ray.max_t = old->step_size;//r.max_t;// old->step_size;
  if (false) {
    Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p0(mesh->positions[v3]);
    Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n0(mesh->normals[v3]);
    auto e1 = p1 - p0, e2 = p2 - p0, s = r.o - p0, s1 = cross(r.d, e2), s2 = cross(s, e1);
    auto vec = (1 / (dot(s1, e1)) * Vector3D(
            dot(s2, e2),
            dot(s1, s),
            dot(s2, r.d)
    ));
    double t = vec.x, b1 = vec.y, b2 = vec.z, b0 = 1 - b1 - b2;
    if (b1 <= 1 && b1 >= 0 && b2 <= 1 && b2 >= 0 && b0 <= 1 && b0 >= 0 && t <= r.max_t && t >= r.min_t) {

      isect->t = t;
      isect->primitive = this;
      isect->bsdf=get_bsdf();
      isect->n = b1 * n1 + b2 * n2 + b0 *n0;
      isect -> n /= isect -> n.norm();
      r.max_t = t;
      return true;
    }
    return false;


  }
  for(int i=0; i < 100; i++) {
    Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p0(mesh->positions[v3]);
    Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n0(mesh->normals[v3]);
    auto e1 = p1 - p0, e2 = p2 - p0, s = ray.o - p0, s1 = cross(ray.d, e2), s2 = cross(s, e1);
    auto vec = (1 / (dot(s1, e1)) * Vector3D(
            dot(s2, e2),
            dot(s1, s),
            dot(s2, ray.d)
    ));
    double t = vec.x, b1 = vec.y, b2 = vec.z, b0 = 1 - b1 - b2;
    if (b1 <= 1 && b1 >= 0 && b2 <= 1 && b2 >= 0 && b0 <= 1 && b0 >= 0 && t <= ray.max_t && t >= ray.min_t) {

      isect->t = t;
      isect->primitive = this;
      isect->bsdf=get_bsdf();
      isect->n = b1 * n1 + b2 * n2 + b0 *n0;
      isect -> n /= isect -> n.norm();
      ray.max_t = t;
      r.max_t = t;
      return true;
    }

    auto temp = ray;
    ray = old->next_ray();
    ray.min_t = 0;
    ray.max_t = old->step_size;
    if (i == 18) {
      ray.max_t = INF_D;
    }

    old = &temp;
  }


  return false;
  // TODO (Part 1.3):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  /*

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p0(mesh->positions[v3]);
  Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n0(mesh->normals[v3]);

  auto e1 = p1 - p0, e2 = p2 - p0, s = r.o - p0, s1 = cross(r.d, e2), s2 = cross(s, e1);
  auto vec = (1 / (dot(s1, e1)) * Vector3D(
          dot(s2, e2),
          dot(s1, s),
          dot(s2, r.d)
  ));
  auto t = vec.x, b1 = vec.y, b2 = vec.z, b0 = 1 - b1 - b2;



  if (b1 <= 1 && b1 >= 0 && b2 <= 1 && b2 >= 0 && b0 <= 1 && b0 >= 0 && t <= r.max_t && t >= r.min_t) {
    isect->t = t;
    isect->primitive = this;
    isect->bsdf=get_bsdf();
    isect->n = b1 * n1 + b2 * n2 + b0 *n0;
    isect -> n /= isect -> n.norm();
    r.max_t = t;
    return true;
  }
  return false;
   */

}

void Triangle::draw(const Color& c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CGL
