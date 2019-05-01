#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c, alpha);
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c, alpha);
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

std::vector<std::vector<Primitive*>> split_along_axis(const std::vector<Primitive*>& prims, int index) {

  auto vals = std::vector<double>();
  for(Primitive* prim : prims) {
      double curr = prim -> get_bbox() . centroid() [index];
      vals.push_back(curr);
  }
  auto n = vals.size() / 2;
  nth_element(vals.begin(), vals.begin() + n, vals.end());
  double median = vals[n];

  std::vector<Primitive*> left, right;
  for(Primitive* prim: prims) {
      auto curr = prim -> get_bbox() . centroid()[index];
      if (curr < median) {
        left.push_back(prim);
      } else {
        right.push_back(prim);
      }
  }

  if (left.empty()) {
    left.push_back(right.back());
    right.pop_back();
  }
  if (right.empty()) {
    right.push_back(left.back());
    left.pop_back();
  }

  std::vector<std::vector<Primitive*>> ret;
  ret.push_back(left);
  ret.push_back(right);
  return ret;
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
      BBox bb = p->get_bbox();
      bbox.expand(bb);
      Vector3D c = bb.centroid();
      centroid_box.expand(c);
  }

  BVHNode *node = new BVHNode(bbox);

  if (prims.size() <= max_leaf_size) {
    node->prims = new vector<Primitive *>(prims);
  } else {
    auto extent = bbox.extent;
    int index;

    if (extent.x > extent.y && extent.x > extent.z) index = 0;
    else if(extent.y > extent.x && extent.y > extent.z) index = 1;
    else index = 2;

    auto ret = split_along_axis(prims, index);
    auto left = ret[0], right = ret[1];
    node->l = construct_bvh(left, max_leaf_size);
    node->r = construct_bvh(right, max_leaf_size);
  }


  return node;


}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) return false;


  //if(t0 < ray.min_t || t1 > ray.max_t) return false;

  if (node->isLeaf()) {
    for (Primitive *p : *node->prims) {
      total_isects++;
      if (p->intersect(ray))
        return true;
    }
    return false;
  } else {

    auto left = intersect(ray, node->l),
            right = intersect(ray, node->r);
    return right || left ;
  }
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.

  //double t0 = ray.min_t, t1 = ray.max_t;
  //if (!node->bb.intersect(ray, t0, t1)) return false;

 //if(t0 < ray.min_t || t1 > ray.max_t) return false;

  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) return false;
  if (node->isLeaf()) {
    bool hit = false;
    for (Primitive *p : *node->prims) {
      total_isects++;
      if (p->intersect(ray, i)) hit = true;
    }
    return hit;
  } else {

    auto left = intersect(ray, i, node->l);
    auto right = intersect(ray, i, node->r);
    return right || left;
  }
}

}  // namespace StaticScene
}  // namespace CGL
