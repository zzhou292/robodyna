#pragma once
#include "HydroelasticCollisionTypes.cuh"

// Existing broadphase box ABI, shared without owning Eigen/mesh storage.
struct AABB {
  double3 min;
  double3 max;
  int objectId;
};
