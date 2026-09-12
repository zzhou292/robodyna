#pragma once
#include "../HydroelasticBroadphaseTypes.cuh"

namespace broadphase_detail {
// Exact existing sweep/overlap traversal. Filter and consumer select policy and
// output storage; neither changes the complete candidate enumeration.
__host__ __device__ inline double AxisValue(const double3& p, int axis) {
  return axis == 0 ? p.x : (axis == 1 ? p.y : p.z);
}
__host__ __device__ inline bool IsNeighborPair(int idA, int idB,
    const long long* hashes, int count) {
  if (count == 0 || hashes == nullptr) return false;
  if (idA > idB) { int temporary = idA; idA = idB; idB = temporary; }
  const long long hash = (static_cast<long long>(idA) << 32) | idB;
  int left = 0, right = count - 1;
  while (left <= right) {
    const int middle = left + (right - left) / 2;
    if (hashes[middle] == hash) return true;
    if (hashes[middle] < hash) left = middle + 1;
    else right = middle - 1;
  }
  return false;
}
struct LegacyFilter {
  const long long* neighbors;
  int neighbor_count;
  const int* mesh_ids;
  int enable_self_collision;
  __host__ __device__ bool operator()(int a, int b) const {
    if (!enable_self_collision && mesh_ids != nullptr && mesh_ids[a] == mesh_ids[b]) return false;
    return !IsNeighborPair(a, b, neighbors, neighbor_count);
  }
};
struct KeepAll {
  __host__ __device__ bool operator()(int, int) const { return true; }
};
struct CountPairs {
  unsigned long long count = 0;
  __host__ __device__ void operator()(int, int) { ++count; }
};
struct WriteLegacyPairs {
  CollisionPair* pairs;
  unsigned long long offset;
  __host__ __device__ void operator()(int a, int b) { pairs[offset++] = CollisionPair(a, b); }
};
template<class Filter, class Consumer>
__host__ __device__ inline void VisitLater(const AABB* sorted, int n, int i,
    int axis, const Filter& keep, Consumer& consumer) {
  const AABB& a = sorted[i];
  for (int j = i + 1; j < n; ++j) {
    const AABB& b = sorted[j];
    if (AxisValue(b.min, axis) > AxisValue(a.max, axis)) break;
    const bool x = a.min.x <= b.max.x && b.min.x <= a.max.x;
    const bool y = a.min.y <= b.max.y && b.min.y <= a.max.y;
    const bool z = a.min.z <= b.max.z && b.min.z <= a.max.z;
    if (x && y && z && keep(a.objectId, b.objectId)) consumer(a.objectId, b.objectId);
  }
}
} // namespace broadphase_detail
