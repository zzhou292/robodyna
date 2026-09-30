/*==============================================================
 *==============================================================
 * Project: RoboDyna
 * Author:  Json Zhou
 * Email:   zzhou292@wisc.edu
 * File:    HydroelasticBroadphaseFunc.cuh
 * Brief:   CUDA device and kernel functions used by the Broadphase module:
 *          element AABB construction, extraction of sort keys, AABB
 *          reordering, neighbor filtering by hashed pairs, and generation of
 *          non-neighbor collision pairs on the GPU.
 *==============================================================
 *==============================================================*/

#pragma once

#include "HydroelasticBroadphase.cuh"
#include "broadphase/Sweep.h"

// Device / kernel functions for Broadphase

// Kernel to compute AABB for each element
__global__ void computeAABBKernel(Broadphase* bp, AABB* aabbs, int n_elems,
                                 BroadphaseAABBOptions options,
                                 int* invalidBounds) {
  unsigned int elem_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (elem_idx >= n_elems)
    return;

  // Get first node ID using element getter
  int first_node_id = bp->element_node(elem_idx, 0);

  // Access node coordinates using getters - use double precision
  double3 min_pt =
      make_double3(bp->node_x(first_node_id), bp->node_y(first_node_id),
                   bp->node_z(first_node_id));
  double3 max_pt = min_pt;

  bool valid = true;
  // Include both motion endpoints. The convex hull of a linear primitive
  // undergoing linear nodal motion is bounded by these endpoint extrema.
  for (int i = 0; i < bp->nodesPerElement; i++) {
    int node_id = bp->element_node(elem_idx, i);

    double x = bp->node_x(node_id);
    double y = bp->node_y(node_id);
    double z = bp->node_z(node_id);
    valid = valid && isfinite(x) && isfinite(y) && isfinite(z);

    min_pt.x = fmin(min_pt.x, x);
    min_pt.y = fmin(min_pt.y, y);
    min_pt.z = fmin(min_pt.z, z);

    max_pt.x = fmax(max_pt.x, x);
    max_pt.y = fmax(max_pt.y, y);
    max_pt.z = fmax(max_pt.z, z);

    if (options.d_endNodes) {
      x = options.d_endNodes[node_id];
      y = options.d_endNodes[node_id + bp->n_nodes];
      z = options.d_endNodes[node_id + 2 * bp->n_nodes];
      valid = valid && isfinite(x) && isfinite(y) && isfinite(z);
      min_pt.x = fmin(min_pt.x, x);
      min_pt.y = fmin(min_pt.y, y);
      min_pt.z = fmin(min_pt.z, z);
      max_pt.x = fmax(max_pt.x, x);
      max_pt.y = fmax(max_pt.y, y);
      max_pt.z = fmax(max_pt.z, z);
    }
  }

  double inflation = options.inflation;
  if (options.d_elementInflation) {
    const double local = options.d_elementInflation[elem_idx];
    valid = valid && isfinite(local) && local >= 0.0;
    inflation = __dadd_ru(inflation, local);
  }
  valid = valid && isfinite(inflation);
  // Directed rounding keeps the requested thickness conservative at large
  // coordinate scales too; default zero inflation leaves node extrema intact.
  min_pt.x = __dsub_rd(min_pt.x, inflation);
  min_pt.y = __dsub_rd(min_pt.y, inflation);
  min_pt.z = __dsub_rd(min_pt.z, inflation);
  max_pt.x = __dadd_ru(max_pt.x, inflation);
  max_pt.y = __dadd_ru(max_pt.y, inflation);
  max_pt.z = __dadd_ru(max_pt.z, inflation);
  valid = valid && isfinite(min_pt.x) && isfinite(min_pt.y) &&
          isfinite(min_pt.z) && isfinite(max_pt.x) && isfinite(max_pt.y) &&
          isfinite(max_pt.z);
  if (!valid) {
    atomicExch(invalidBounds, 1);
  }

  // Store AABB
  aabbs[elem_idx].min      = min_pt;
  aabbs[elem_idx].max      = max_pt;
  aabbs[elem_idx].objectId = elem_idx;
}

__global__ void countSameMeshPairsKernel(const CollisionPair* pairs,
                                         int numPairs,
                                         const int* elementMeshIds,
                                         int* outCount) {
  unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= numPairs) {
    return;
  }

  int a = pairs[idx].idA;
  int b = pairs[idx].idB;
  if (a < 0 || b < 0) {
    return;
  }

  if (elementMeshIds[a] == elementMeshIds[b]) {
    atomicAdd(outCount, 1);
  }
}

// Kernel to extract sort keys from AABBs
__global__ void extractSortKeysKernel(const AABB* aabbs, double* keys,
                                      int* indices, int axis, int n) {
  unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n) {
    // Extract min value for the specified axis
    if (axis == 0) {
      keys[idx] = aabbs[idx].min.x;
    } else if (axis == 1) {
      keys[idx] = aabbs[idx].min.y;
    } else {
      keys[idx] = aabbs[idx].min.z;
    }
    indices[idx] = idx;
  }
}

// Kernel to reorder AABBs based on sorted indices
__global__ void reorderAABBsKernel(const AABB* input, AABB* output,
                                   const int* indices, int n) {
  unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n) {
    output[idx] = input[indices[idx]];
  }
}

// Compatibility helpers retain their original names and semantics.
__device__ bool isNeighborPair(int a, int b, const long long* hashes, int count) {
  return broadphase_detail::IsNeighborPair(a,b,hashes,count);
}
__device__ double broadphaseAxisValue(const double3& p, int axis) {
  return broadphase_detail::AxisValue(p,axis);
}

// Kernel to count potential collisions per element (with neighbor filtering)
__global__ void countCollisionsKernel(const AABB* sortedAABBs,
                                      unsigned long long* collisionCounts, int n,
                                      const long long* neighborHashes,
                                      int numHashes, const int* elementMeshIds,
                                      int enableSelfCollision, int axis) {
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;

  broadphase_detail::CountPairs count;
  broadphase_detail::VisitLater(sortedAABBs,n,i,axis,
      broadphase_detail::LegacyFilter{neighborHashes,numHashes,elementMeshIds,enableSelfCollision},count);
  collisionCounts[i] = count.count;
}

// Kernel to generate collision pairs (with neighbor filtering)
__global__ void generateCollisionPairsKernel(
    const AABB* sortedAABBs, const unsigned long long* collisionOffsets,
    CollisionPair* collisionPairs, int n, const long long* neighborHashes,
    int numHashes, const int* elementMeshIds, int enableSelfCollision, int axis) {
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;

  broadphase_detail::WriteLegacyPairs output{collisionPairs,collisionOffsets[i]};
  broadphase_detail::VisitLater(sortedAABBs,n,i,axis,
      broadphase_detail::LegacyFilter{neighborHashes,numHashes,elementMeshIds,enableSelfCollision},output);
}
