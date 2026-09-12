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

// Device function: binary search in sorted hash array
__device__ bool isNeighborPair(int idA, int idB,
                               const long long* neighborHashes, int numHashes) {
  // Handle case with no neighbor data
  if (numHashes == 0 || neighborHashes == nullptr)
    return false;

  // Ensure idA < idB for consistent hashing
  if (idA > idB) {
    int temp = idA;
    idA      = idB;
    idB      = temp;
  }

  long long hash = ((long long)idA << 32) | idB;

  // Binary search
  int left = 0, right = numHashes - 1;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    if (neighborHashes[mid] == hash)
      return true;
    if (neighborHashes[mid] < hash)
      left = mid + 1;
    else
      right = mid - 1;
  }
  return false;
}

__device__ double broadphaseAxisValue(const double3& p, int axis) {
  return axis == 0 ? p.x : (axis == 1 ? p.y : p.z);
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

  const AABB& Ai = sortedAABBs[i];
  unsigned long long count = 0;

  for (int j = i + 1; j < n; ++j) {
    const AABB& Aj = sortedAABBs[j];

    if (broadphaseAxisValue(Aj.min, axis) >
        broadphaseAxisValue(Ai.max, axis))
      break;

    bool overlapX = (Ai.min.x <= Aj.max.x && Aj.min.x <= Ai.max.x);
    bool overlapY = (Ai.min.y <= Aj.max.y && Aj.min.y <= Ai.max.y);
    bool overlapZ = (Ai.min.z <= Aj.max.z && Aj.min.z <= Ai.max.z);

    if (overlapX && overlapY && overlapZ) {
      if (!enableSelfCollision && elementMeshIds != nullptr) {
        int meshIdA = elementMeshIds[Ai.objectId];
        int meshIdB = elementMeshIds[Aj.objectId];
        if (meshIdA == meshIdB) {
          continue;
        }
      }
      // Check if they are neighbors - skip if true
      if (!isNeighborPair(Ai.objectId, Aj.objectId, neighborHashes,
                          numHashes)) {
        count++;
      }
    }
  }

  collisionCounts[i] = count;
}

// Kernel to generate collision pairs (with neighbor filtering)
__global__ void generateCollisionPairsKernel(
    const AABB* sortedAABBs, const unsigned long long* collisionOffsets,
    CollisionPair* collisionPairs, int n, const long long* neighborHashes,
    int numHashes, const int* elementMeshIds, int enableSelfCollision, int axis) {
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;

  const AABB& Ai = sortedAABBs[i];
  unsigned long long writeIdx = collisionOffsets[i];

  for (int j = i + 1; j < n; ++j) {
    const AABB& Aj = sortedAABBs[j];

    if (broadphaseAxisValue(Aj.min, axis) >
        broadphaseAxisValue(Ai.max, axis))
      break;

    bool overlapX = (Ai.min.x <= Aj.max.x && Aj.min.x <= Ai.max.x);
    bool overlapY = (Ai.min.y <= Aj.max.y && Aj.min.y <= Ai.max.y);
    bool overlapZ = (Ai.min.z <= Aj.max.z && Aj.min.z <= Ai.max.z);

    if (overlapX && overlapY && overlapZ) {
      if (!enableSelfCollision && elementMeshIds != nullptr) {
        int meshIdA = elementMeshIds[Ai.objectId];
        int meshIdB = elementMeshIds[Aj.objectId];
        if (meshIdA == meshIdB) {
          continue;
        }
      }
      // Check if they are neighbors - skip if true
      if (!isNeighborPair(Ai.objectId, Aj.objectId, neighborHashes,
                          numHashes)) {
        collisionPairs[writeIdx++] = CollisionPair(Ai.objectId, Aj.objectId);
      }
    }
  }
}
