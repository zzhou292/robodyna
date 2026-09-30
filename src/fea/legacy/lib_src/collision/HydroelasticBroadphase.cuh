/*==============================================================
 *==============================================================
 * Project: RoboDyna
 * Author:  Json Zhou
 * Email:   zzhou292@wisc.edu
 * File:    HydroelasticBroadphase.cuh
 * Brief:   Declares GPU broadphase collision detection utilities for
 *          tetrahedral meshes, including AABB generation and collision-pair
 *          candidate enumeration for the narrowphase.
 *==============================================================
 *==============================================================*/

#pragma once

#include <cuda_runtime.h>

#include <Eigen/Dense>
#include <cstddef>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <vector>

#include "HydroelasticBroadphaseTypes.cuh"

namespace ANCFCPUUtils {
class MeshManager;
}

// Definition of GPU_ANCF3243 and data access device functions


// Opt-in bounds for linear surface primitives. All device buffers are borrowed,
// in the same element/node order as Initialize(), and must remain alive until
// the default CUDA stream finishes CreateAABB(). Inflation is a nonnegative
// radius (e.g. shell half-thickness), not a full shell thickness.
struct BroadphaseAABBOptions {
  double inflation = 0.0;
  const double* d_elementInflation = nullptr;  // added to uniform inflation
  const double* d_endNodes = nullptr;  // optional 3*n_nodes, column-major
};

struct BroadphaseDetectionLimits {
  unsigned long long maxPairs = std::numeric_limits<int>::max();
  // Counts, offsets, CUB scan scratch and retained pair storage only. Mesh,
  // neighbor map and sorting storage are not included in this budget.
  size_t maxWorkspaceBytes = std::numeric_limits<size_t>::max();
};

// Hash function for pair (for CPU unordered_set)
struct PairHash {
  std::size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<long long>()(((long long)p.first << 32) | p.second);
  }
};

struct Broadphase {
#if defined(__CUDACC__)
  // Device getters for node coordinates (column-major indexing)
  __device__ double node_x(int node_id) const {
    return d_nodes[node_id];
  }

  __device__ double node_y(int node_id) const {
    return d_nodes[node_id + n_nodes];
  }

  __device__ double node_z(int node_id) const {
    return d_nodes[node_id + 2 * n_nodes];
  }

  // Device getter for element node IDs (column-major indexing)
  __device__ int element_node(int elem_id, int local_node_idx) const {
    return d_elements[elem_id + local_node_idx * n_elems];
  }
#endif

  // Host data
  std::vector<AABB> h_aabbs;
  std::vector<CollisionPair> h_collisionPairs;
  int numObjects;
  int numCollisions;

  // Mesh data (host)
  Eigen::MatrixXd h_nodes;
  Eigen::MatrixXi h_elements;
  int n_nodes;
  int n_elems;

  // Neighbor tracking (host)
  std::unordered_set<std::pair<int, int>, PairHash> h_neighborPairs;

  // Device data
  AABB* d_aabbs;

  // Device mesh data
  double* d_nodes;
  int* d_elements;
  int nodesPerElement;
  bool ownsNodes;

  // Element-to-mesh mapping (device). Used to optionally filter out same-mesh
  // (self) collisions efficiently in broadphase.
  int* d_elementMeshIds;

  // If false, collision pairs with elementMeshIds[a] == elementMeshIds[b] are
  // skipped in broadphase.
  bool enableSelfCollision;

  // Host-side copy of element mesh IDs (optional, used for diagnostics).
  std::vector<int> h_elementMeshIds;

  // Device pointer to this struct
  Broadphase* d_bp;

  // Sorting data for sweep and prune
  double* d_sortKeys;    // Sort keys (e.g., min.x values)
  int* d_sortIndices;    // Original indices
  double* d_sortedKeys;  // Sorted keys (output)
  int* d_sortedIndices;  // Sorted indices (output)
  AABB* d_sortedAABBs;   // Sorted AABBs
  void* d_tempStorage;   // Temporary storage for CUB
  size_t tempStorageBytes;
  int sortedAxis;
  bool aabbsReady;
  bool sortedAABBsReady;
  BroadphaseAABBOptions aabbOptions;
  int* d_invalidBounds;

  // Collision detection data
  CollisionPair* d_collisionPairs;  // Device collision pairs

  int* d_sameMeshPairsCount;

  // Neighbor filter data (compact representation for GPU)
  long long* d_neighborPairHashes;  // Sorted array of hashed pairs
  int numNeighborPairs;

  // Reused temporary buffers for collision pair generation (avoid per-step
  // malloc/free)
  // Widen the scan before checking the legacy int-sized output contract.
  // n*(n-1)/2 fits in this type for every supported int-sized mesh.
  unsigned long long* d_collisionCounts;
  unsigned long long* d_collisionOffsets;
  int collisionCountCapacity;  // includes the scan sentinel slot
  void* d_scanTempStorage;
  size_t scanTempStorageBytes;
  int collisionPairsCapacity;
  BroadphaseDetectionLimits detectionLimits;

  bool verbose;

  // Constructor
  Broadphase();

  // Destructor
  ~Broadphase();

  // Initialize GPU resources with mesh data
  void Initialize(const Eigen::MatrixXd& nodes, const Eigen::MatrixXi& elements,
                  const Eigen::VectorXi& elementMeshIds = Eigen::VectorXi());

  // Convenience overload: build element-to-mesh mapping from MeshManager.
  void Initialize(const ANCFCPUUtils::MeshManager& mesh_manager);

  int GetElementMeshIdHost(int elem_id) const {
    if (elem_id < 0 || elem_id >= static_cast<int>(h_elementMeshIds.size())) {
      return 0;
    }
    return h_elementMeshIds[elem_id];
  }

  void EnableSelfCollision(bool enable);

  // Update only nodal positions on the device while reusing existing
  // topology, neighbor maps, and allocated buffers.
  // - Expect same number of nodes and 3 columns.
  // - Does NOT rebuild neighbor maps or reallocate device memory.
  void UpdateNodes(const Eigen::MatrixXd& nodes);

  void RetrieveAABBandPrints();

  // Destroy/cleanup GPU resources
  void Destroy();

  // Create/update AABBs from mesh data. Defaults retain instantaneous node
  // extrema. Optional endpoint union bounds straight-line nodal motion of
  // linear primitives, NOT a curved ANCF interpolation/trajectory. Operations
  // use the default CUDA stream; callers must order external-buffer writes.
  // Nonfinite coordinates/inflation fail explicitly rather than losing pairs.
  void CreateAABB(bool copyToHost = false);

  void SetAABBOptions(const BroadphaseAABBOptions& options);

  // Limits cause std::length_error, never truncation. Observed CUDA API errors
  // throw std::runtime_error. A thrown detection leaves numCollisions == 0 and
  // an empty host result; raise the limit and retry the same sorted bounds.
  void SetDetectionLimits(const BroadphaseDetectionLimits& limits);
  size_t GetDetectionWorkspaceBytes() const;

  // Bind an externally-managed device node buffer (column-major, length
  // 3*n_nodes) to avoid per-step host->device copies. Caller owns the buffer
  // lifetime.
  void BindNodesDevicePtr(double* d_nodes_external);

  void SetVerbose(bool enable) {
    verbose = enable;
  }

  // Sort AABBs along specified axis (0=x, 1=y, 2=z)
  void SortAABBs(int axis = 0);

  // Print sorted AABBs for verification
  void PrintSortedAABBs(int axis = 0);

  // Build neighbor connectivity map
  void BuildNeighborMap();

  // Detect collisions using sweep and prune (with neighbor filtering).
  // With copyPairsToHost=false, the count is validated but pair fill remains
  // enqueued on the default stream. The caller must order downstream reads
  // and check completion for asynchronous execution failures. No global CUDA
  // synchronization is added to this resident-data path.
  void DetectCollisions(bool copyPairsToHost = false);

  int CountSameMeshPairsDevice() const;

  const CollisionPair* GetCollisionPairsDevicePtr() const {
    return numCollisions == 0 ? nullptr : d_collisionPairs;
  }

  // Print collision pairs
  void PrintCollisionPairs();
};
