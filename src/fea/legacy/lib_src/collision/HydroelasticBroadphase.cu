/*==============================================================
 *==============================================================
 * Project: RoboDyna
 * Author:  Json Zhou
 * Email:   zzhou292@wisc.edu
 * File:    HydroelasticBroadphase.cu
 * Brief:   Implements the GPU broadphase collision stage for tetrahedral
 *          meshes. Builds AABBs for elements, performs sweep-and-prune along
 *          a chosen axis, filters out mesh neighbors, and uses CUB-based
 *          prefix sums to allocate and generate non-neighbor collision pairs
 *          for the narrowphase.
 *==============================================================
 *==============================================================*/

#include <algorithm>
#include <cmath>
#include <cub/cub.cuh>  // Include CUB only in .cu file
#include <stdexcept>
#include <string>

#include "HydroelasticBroadphase.cuh"
#include "HydroelasticBroadphaseFunc.cuh"
#include "lib_utils/mesh_manager.h"

namespace {
void CheckCuda(cudaError_t error) {
  if (error != cudaSuccess) {
    throw std::runtime_error(std::string("Broadphase CUDA failure: ") +
                             cudaGetErrorString(error));
  }
}

void CheckWorkspace(size_t counts, size_t scan, size_t pairs, size_t limit) {
  // Subtraction also prevents size_t addition overflow.
  if (counts > limit || scan > limit - counts ||
      pairs > limit - counts - scan) {
    throw std::length_error("Broadphase detection workspace budget exceeded");
  }
}
}  // namespace

// Constructor
Broadphase::Broadphase()
    : numObjects(0),
      numCollisions(0),
      n_nodes(0),
      n_elems(0),
      d_aabbs(nullptr),
      d_nodes(nullptr),
      d_elements(nullptr),
      nodesPerElement(0),
      ownsNodes(true),
      d_elementMeshIds(nullptr),
      enableSelfCollision(true),
      d_bp(nullptr),
      d_sortKeys(nullptr),
      d_sortIndices(nullptr),
      d_sortedKeys(nullptr),
      d_sortedIndices(nullptr),
      d_sortedAABBs(nullptr),
      d_tempStorage(nullptr),
      tempStorageBytes(0),
      sortedAxis(0),
      aabbsReady(false),
      sortedAABBsReady(false),
      d_invalidBounds(nullptr),
      d_collisionPairs(nullptr),
      d_sameMeshPairsCount(nullptr),
      d_neighborPairHashes(nullptr),
      numNeighborPairs(0),
      d_collisionCounts(nullptr),
      d_collisionOffsets(nullptr),
      collisionCountCapacity(0),
      d_scanTempStorage(nullptr),
      scanTempStorageBytes(0),
      collisionPairsCapacity(0),
      verbose(false) {}

// Destructor
Broadphase::~Broadphase() {
  Destroy();
}

static void BestEffortCudaFree(void* ptr, const char* name) {
  if (!ptr) {
    return;
  }
  cudaError_t err = cudaFree(ptr);
  if (err == cudaSuccess || err == cudaErrorCudartUnloading) {
    return;
  }
  std::cerr << "cudaFree(" << name << ") failed: " << cudaGetErrorString(err)
            << std::endl;
}

// Initialize GPU resources with mesh data
void Broadphase::Initialize(const Eigen::MatrixXd& nodes,
                            const Eigen::MatrixXi& elements,
                            const Eigen::VectorXi& elementMeshIds) {
  // Validate before destroying a usable prior initialization. Device indexing
  // and CUB item counts are int-sized, including the scan sentinel.
  const auto intMax = std::numeric_limits<int>::max();
  if (nodes.cols() != 3 || nodes.rows() <= 0 || elements.rows() <= 0 ||
      elements.cols() <= 0 || nodes.rows() > intMax / 3 ||
      elements.rows() > intMax - 1 ||
      elements.cols() > intMax / elements.rows() || !nodes.allFinite()) {
    throw std::invalid_argument("Broadphase invalid mesh dimensions/coordinates");
  }
  if ((elementMeshIds.size() != 0 &&
       elementMeshIds.size() != elements.rows()) ||
      elements.minCoeff() < 0 || elements.maxCoeff() >= nodes.rows()) {
    throw std::invalid_argument("Broadphase invalid connectivity or mesh IDs");
  }
  const bool selfCollision = enableSelfCollision;
  Destroy();
  enableSelfCollision = selfCollision;

  // Store mesh dimensions
  n_nodes         = nodes.rows();
  n_elems         = elements.rows();
  nodesPerElement = elements.cols();

  // Store host mesh data
  h_nodes    = nodes;
  h_elements = elements;

  // Use n_elems as the actual number of objects
  numObjects = n_elems;

  // Allocate device memory for AABBs (one per element)
  CheckCuda(cudaMalloc(&d_aabbs, n_elems * sizeof(AABB)));

  // Allocate sorting arrays (input and output buffers)
  CheckCuda(cudaMalloc(&d_sortKeys, n_elems * sizeof(double)));
  CheckCuda(cudaMalloc(&d_sortIndices, n_elems * sizeof(int)));
  CheckCuda(cudaMalloc(&d_sortedKeys, n_elems * sizeof(double)));
  CheckCuda(cudaMalloc(&d_sortedIndices, n_elems * sizeof(int)));
  CheckCuda(cudaMalloc(&d_sortedAABBs, n_elems * sizeof(AABB)));

  // Allocate temporary storage for CUB sorting
  d_tempStorage = nullptr;
  CheckCuda(cub::DeviceRadixSort::SortPairs(
      d_tempStorage, tempStorageBytes, d_sortKeys, d_sortedKeys, d_sortIndices,
      d_sortedIndices, n_elems));
  CheckCuda(cudaMalloc(&d_tempStorage, tempStorageBytes));

  // Allocate and copy mesh data to device
  // Nodes: n_nodes x 3 (column-major)
  CheckCuda(cudaMalloc(&d_nodes, n_nodes * 3 * sizeof(double)));
  CheckCuda(cudaMemcpy(d_nodes, nodes.data(), n_nodes * 3 * sizeof(double),
                          cudaMemcpyHostToDevice));
  ownsNodes = true;

  // Elements: n_elems x nodesPerElement (column-major)
  CheckCuda(
      cudaMalloc(&d_elements, n_elems * nodesPerElement * sizeof(int)));
  CheckCuda(cudaMemcpy(d_elements, elements.data(),
                          n_elems * nodesPerElement * sizeof(int),
                          cudaMemcpyHostToDevice));

  // Allocate and copy element mesh IDs
  h_elementMeshIds.assign(n_elems, 0);
  if (elementMeshIds.size() == n_elems) {
    for (int i = 0; i < n_elems; ++i) {
      h_elementMeshIds[i] = elementMeshIds(i);
    }
  }

  CheckCuda(cudaMalloc(&d_elementMeshIds, n_elems * sizeof(int)));
  CheckCuda(cudaMemcpy(d_elementMeshIds, h_elementMeshIds.data(),
                          n_elems * sizeof(int), cudaMemcpyHostToDevice));

  // Allocate device copy of this struct and copy to device
  CheckCuda(cudaMalloc(&d_bp, sizeof(Broadphase)));
  CheckCuda(
      cudaMemcpy(d_bp, this, sizeof(Broadphase), cudaMemcpyHostToDevice));

  if (d_sameMeshPairsCount == nullptr) {
    CheckCuda(cudaMalloc(&d_sameMeshPairsCount, sizeof(int)));
  }
  CheckCuda(cudaMalloc(&d_invalidBounds, sizeof(int)));

  std::cout << "Broadphase initialized with " << n_nodes << " nodes and "
            << n_elems << " elements" << std::endl;
}

void Broadphase::EnableSelfCollision(bool enable) {
  enableSelfCollision = enable;
  numCollisions = 0;
  h_collisionPairs.clear();

  if (d_bp) {
    CheckCuda(
        cudaMemcpy(d_bp, this, sizeof(Broadphase), cudaMemcpyHostToDevice));
  }
}

// Destroy/cleanup GPU resources
void Broadphase::Destroy() {
  if (d_invalidBounds) {
    BestEffortCudaFree(d_invalidBounds, "d_invalidBounds");
    d_invalidBounds = nullptr;
  }
  if (d_aabbs) {
    BestEffortCudaFree(d_aabbs, "d_aabbs");
    d_aabbs = nullptr;
  }

  if (d_sortKeys) {
    BestEffortCudaFree(d_sortKeys, "d_sortKeys");
    d_sortKeys = nullptr;
  }

  if (d_sortIndices) {
    BestEffortCudaFree(d_sortIndices, "d_sortIndices");
    d_sortIndices = nullptr;
  }

  if (d_sortedKeys) {
    BestEffortCudaFree(d_sortedKeys, "d_sortedKeys");
    d_sortedKeys = nullptr;
  }

  if (d_sortedIndices) {
    BestEffortCudaFree(d_sortedIndices, "d_sortedIndices");
    d_sortedIndices = nullptr;
  }

  if (d_sortedAABBs) {
    BestEffortCudaFree(d_sortedAABBs, "d_sortedAABBs");
    d_sortedAABBs = nullptr;
  }

  if (d_tempStorage) {
    BestEffortCudaFree(d_tempStorage, "d_tempStorage");
    d_tempStorage = nullptr;
  }

  if (d_collisionPairs) {
    BestEffortCudaFree(d_collisionPairs, "d_collisionPairs");
    d_collisionPairs = nullptr;
  }
  collisionPairsCapacity = 0;

  if (d_sameMeshPairsCount) {
    BestEffortCudaFree(d_sameMeshPairsCount, "d_sameMeshPairsCount");
    d_sameMeshPairsCount = nullptr;
  }

  if (d_neighborPairHashes) {
    BestEffortCudaFree(d_neighborPairHashes, "d_neighborPairHashes");
    d_neighborPairHashes = nullptr;
  }

  if (d_collisionCounts) {
    BestEffortCudaFree(d_collisionCounts, "d_collisionCounts");
    d_collisionCounts = nullptr;
  }
  if (d_collisionOffsets) {
    BestEffortCudaFree(d_collisionOffsets, "d_collisionOffsets");
    d_collisionOffsets = nullptr;
  }
  collisionCountCapacity = 0;

  if (d_scanTempStorage) {
    BestEffortCudaFree(d_scanTempStorage, "d_scanTempStorage");
    d_scanTempStorage = nullptr;
  }
  scanTempStorageBytes = 0;

  if (d_nodes) {
    if (ownsNodes) {
      BestEffortCudaFree(d_nodes, "d_nodes");
    }
    d_nodes   = nullptr;
    ownsNodes = true;
  }

  if (d_elementMeshIds) {
    BestEffortCudaFree(d_elementMeshIds, "d_elementMeshIds");
    d_elementMeshIds = nullptr;
  }

  if (d_elements) {
    BestEffortCudaFree(d_elements, "d_elements");
    d_elements = nullptr;
  }

  if (d_bp) {
    BestEffortCudaFree(d_bp, "d_bp");
    d_bp = nullptr;
  }

  h_elementMeshIds.clear();
  h_collisionPairs.clear();
  h_aabbs.clear();
  h_neighborPairs.clear();
  aabbOptions = BroadphaseAABBOptions{};
  aabbsReady = false;
  sortedAABBsReady = false;
  sortedAxis = 0;
  tempStorageBytes = 0;
  enableSelfCollision = true;

  numObjects       = 0;
  numCollisions    = 0;
  numNeighborPairs = 0;
  n_nodes          = 0;
  n_elems          = 0;
}

void Broadphase::Initialize(const ANCFCPUUtils::MeshManager& mesh_manager) {
  const Eigen::MatrixXd& nodes    = mesh_manager.GetAllNodes();
  const Eigen::MatrixXi& elements = mesh_manager.GetAllElements();

  Eigen::VectorXi elementMeshIds(mesh_manager.GetTotalElements());
  for (int i = 0; i < mesh_manager.GetNumMeshes(); ++i) {
    const auto& instance = mesh_manager.GetMeshInstance(i);
    for (int e = 0; e < instance.num_elements; ++e) {
      elementMeshIds(instance.element_offset + e) = i;
    }
  }

  Initialize(nodes, elements, elementMeshIds);
}

// Update node positions on device without changing topology or neighbor data
void Broadphase::UpdateNodes(const Eigen::MatrixXd& nodes) {
  if (n_nodes == 0 || d_nodes == nullptr) {
    throw std::logic_error("Broadphase::UpdateNodes called before Initialize");
  }

  if (nodes.rows() != n_nodes || nodes.cols() != 3 || !nodes.allFinite()) {
    throw std::invalid_argument("Broadphase::UpdateNodes invalid coordinates");
  }

  aabbsReady = sortedAABBsReady = false;
  numCollisions = 0;
  h_collisionPairs.clear();

  // Update host copy (optional but keeps diagnostics consistent)
  h_nodes = nodes;

  // Copy updated positions to device; connectivity and neighbor map are reused
  CheckCuda(cudaMemcpy(d_nodes, nodes.data(), n_nodes * 3 * sizeof(double),
                          cudaMemcpyHostToDevice));
}

// Create/update AABBs from mesh data
void Broadphase::CreateAABB(bool copyToHost) {
  aabbsReady = sortedAABBsReady = false;
  numCollisions = 0;
  h_collisionPairs.clear();
  h_aabbs.clear();
  if (n_elems == 0 || d_nodes == nullptr || d_elements == nullptr) {
    throw std::logic_error("Broadphase::CreateAABB called before Initialize");
  }

  // Launch kernel to compute AABBs using the device copy of this struct
  int blockSize = 256;
  int gridSize  = (n_elems - 1) / blockSize + 1;
  CheckCuda(cudaMemset(d_invalidBounds, 0, sizeof(int)));
  computeAABBKernel<<<gridSize, blockSize>>>(d_bp, d_aabbs, n_elems,
                                            aabbOptions, d_invalidBounds);
  CheckCuda(cudaPeekAtLastError());
  int invalid = 0;
  CheckCuda(cudaMemcpy(&invalid, d_invalidBounds, sizeof(int),
                        cudaMemcpyDeviceToHost));
  if (invalid) {
    throw std::invalid_argument("Broadphase nonfinite bounds or invalid inflation");
  }

  if (copyToHost) {
    h_aabbs.resize(n_elems);
    CheckCuda(cudaMemcpy(h_aabbs.data(), d_aabbs, n_elems * sizeof(AABB),
                            cudaMemcpyDeviceToHost));
  }

  numObjects = n_elems;
  aabbsReady = true;

  if (verbose) {
    std::cout << "Created " << numObjects << " AABBs from mesh elements\n";
  }
}

void Broadphase::SetAABBOptions(const BroadphaseAABBOptions& options) {
  if (!std::isfinite(options.inflation) || options.inflation < 0.0) {
    throw std::invalid_argument("Broadphase inflation must be finite and nonnegative");
  }
  aabbOptions = options;
  aabbsReady = sortedAABBsReady = false;
  numCollisions = 0;
  h_collisionPairs.clear();
}

void Broadphase::SetDetectionLimits(const BroadphaseDetectionLimits& limits) {
  detectionLimits = limits;
}

size_t Broadphase::GetDetectionWorkspaceBytes() const {
  const size_t countArrays = static_cast<size_t>(d_collisionCounts != nullptr) +
                             static_cast<size_t>(d_collisionOffsets != nullptr);
  return countArrays * static_cast<size_t>(collisionCountCapacity) *
             sizeof(unsigned long long) +
         scanTempStorageBytes +
         static_cast<size_t>(collisionPairsCapacity) * sizeof(CollisionPair);
}

void Broadphase::RetrieveAABBandPrints() {
  if (n_elems == 0 || d_aabbs == nullptr) {
    std::cerr << "Error: No AABBs to print" << std::endl;
    return;
  }

  h_aabbs.resize(n_elems);
  CheckCuda(cudaMemcpy(h_aabbs.data(), d_aabbs, n_elems * sizeof(AABB),
                          cudaMemcpyDeviceToHost));

  std::cout << "\n========== AABB Results ==========\n" << std::endl;

  for (int elem_idx = 0; elem_idx < n_elems; elem_idx++) {
    std::cout << "Element " << elem_idx << ":" << std::endl;

    // Print all nodes of this element
    std::cout << "  Nodes:" << std::endl;
    for (int i = 0; i < nodesPerElement; i++) {
      int node_id = h_elements(elem_idx, i);
      std::cout << "    Node " << node_id << ": (" << h_nodes(node_id, 0)
                << ", " << h_nodes(node_id, 1) << ", " << h_nodes(node_id, 2)
                << ")" << std::endl;
    }

    // Print AABB for this element
    AABB aabb = h_aabbs[elem_idx];
    std::cout << "  AABB:" << std::endl;
    std::cout << "    Min: (" << aabb.min.x << ", " << aabb.min.y << ", "
              << aabb.min.z << ")" << std::endl;
    std::cout << "    Max: (" << aabb.max.x << ", " << aabb.max.y << ", "
              << aabb.max.z << ")" << std::endl;
    std::cout << "    ObjectId: " << aabb.objectId << std::endl;
    std::cout << std::endl;
  }

  std::cout << "========== End of AABB Results ==========\n" << std::endl;
}

void Broadphase::BindNodesDevicePtr(double* d_nodes_external) {
  if (n_nodes == 0 || d_bp == nullptr) {
    throw std::logic_error("Broadphase::BindNodesDevicePtr before Initialize");
  }
  if (d_nodes_external == nullptr) {
    throw std::invalid_argument("Broadphase::BindNodesDevicePtr null pointer");
  }
  // Binding our existing allocation must not free it or transfer ownership.
  if (d_nodes_external == d_nodes) {
    return;
  }
  cudaPointerAttributes attributes{};
  const auto pointerError = cudaPointerGetAttributes(&attributes, d_nodes_external);
  if (pointerError == cudaErrorInvalidValue) {
    cudaGetLastError();  // clear this rejected pointer-query error
    throw std::invalid_argument("Broadphase node binding is not a CUDA allocation");
  }
  CheckCuda(pointerError);
  if (attributes.type != cudaMemoryTypeDevice &&
      attributes.type != cudaMemoryTypeManaged) {
    throw std::invalid_argument("Broadphase node binding requires device/managed memory");
  }

  if (d_nodes && ownsNodes) {
    CheckCuda(cudaFree(d_nodes));
  }
  d_nodes   = d_nodes_external;
  ownsNodes = false;
  aabbsReady = sortedAABBsReady = false;
  numCollisions = 0;
  h_collisionPairs.clear();

  CheckCuda(
      cudaMemcpy(d_bp, this, sizeof(Broadphase), cudaMemcpyHostToDevice));
}

// Sort AABBs along specified axis
void Broadphase::SortAABBs(int axis) {
  if (numObjects == 0)
    return;
  if (!aabbsReady) {
    throw std::logic_error("Broadphase::SortAABBs requires current AABBs");
  }
  sortedAABBsReady = false;
  numCollisions = 0;
  h_collisionPairs.clear();

  axis = std::min(std::max(axis, 0), 2);  // Clamp to [0, 2]

  // Extract sort keys and initialize indices
  int blockSize = 256;
  int gridSize  = (numObjects - 1) / blockSize + 1;
  extractSortKeysKernel<<<gridSize, blockSize>>>(
      d_aabbs, d_sortKeys, d_sortIndices, axis, numObjects);
  CheckCuda(cudaPeekAtLastError());

  // Sort using CUB (separate input and output buffers)
  CheckCuda(cub::DeviceRadixSort::SortPairs(
      d_tempStorage, tempStorageBytes, d_sortKeys, d_sortedKeys, d_sortIndices,
      d_sortedIndices, numObjects));

  // Reorder AABBs based on sorted indices
  reorderAABBsKernel<<<gridSize, blockSize>>>(d_aabbs, d_sortedAABBs,
                                              d_sortedIndices, numObjects);
  CheckCuda(cudaPeekAtLastError());
  sortedAxis = axis;
  sortedAABBsReady = true;

  if (verbose) {
    std::cout << "Sorted " << numObjects << " AABBs along axis " << axis
              << "\n";
  }
}

// Print sorted AABBs for verification
void Broadphase::PrintSortedAABBs(int axis) {
  if (numObjects == 0) {
    std::cerr << "Error: No sorted AABBs to print" << std::endl;
    return;
  }

  axis = std::min(std::max(axis, 0), 2);

  // Copy sorted AABBs and indices to host
  std::vector<AABB> h_sortedAABBs(numObjects);
  std::vector<double> h_sortedKeys(numObjects);
  std::vector<int> h_sortedIndices(numObjects);

  CheckCuda(cudaMemcpy(h_sortedAABBs.data(), d_sortedAABBs,
                          numObjects * sizeof(AABB), cudaMemcpyDeviceToHost));
  CheckCuda(cudaMemcpy(h_sortedKeys.data(), d_sortedKeys,
                          numObjects * sizeof(double), cudaMemcpyDeviceToHost));
  CheckCuda(cudaMemcpy(h_sortedIndices.data(), d_sortedIndices,
                          numObjects * sizeof(int), cudaMemcpyDeviceToHost));

  const char* axis_names[] = {"X", "Y", "Z"};
  std::cout << "\n========== Sorted AABBs (Axis: " << axis_names[axis]
            << ") ==========\n"
            << std::endl;

  for (int i = 0; i < numObjects; i++) {
    AABB aabb = h_sortedAABBs[i];
    std::cout << "Sorted Index " << i << " (Original Element "
              << h_sortedIndices[i] << "):" << std::endl;
    std::cout << "  Sort Key: " << h_sortedKeys[i] << std::endl;
    std::cout << "  AABB Min: (" << aabb.min.x << ", " << aabb.min.y << ", "
              << aabb.min.z << ")" << std::endl;
    std::cout << "  AABB Max: (" << aabb.max.x << ", " << aabb.max.y << ", "
              << aabb.max.z << ")" << std::endl;
    std::cout << "  ObjectId: " << aabb.objectId << std::endl;

    // Verify sort key matches AABB min value
    double expected_key = (axis == 0)   ? aabb.min.x
                          : (axis == 1) ? aabb.min.y
                                        : aabb.min.z;
    if (fabs(expected_key - h_sortedKeys[i]) > 1e-12) {
      std::cout << "  WARNING: Sort key mismatch. Expected " << expected_key
                << " but got " << h_sortedKeys[i] << std::endl;
    }

    // Check if sorted correctly (should be in ascending order)
    if (i > 0 && h_sortedKeys[i] < h_sortedKeys[i - 1]) {
      std::cout << "  ERROR: Sort order violated. Key " << h_sortedKeys[i]
                << " < previous key " << h_sortedKeys[i - 1] << std::endl;
    }

    std::cout << std::endl;
  }

  std::cout << "========== End of Sorted AABBs ==========\n" << std::endl;

  // Summary check
  bool is_sorted = true;
  for (int i = 1; i < numObjects; i++) {
    if (h_sortedKeys[i] < h_sortedKeys[i - 1]) {
      is_sorted = false;
      break;
    }
  }

  if (is_sorted) {
    std::cout << "Sorting verification PASSED: AABBs are correctly sorted"
              << std::endl;
  } else {
    std::cout << "Sorting verification FAILED: AABBs are NOT correctly "
                 "sorted"
              << std::endl;
  }
}

// Build neighbor connectivity map (CPU).
// Optimized using a node-to-element map:
//   - Previous approach: O(n_elems^2) all-pairs element neighbor search.
//   - Current approach: O(n_nodes * avg_elements_per_node^2),
//     since we only compare elements that share a node.
// For typical tetrahedral meshes where each node belongs to few elements,
// this yields a substantial reduction in work compared to the naive method.
void Broadphase::BuildNeighborMap() {
  if (n_nodes == 0 || n_elems == 0) {
    throw std::logic_error("Broadphase::BuildNeighborMap called before Initialize");
  }
  numCollisions = 0;
  h_collisionPairs.clear();
  h_neighborPairs.clear();

  std::cout << "Building neighbor map..." << std::endl;

  // Step 1: Build node-to-element map (which elements contain each node)
  std::vector<std::vector<int>> nodeToElements(n_nodes);
  for (int elem = 0; elem < n_elems; elem++) {
    for (int i = 0; i < nodesPerElement; i++) {
      int nodeId = h_elements(elem, i);
      nodeToElements[nodeId].push_back(elem);
    }
  }

  // Step 2: For each node, all elements sharing that node are neighbors
  for (int nodeId = 0; nodeId < n_nodes; nodeId++) {
    const std::vector<int>& elems = nodeToElements[nodeId];
    int numElems                  = elems.size();

    // All pairs of elements sharing this node are neighbors
    for (int i = 0; i < numElems; i++) {
      for (int j = i + 1; j < numElems; j++) {
        int elemA = elems[i];
        int elemB = elems[j];
        if (elemA == elemB)
          continue;
        // Ensure consistent ordering (smaller id first)
        if (elemA > elemB)
          std::swap(elemA, elemB);
        h_neighborPairs.insert({elemA, elemB});
      }
    }
  }

  std::cout << "Found " << h_neighborPairs.size() << " neighbor pairs"
            << std::endl;

  // Convert to sorted array of hashes for GPU binary search
  std::vector<long long> hashes;
  hashes.reserve(h_neighborPairs.size());

  for (const auto& pair : h_neighborPairs) {
    // Hash: combine two integers into one long long
    long long hash = ((long long)pair.first << 32) | pair.second;
    hashes.push_back(hash);
  }

  std::sort(hashes.begin(), hashes.end());

  // Copy to device
  if (hashes.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::length_error("Broadphase neighbor count exceeds int indexing");
  }
  numNeighborPairs = 0;
  if (d_neighborPairHashes) {
    CheckCuda(cudaFree(d_neighborPairHashes));
    d_neighborPairHashes = nullptr;
  }

  if (!hashes.empty()) {
    CheckCuda(cudaMalloc(&d_neighborPairHashes,
                            hashes.size() * sizeof(long long)));
    CheckCuda(cudaMemcpy(d_neighborPairHashes, hashes.data(),
                            hashes.size() * sizeof(long long),
                            cudaMemcpyHostToDevice));
    numNeighborPairs = static_cast<int>(hashes.size());
  }

  std::cout << "Neighbor map uploaded to GPU (" << numNeighborPairs << " pairs)"
            << std::endl;
}

// Detect collisions using two-pass sweep and prune (with neighbor filtering).
// Count in 64 bits, validate the COMPLETE result, then allocate and fill it.
void Broadphase::DetectCollisions(bool copyPairsToHost) {
  numCollisions = 0;
  h_collisionPairs.clear();
  if (numObjects == 0)
    return;
  if (!sortedAABBsReady) {
    throw std::logic_error("Broadphase::DetectCollisions requires sorted current AABBs");
  }

  const size_t limit = detectionLimits.maxWorkspaceBytes;
  const size_t countBytes = static_cast<size_t>(numObjects + 1) *
                            sizeof(unsigned long long);
  // A reduced budget also bounds retained allocations. These buffers can all
  // be rebuilt from the sorted AABBs after a failed or budget-limited attempt.
  if (GetDetectionWorkspaceBytes() > limit) {
    CheckCuda(cudaFree(d_collisionCounts));
    d_collisionCounts = nullptr;
    CheckCuda(cudaFree(d_collisionOffsets));
    d_collisionOffsets = nullptr;
    collisionCountCapacity = 0;
    CheckCuda(cudaFree(d_scanTempStorage));
    d_scanTempStorage = nullptr;
    scanTempStorageBytes = 0;
    CheckCuda(cudaFree(d_collisionPairs));
    d_collisionPairs = nullptr;
    collisionPairsCapacity = 0;
  }
  CheckWorkspace(2 * countBytes, scanTempStorageBytes,
                 static_cast<size_t>(collisionPairsCapacity) * sizeof(CollisionPair),
                 limit);

  if (d_collisionCounts == nullptr || d_collisionOffsets == nullptr ||
      collisionCountCapacity < numObjects + 1) {
    CheckCuda(cudaFree(d_collisionCounts));
    d_collisionCounts = nullptr;
    CheckCuda(cudaFree(d_collisionOffsets));
    d_collisionOffsets = nullptr;
    collisionCountCapacity = numObjects + 1;
    CheckCuda(cudaMalloc(&d_collisionCounts, countBytes));
    CheckCuda(cudaMalloc(&d_collisionOffsets, countBytes));
  }

  const int blockSize = 256;
  const int gridSize = (numObjects - 1) / blockSize + 1;
  countCollisionsKernel<<<gridSize, blockSize>>>(
      d_sortedAABBs, d_collisionCounts, numObjects, d_neighborPairHashes,
      numNeighborPairs, d_elementMeshIds, enableSelfCollision ? 1 : 0, sortedAxis);
  CheckCuda(cudaPeekAtLastError());
  CheckCuda(cudaMemset(&d_collisionCounts[numObjects], 0,
                        sizeof(unsigned long long)));

  size_t requiredScanBytes = 0;
  CheckCuda(cub::DeviceScan::ExclusiveSum(
      nullptr, requiredScanBytes, d_collisionCounts, d_collisionOffsets,
      numObjects + 1));
  CheckWorkspace(2 * countBytes, requiredScanBytes,
                 static_cast<size_t>(collisionPairsCapacity) * sizeof(CollisionPair),
                 limit);
  if (d_scanTempStorage == nullptr || scanTempStorageBytes < requiredScanBytes) {
    CheckCuda(cudaFree(d_scanTempStorage));
    d_scanTempStorage = nullptr;
    scanTempStorageBytes = 0;
    CheckCuda(cudaMalloc(&d_scanTempStorage, requiredScanBytes));
    scanTempStorageBytes = requiredScanBytes;
  }
  CheckCuda(cub::DeviceScan::ExclusiveSum(
      d_scanTempStorage, scanTempStorageBytes, d_collisionCounts,
      d_collisionOffsets, numObjects + 1));

  unsigned long long total = 0;
  CheckCuda(cudaMemcpy(&total, &d_collisionOffsets[numObjects], sizeof(total),
                        cudaMemcpyDeviceToHost));
  if (total > static_cast<unsigned long long>(std::numeric_limits<int>::max())) {
    throw std::length_error("Broadphase pair count exceeds int-sized consumer interface");
  }
  if (total > detectionLimits.maxPairs) {
    throw std::length_error("Broadphase pair budget exceeded; no pairs published");
  }
  if (total > std::numeric_limits<size_t>::max() / sizeof(CollisionPair)) {
    throw std::length_error("Broadphase pair allocation size overflow");
  }
  if (total == 0)
    return;

  const size_t pairBytes = static_cast<size_t>(total) * sizeof(CollisionPair);
  const size_t retainedPairBytes =
      static_cast<size_t>(collisionPairsCapacity) * sizeof(CollisionPair);
  CheckWorkspace(2 * countBytes, scanTempStorageBytes,
                 std::max(pairBytes, retainedPairBytes), limit);
  if (d_collisionPairs == nullptr ||
      static_cast<unsigned long long>(collisionPairsCapacity) < total) {
    CheckCuda(cudaFree(d_collisionPairs));
    d_collisionPairs = nullptr;
    collisionPairsCapacity = 0;
    CheckCuda(cudaMalloc(&d_collisionPairs, pairBytes));
    collisionPairsCapacity = static_cast<int>(total);
  }

  generateCollisionPairsKernel<<<gridSize, blockSize>>>(
      d_sortedAABBs, d_collisionOffsets, d_collisionPairs, numObjects,
      d_neighborPairHashes, numNeighborPairs, d_elementMeshIds,
      enableSelfCollision ? 1 : 0, sortedAxis);
  CheckCuda(cudaPeekAtLastError());

  if (copyPairsToHost) {
    // Publish only after a successful copy, including allocation failure.
    std::vector<CollisionPair> result(static_cast<size_t>(total));
    CheckCuda(cudaMemcpy(result.data(), d_collisionPairs, pairBytes,
                          cudaMemcpyDeviceToHost));
    h_collisionPairs.swap(result);
  }
  numCollisions = static_cast<int>(total);
  if (verbose) {
    std::cout << "Detected " << numCollisions
              << " collision pairs (neighbors filtered)\n";
  }
}

int Broadphase::CountSameMeshPairsDevice() const {
  if (numCollisions == 0 || d_collisionPairs == nullptr ||
      d_elementMeshIds == nullptr || d_sameMeshPairsCount == nullptr) {
    return 0;
  }

  CheckCuda(cudaMemset(d_sameMeshPairsCount, 0, sizeof(int)));

  int blockSize = 256;
  int gridSize  = (numCollisions - 1) / blockSize + 1;

  countSameMeshPairsKernel<<<gridSize, blockSize>>>(
      d_collisionPairs, numCollisions, d_elementMeshIds, d_sameMeshPairsCount);
  CheckCuda(cudaPeekAtLastError());
  CheckCuda(cudaDeviceSynchronize());

  int count = 0;
  CheckCuda(cudaMemcpy(&count, d_sameMeshPairsCount, sizeof(int),
                          cudaMemcpyDeviceToHost));
  return count;
}

// Print collision pairs
void Broadphase::PrintCollisionPairs() {
  std::cout << "\n========== Collision Pairs (Non-Neighbors) ==========\n"
            << std::endl;

  // for (int i = 0; i < numCollisions; i++) {
  //   std::cout << "Pair " << i << ": Element " << h_collisionPairs[i].idA
  //             << " <-> Element " << h_collisionPairs[i].idB << std::endl;
  // }

  std::cout << "Total non-neighbor collisions: " << numCollisions << std::endl;

  std::cout << "\n========== End of Collision Pairs ==========\n" << std::endl;
}
