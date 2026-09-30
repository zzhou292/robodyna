// Small GPU broadphase gates. No external meshes, dynamics, or large allocation.
// The CPU oracle enumerates all unordered element pairs independently of SAP.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lib_src/collision/HydroelasticBroadphase.cuh"

namespace {
using Box = std::array<double, 6>;  // xmin,ymin,zmin,xmax,ymax,zmax
using Pairs = std::set<std::pair<int, int>>;

struct Scene {
  Eigen::MatrixXd nodes;
  Eigen::MatrixXi elements;
  Eigen::VectorXi bodies;
};

Scene TrianglesFromBoxes(const std::vector<Box>& boxes) {
  Scene s;
  s.nodes.resize(3 * boxes.size(), 3);
  s.elements.resize(boxes.size(), 3);
  s.bodies.resize(boxes.size());
  for (int e = 0; e < static_cast<int>(boxes.size()); ++e) {
    const auto& b = boxes[e];
    s.nodes.row(3 * e) << b[0], b[1], b[2];
    s.nodes.row(3 * e + 1) << b[3], b[4], b[2];
    s.nodes.row(3 * e + 2) << b[0], b[1], b[5];
    s.elements.row(e) << 3 * e, 3 * e + 1, 3 * e + 2;
    s.bodies[e] = e;
  }
  return s;
}

Pairs ExhaustiveOracle(const Scene& s, bool selfCollision = true,
                       bool filterNeighbors = false, double inflation = 0.0,
                       const Eigen::MatrixXd* end = nullptr,
                       const std::vector<double>* localInflation = nullptr) {
  std::vector<Box> bounds(s.elements.rows());
  for (int e = 0; e < s.elements.rows(); ++e) {
    const double radius = inflation + (localInflation ? (*localInflation)[e] : 0.0);
    for (int axis = 0; axis < 3; ++axis) {
      double lo = std::numeric_limits<double>::infinity();
      double hi = -lo;
      for (int k = 0; k < s.elements.cols(); ++k) {
        const int node = s.elements(e, k);
        lo = std::min(lo, s.nodes(node, axis));
        hi = std::max(hi, s.nodes(node, axis));
        if (end) {
          lo = std::min(lo, (*end)(node, axis));
          hi = std::max(hi, (*end)(node, axis));
        }
      }
      bounds[e][axis] = lo - radius;
      bounds[e][axis + 3] = hi + radius;
    }
  }
  Pairs out;
  for (int a = 0; a < s.elements.rows(); ++a) {
    for (int b = a + 1; b < s.elements.rows(); ++b) {
      if (!selfCollision && s.bodies[a] == s.bodies[b])
        continue;
      bool neighbors = false;
      if (filterNeighbors) {
        for (int i = 0; i < s.elements.cols(); ++i)
          for (int j = 0; j < s.elements.cols(); ++j)
            neighbors = neighbors || s.elements(a, i) == s.elements(b, j);
      }
      if (neighbors)
        continue;
      bool overlap = true;
      for (int axis = 0; axis < 3; ++axis) {
        overlap = overlap && bounds[a][axis] <= bounds[b][axis + 3] &&
                  bounds[b][axis] <= bounds[a][axis + 3];
      }
      if (overlap)
        out.emplace(a, b);
    }
  }
  return out;
}

Pairs HostPairs(const Broadphase& bp) {
  Pairs pairs;
  for (const auto& p : bp.h_collisionPairs) {
    EXPECT_NE(p.idA, p.idB);
    pairs.emplace(std::min(p.idA, p.idB), std::max(p.idA, p.idB));
  }
  EXPECT_EQ(pairs.size(), bp.h_collisionPairs.size()) << "Duplicate candidates";
  EXPECT_EQ(pairs.size(), static_cast<size_t>(bp.numCollisions));
  return pairs;
}

struct DeviceDoubles {
  explicit DeviceDoubles(const double* src, size_t count) {
    if (cudaMalloc(&data, count * sizeof(double)) != cudaSuccess)
      throw std::runtime_error("Test device allocation failed");
    if (cudaMemcpy(data, src, count * sizeof(double), cudaMemcpyHostToDevice) !=
        cudaSuccess) {
      cudaFree(data);
      throw std::runtime_error("Test device upload failed");
    }
  }
  ~DeviceDoubles() { cudaFree(data); }
  DeviceDoubles(const DeviceDoubles&) = delete;
  DeviceDoubles& operator=(const DeviceDoubles&) = delete;
  double* data = nullptr;
};

class SurfaceBroadphase : public ::testing::Test {
 protected:
  void SetUp() override {
    int count = 0;
    ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess)
        << "This gate requires actual CUDA execution; it must not silently skip";
    ASSERT_GT(count, 0);
  }
  void CheckAxes(Broadphase& bp, const Pairs& expected) {
    bp.CreateAABB();
    for (int axis = 0; axis < 3; ++axis) {
      SCOPED_TRACE(axis);
      bp.SortAABBs(axis);
      bp.DetectCollisions(true);
      EXPECT_EQ(HostPairs(bp), expected);
    }
  }
};

TEST_F(SurfaceBroadphase, AllAxesMatchExhaustiveOracle) {
  // Along Y/Z, the remote X box occurs between two intersecting boxes. The
  // previous unconditional X early-break missed the intersecting pair.
  std::vector<Box> boxes = {{0, 0, 0, 1, 3, 3}, {10, 1, 1, 11, 2, 2},
                           {0, 2, 2, 1, 3, 3}, {1, 0, 0, 2, 1, 1}};
  std::mt19937 rng(720);
  std::uniform_real_distribution<double> position(-2.0, 2.0);
  for (int i = 0; i < 32; ++i) {
    const double x = position(rng), y = position(rng), z = position(rng);
    boxes.push_back({x, y, z, x + 0.75, y + 0.75, z + 0.75});
  }
  auto s = TrianglesFromBoxes(boxes);
  Broadphase bp;
  bp.Initialize(s.nodes, s.elements, s.bodies);
  const auto expected = ExhaustiveOracle(s);
  ASSERT_EQ(expected.count({0, 2}), 1u);
  CheckAxes(bp, expected);
}

TEST_F(SurfaceBroadphase, TouchingAndPerElementThicknessCanBeReset) {
  auto s = TrianglesFromBoxes({{0, 0, 0, 1, 1, 0},
                               {0, 0, 0.25, 1, 1, 0.25},
                               {0, 0, 0.5, 1, 1, 0.5}});
  std::vector<double> thickness = {0.0, 0.25, 0.0};
  DeviceDoubles local(thickness.data(), thickness.size());
  Broadphase bp;
  bp.Initialize(s.nodes, s.elements, s.bodies);
  CheckAxes(bp, {});
  BroadphaseAABBOptions options;
  options.inflation = 0.125;
  bp.SetAABBOptions(options);
  const auto touching = ExhaustiveOracle(s, true, false, options.inflation);
  ASSERT_EQ(touching.size(), 2u);
  CheckAxes(bp, touching);
  options.inflation = 0.0;
  options.d_elementInflation = local.data;
  bp.SetAABBOptions(options);
  CheckAxes(bp, ExhaustiveOracle(s, true, false, 0.0, nullptr, &thickness));
  bp.SetAABBOptions({});
  CheckAxes(bp, {});
}

TEST_F(SurfaceBroadphase, LinearSweptBoundsCatchEndpointInvisibleCrossing) {
  auto s = TrianglesFromBoxes({{-1, 0, 0, -1, 1, 1}, {0, 0, 0, 0, 1, 1}});
  Eigen::MatrixXd end = s.nodes;
  end.topRows(3).col(0).array() += 2.0;
  DeviceDoubles deviceEnd(end.data(), end.size());
  Broadphase bp;
  bp.Initialize(s.nodes, s.elements, s.bodies);
  CheckAxes(bp, {});
  BroadphaseAABBOptions options;
  options.d_endNodes = deviceEnd.data;
  bp.SetAABBOptions(options);
  auto swept = ExhaustiveOracle(s, true, false, 0.0, &end);
  ASSERT_EQ(swept.size(), 1u);
  CheckAxes(bp, swept);
  bp.SetAABBOptions({});
  bp.UpdateNodes(end);
  CheckAxes(bp, {});
}

TEST_F(SurfaceBroadphase, SameBodyFilteringAndSharedNodeNeighbors) {
  auto s = TrianglesFromBoxes(std::vector<Box>(4, {0, 0, 0, 1, 1, 1}));
  s.elements(1, 0) = s.elements(0, 0);
  s.bodies << 0, 0, 0, 1;
  Broadphase bp;
  bp.Initialize(s.nodes, s.elements, s.bodies);
  bp.BuildNeighborMap();
  ASSERT_EQ(bp.numNeighborPairs, 1);
  auto sameBody = ExhaustiveOracle(s, true, true);
  ASSERT_EQ(sameBody.size(), 5u);
  CheckAxes(bp, sameBody);
  EXPECT_EQ(bp.CountSameMeshPairsDevice(), 2);
  bp.EnableSelfCollision(false);
  EXPECT_EQ(bp.numCollisions, 0);
  EXPECT_TRUE(bp.h_collisionPairs.empty());
  EXPECT_EQ(bp.GetCollisionPairsDevicePtr(), nullptr);
  auto otherBodies = ExhaustiveOracle(s, false, true);
  ASSERT_EQ(otherBodies.size(), 3u);
  CheckAxes(bp, otherBodies);
  EXPECT_EQ(bp.CountSameMeshPairsDevice(), 0);
  bp.EnableSelfCollision(true);
  CheckAxes(bp, sameBody);
}

TEST_F(SurfaceBroadphase, PairAndMemoryBudgetFailuresPublishNothingAndRetry) {
  auto s = TrianglesFromBoxes(std::vector<Box>(4, {0, 0, 0, 1, 1, 1}));
  Broadphase bp;
  bp.Initialize(s.nodes, s.elements, s.bodies);
  const auto expected = ExhaustiveOracle(s);
  ASSERT_EQ(expected.size(), 6u);
  CheckAxes(bp, expected);
  const size_t successfulBytes = bp.GetDetectionWorkspaceBytes();
  BroadphaseDetectionLimits limits;
  limits.maxPairs = 5;
  bp.SetDetectionLimits(limits);
  EXPECT_THROW(bp.DetectCollisions(true), std::length_error);
  EXPECT_EQ(bp.numCollisions, 0);
  EXPECT_TRUE(bp.h_collisionPairs.empty());
  EXPECT_EQ(bp.GetCollisionPairsDevicePtr(), nullptr);
  limits.maxPairs = 6;
  bp.SetDetectionLimits(limits);
  bp.DetectCollisions(true);
  EXPECT_EQ(HostPairs(bp), expected);
  limits.maxWorkspaceBytes = 1;
  bp.SetDetectionLimits(limits);
  EXPECT_THROW(bp.DetectCollisions(true), std::length_error);
  EXPECT_EQ(bp.numCollisions, 0);
  EXPECT_TRUE(bp.h_collisionPairs.empty());
  EXPECT_LE(bp.GetDetectionWorkspaceBytes(), limits.maxWorkspaceBytes);
  limits.maxWorkspaceBytes = successfulBytes;
  bp.SetDetectionLimits(limits);
  bp.DetectCollisions(true);
  EXPECT_EQ(HostPairs(bp), expected);
  EXPECT_LE(bp.GetDetectionWorkspaceBytes(), limits.maxWorkspaceBytes);
}

TEST_F(SurfaceBroadphase, InvalidStateAndBorrowedBindingPreserveUsableData) {
  auto s = TrianglesFromBoxes(std::vector<Box>(2, {0, 0, 0, 1, 1, 1}));
  DeviceDoubles external(s.nodes.data(), s.nodes.size());
  Broadphase bp;
  EXPECT_THROW(bp.CreateAABB(), std::logic_error);
  EXPECT_THROW(bp.BindNodesDevicePtr(nullptr), std::logic_error);
  bp.Initialize(s.nodes, s.elements, s.bodies);
  EXPECT_THROW(bp.BindNodesDevicePtr(nullptr), std::invalid_argument);
  EXPECT_THROW(bp.BindNodesDevicePtr(s.nodes.data()), std::invalid_argument);
  EXPECT_THROW(bp.DetectCollisions(), std::logic_error);
  double* original = bp.d_nodes;
  bp.BindNodesDevicePtr(original);
  EXPECT_TRUE(bp.ownsNodes);
  bp.BindNodesDevicePtr(external.data);
  EXPECT_FALSE(bp.ownsNodes);
  const auto expected = ExhaustiveOracle(s);
  CheckAxes(bp, expected);
  auto bad = s.elements;
  bad(0, 0) = -1;
  EXPECT_THROW(bp.Initialize(s.nodes, bad), std::invalid_argument);
  CheckAxes(bp, expected);
  // Reinitialization frees only owned resources and resets borrowed options.
  bp.Initialize(s.nodes, s.elements, s.bodies);
  EXPECT_TRUE(bp.ownsNodes);
  CheckAxes(bp, expected);
  bp.BuildNeighborMap();
  bp.BuildNeighborMap();  // empty map remains a safe null device allocation
  CheckAxes(bp, expected);
}

TEST_F(SurfaceBroadphase, InvalidDeviceCoordinatesAndInflationFailThenRecover) {
  auto s = TrianglesFromBoxes(std::vector<Box>(2, {0, 0, 0, 1, 1, 1}));
  Eigen::MatrixXd badEnd = s.nodes;
  badEnd(0, 0) = std::numeric_limits<double>::quiet_NaN();
  DeviceDoubles deviceEnd(badEnd.data(), badEnd.size());
  std::vector<double> invalidInflation = {0.0, -1.0};
  DeviceDoubles deviceInflation(invalidInflation.data(), invalidInflation.size());
  Broadphase bp;
  bp.Initialize(s.nodes, s.elements, s.bodies);
  BroadphaseAABBOptions options;
  options.inflation = -1.0;
  EXPECT_THROW(bp.SetAABBOptions(options), std::invalid_argument);
  options.inflation = std::numeric_limits<double>::infinity();
  EXPECT_THROW(bp.SetAABBOptions(options), std::invalid_argument);
  options.inflation = 0.0;
  options.d_endNodes = deviceEnd.data;
  bp.SetAABBOptions(options);
  EXPECT_THROW(bp.CreateAABB(), std::invalid_argument);
  EXPECT_THROW(bp.SortAABBs(), std::logic_error);
  EXPECT_EQ(bp.numCollisions, 0);
  options.d_endNodes = nullptr;
  options.d_elementInflation = deviceInflation.data;
  bp.SetAABBOptions(options);
  EXPECT_THROW(bp.CreateAABB(), std::invalid_argument);
  bp.SetAABBOptions({});
  CheckAxes(bp, ExhaustiveOracle(s));
}
}  // namespace
