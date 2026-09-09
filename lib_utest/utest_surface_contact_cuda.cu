#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cmath>

#include "lib_src/collision/SurfaceContactLaw.h"
#include "lib_src/collision/SurfaceContactGeometry.h"

namespace {
namespace sc = tlfea::contact;

template <class T>
struct DeviceValue {
  DeviceValue() : allocation(cudaMalloc(&data, sizeof(T))) {}
  ~DeviceValue() { if (data) cudaFree(data); }
  DeviceValue(const DeviceValue&) = delete;
  DeviceValue& operator=(const DeviceValue&) = delete;
  T* data = nullptr;
  cudaError_t allocation;
};

struct NormalResults {
  sc::Status statuses[4];
  sc::NormalContactResponse values[4];
};

__global__ void NormalFixture(NormalResults* out) {
  const int i = threadIdx.x;
  if (i >= 4) return;
  const sc::NormalContactParameters parameters{1000, 0.25, 0.8};
  const sc::NormalContactInput inputs[4] = {
      {-0.01, -2, 0.25}, {-0.01, 100, 0.25},
      {0.01, -2, 0.25}, {-0.01, -2, -1}};
  out->statuses[i] = sc::EvaluateNormalContact(parameters, inputs[i], &out->values[i]);
}

struct ProjectionResults {
  sc::Status evaluate_status;
  sc::Status project_status;
  sc::LinearPointKinematics point;
  sc::TriangleNodalForces nodal;
};

__global__ void ProjectionFixture(ProjectionResults* out) {
  // Exercise distinct SoA position and AoS velocity storage on the device.
  const double xyz[9] = {1, 3, 1, 2, 2, 5, -1, -1, -1};
  const double velocity[9] = {1, 2, 4, -2, 5, 1, 7, 3, -2};
  const double inverse_mass[3] = {0.5, 0.25, 0.125};
  const sc::SurfaceTriangle triangle{{0, 1, 2}, 73, 42, 2, 0.0005,
                                     sc::SurfaceInterpolation::kLinearTriangle};
  const sc::LinearTriangleSurfaceView view{
      {xyz, 3, 1, 3}, {velocity, 3, 3, 1}, inverse_mass, &triangle, 1};
  const sc::LinearTrianglePoint point{0, {0.2, 0.3, 0.5}};
  out->evaluate_status = sc::EvaluateLinearPoint(view, point, &out->point);
  out->project_status = sc::ProjectLinearPointForce(view, point, {3, -4, 9}, &out->nodal);
}

struct TrialResults {
  sc::Status make_status;
  sc::Status discard_commit_status;
  sc::Status retry_status;
  sc::Status commit_status;
  sc::Status second_commit_status;
  sc::NormalContactState after_discard;
  sc::NormalContactState after_commit;
};

__global__ void TrialFixture(TrialResults* out) {
  sc::NormalContactState committed;
  sc::NormalContactTrial trial;
  sc::NormalContactResponse response;
  response.dissipated_power = 20;
  out->make_status = sc::MakeNormalContactTrial(committed, response, 0.01, &trial);
  sc::DiscardNormalContactTrial(&trial);
  out->discard_commit_status = sc::CommitNormalContactTrial(&committed, &trial);
  out->after_discard = committed;
  out->retry_status = sc::MakeNormalContactTrial(committed, response, 0.005, &trial);
  out->commit_status = sc::CommitNormalContactTrial(&committed, &trial);
  out->second_commit_status = sc::CommitNormalContactTrial(&committed, &trial);
  out->after_commit = committed;
}

struct GeometryResults {
  sc::Status statuses[4];
  sc::TrianglePointGeometry triangles[2];
  sc::SegmentPairGeometry edges[2];
};

__global__ void GeometryFixture(GeometryResults* out) {
  const sc::TriangleGeometry triangles[2] = {
      {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, {10, 11, 12}, 80},
      {{{0, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {10, 12, 13}, 81}};
  for (int i = 0; i < 2; ++i) {
    out->statuses[i] = sc::ClosestPointOnTriangle(
        {0.5, 0.5, 0.25}, triangles[i], &out->triangles[i]);
  }
  const sc::SegmentGeometry a{{{-1, 0, 0}, {1, 0, 0}}, {20, 21}};
  const sc::SegmentGeometry b{{{0, -1, 0.5}, {0, 1, 0.5}}, {30, 31}};
  const sc::SegmentGeometry parallel{{{-1, 1, 0}, {1, 1, 0}}, {40, 41}};
  out->statuses[2] = sc::ClosestPointsBetweenSegments(a, b, &out->edges[0]);
  out->statuses[3] = sc::ClosestPointsBetweenSegments(a, parallel, &out->edges[1]);
}

class SurfaceContactCuda : public ::testing::Test {
 protected:
  void SetUp() override {
    int devices = 0;
    ASSERT_EQ(cudaGetDeviceCount(&devices), cudaSuccess);
    ASSERT_GT(devices, 0) << "GPU execution is required for this gate";
  }
};

TEST_F(SurfaceContactCuda, NormalLawMatchesAnalyticValuesOnDevice) {
  DeviceValue<NormalResults> device;
  ASSERT_EQ(device.allocation, cudaSuccess);
  NormalFixture<<<1, 4>>>(device.data);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  NormalResults result;
  ASSERT_EQ(cudaMemcpy(&result, device.data, sizeof(result), cudaMemcpyDeviceToHost), cudaSuccess);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(result.statuses[i], sc::Status::kOk);
  EXPECT_NEAR(result.values[0].force, 10 + std::sqrt(4000.0), 1e-11);
  EXPECT_NEAR(result.values[0].elastic_energy, 0.05, 1e-14);
  EXPECT_NEAR(result.values[0].dissipated_power, 2 * std::sqrt(4000.0), 1e-11);
  EXPECT_GT(result.values[0].stable_timestep, 0);
  EXPECT_DOUBLE_EQ(result.values[1].force, 0);
  EXPECT_GE(result.values[1].dissipated_power, 0);
  EXPECT_DOUBLE_EQ(result.values[2].force, 0);
  EXPECT_FALSE(result.values[2].active);
  EXPECT_EQ(result.statuses[3], sc::Status::kInvalidArgument);
  EXPECT_DOUBLE_EQ(result.values[3].force, 0);
}

TEST_F(SurfaceContactCuda, ProjectionPreservesMomentAndWorkOnDevice) {
  DeviceValue<ProjectionResults> device;
  ASSERT_EQ(device.allocation, cudaSuccess);
  ProjectionFixture<<<1, 1>>>(device.data);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ProjectionResults result;
  ASSERT_EQ(cudaMemcpy(&result, device.data, sizeof(result), cudaMemcpyDeviceToHost), cudaSuccess);
  ASSERT_EQ(result.evaluate_status, sc::Status::kOk);
  ASSERT_EQ(result.project_status, sc::Status::kOk);
  EXPECT_NEAR(result.point.position.x, 1.6, 1e-13);
  EXPECT_NEAR(result.point.position.y, 3.5, 1e-13);
  EXPECT_NEAR(result.point.velocity.x, 3.1, 1e-13);
  EXPECT_NEAR(result.point.velocity.y, 3.4, 1e-13);
  EXPECT_NEAR(result.point.velocity.z, 0.1, 1e-13);
  EXPECT_NEAR(result.point.inverse_effective_mass, 0.07375, 1e-14);
  const sc::Vec3 positions[3] = {{1, 2, -1}, {3, 2, -1}, {1, 5, -1}};
  const sc::Vec3 velocities[3] = {{1, 2, 4}, {-2, 5, 1}, {7, 3, -2}};
  sc::Vec3 force{}, moment{};
  double power = 0;
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(result.nodal.nodes[i], static_cast<unsigned>(i));
    const auto f = result.nodal.forces[i];
    const auto x = positions[i];
    force = sc::Add(force, f);
    moment = sc::Add(moment, {x.y * f.z - x.z * f.y,
                             x.z * f.x - x.x * f.z,
                             x.x * f.y - x.y * f.x});
    power += sc::Dot(f, velocities[i]);
  }
  EXPECT_NEAR(force.x, 3, 1e-13);
  EXPECT_NEAR(force.y, -4, 1e-13);
  EXPECT_NEAR(force.z, 9, 1e-13);
  EXPECT_NEAR(moment.x, 27.5, 1e-12);
  EXPECT_NEAR(moment.y, -17.4, 1e-12);
  EXPECT_NEAR(moment.z, -16.9, 1e-12);
  EXPECT_NEAR(power, -3.4, 1e-12);
}

TEST_F(SurfaceContactCuda, TrialDiscardAndCommitOnDevice) {
  DeviceValue<TrialResults> device;
  ASSERT_EQ(device.allocation, cudaSuccess);
  TrialFixture<<<1, 1>>>(device.data);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  TrialResults result;
  ASSERT_EQ(cudaMemcpy(&result, device.data, sizeof(result), cudaMemcpyDeviceToHost), cudaSuccess);
  EXPECT_EQ(result.make_status, sc::Status::kOk);
  EXPECT_EQ(result.discard_commit_status, sc::Status::kNoTrial);
  EXPECT_EQ(result.after_discard.revision, 0u);
  EXPECT_DOUBLE_EQ(result.after_discard.accepted_time, 0);
  EXPECT_DOUBLE_EQ(result.after_discard.dissipated_energy, 0);
  EXPECT_EQ(result.retry_status, sc::Status::kOk);
  EXPECT_EQ(result.commit_status, sc::Status::kOk);
  EXPECT_EQ(result.second_commit_status, sc::Status::kNoTrial);
  EXPECT_EQ(result.after_commit.revision, 1u);
  EXPECT_DOUBLE_EQ(result.after_commit.accepted_time, 0.005);
  EXPECT_DOUBLE_EQ(result.after_commit.dissipated_energy, 0.1);
}

TEST_F(SurfaceContactCuda, SeamAndEdgeGeometryOnDevice) {
  DeviceValue<GeometryResults> device;
  ASSERT_EQ(device.allocation, cudaSuccess);
  GeometryFixture<<<1, 1>>>(device.data);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  GeometryResults result;
  ASSERT_EQ(cudaMemcpy(&result, device.data, sizeof(result), cudaMemcpyDeviceToHost), cudaSuccess);
  for (auto status : result.statuses) ASSERT_EQ(status, sc::Status::kOk);
  for (const auto& triangle : result.triangles) {
    EXPECT_DOUBLE_EQ(triangle.point.x, 0.5);
    EXPECT_DOUBLE_EQ(triangle.point.y, 0.5);
    EXPECT_DOUBLE_EQ(triangle.point.z, 0);
    EXPECT_DOUBLE_EQ(triangle.distance, 0.25);
    EXPECT_EQ(triangle.feature.kind, sc::FeatureKind::kEdge);
    EXPECT_EQ(triangle.feature.first, 10u);
    EXPECT_EQ(triangle.feature.second, 12u);
  }
  EXPECT_DOUBLE_EQ(result.edges[0].distance, 0.5);
  EXPECT_DOUBLE_EQ(result.edges[0].parameter_a, 0.5);
  EXPECT_DOUBLE_EQ(result.edges[0].parameter_b, 0.5);
  EXPECT_FALSE(result.edges[0].parallel);
  EXPECT_DOUBLE_EQ(result.edges[1].distance, 1);
  EXPECT_TRUE(result.edges[1].parallel);
}
}  // namespace
