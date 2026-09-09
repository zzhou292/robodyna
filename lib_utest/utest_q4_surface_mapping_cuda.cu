#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include "lib_src/collision/Q4SurfaceMapping.h"
#include "lib_src/collision/Q4SurfaceMass.h"

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

struct Results {
  sc::Status point_status, force_status, mass_status;
  sc::Q4PointKinematics point;
  sc::Q4NodalForces nodal;
  sc::NormalJacobian mass;
  sc::Status rejected_point_status, rejected_force_status, rejected_mass_status;
  sc::Q4PointKinematics after_rejected_point, retried_point;
  sc::Q4NodalForces after_rejected_force;
  sc::NormalJacobian after_rejected_mass;
  sc::Status retry_status;
};

// The identical fixture executes on the host and on one CUDA thread. The host
// test also checks explicit values, so CPU/device agreement is not the oracle.
TL_SURFACE_HD Results EvaluateFixture() {
  const double x[12] = {1,3,1,2, 2,2,5,7, -1,-2,-3,-4};
  const double v[12] = {1,2,4, -2,5,1, 7,3,-2, 4,-1,3};
  const double inverse[4] = {.5,.25,0,.125};
  std::uint8_t fixed[4] = {6,6,7,6};
  sc::SurfaceQ4 parent{{0,1,2,3},73,42,2,0};
  const sc::Q4SurfaceView surface{{x,4,1,4},{v,4,3,1},&parent,1};
  const sc::Q4FixedYZMassView mass{inverse,fixed,4,9};
  const sc::Q4Point point{0,.2,-.4};
  Results result;
  result.point_status = sc::EvaluateQ4Point(surface,point,&result.point);
  result.force_status = sc::ProjectQ4PointForce(surface,point,{3,-4,9},&result.nodal);
  result.mass_status = sc::BuildQ4NormalXJacobian(mass,parent,point.u,point.v,7,&result.mass);

  result.after_rejected_point=result.point;
  result.after_rejected_force=result.nodal;
  result.after_rejected_mass=result.mass;
  // The final parent entry fails after earlier nodes were checked. Caller
  // results must retain every accepted field, including unused mass slots.
  parent.nodes[3]=parent.nodes[0];
  result.rejected_point_status=sc::EvaluateQ4Point(surface,point,&result.after_rejected_point);
  result.rejected_force_status=sc::ProjectQ4PointForce(surface,point,{3,-4,9},&result.after_rejected_force);
  parent.nodes[3]=3;
  fixed[3]=0;
  result.rejected_mass_status=sc::BuildQ4NormalXJacobian(mass,parent,point.u,point.v,8,&result.after_rejected_mass);
  fixed[3]=6;
  result.retry_status=sc::EvaluateQ4Point(surface,point,&result.retried_point);
  return result;
}
__global__ void EvaluateOnDevice(Results* output) { *output=EvaluateFixture(); }

void Near(sc::Vec3 actual, sc::Vec3 expected, double tolerance=2e-13) {
  EXPECT_NEAR(actual.x,expected.x,tolerance);
  EXPECT_NEAR(actual.y,expected.y,tolerance);
  EXPECT_NEAR(actual.z,expected.z,tolerance);
}
void Same(const sc::Q4PointKinematics& actual,const sc::Q4PointKinematics& expected) {
  Near(actual.position,expected.position,0); Near(actual.velocity,expected.velocity,0);
  for(unsigned i=0;i<4;++i) EXPECT_EQ(actual.shape[i],expected.shape[i]);
}

class Q4SurfaceMappingCuda : public ::testing::Test {
 protected:
  void SetUp() override {
    int devices=0;
    ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
    ASSERT_GT(devices,0) << "Actual CUDA execution is required for this gate";
  }
};

TEST_F(Q4SurfaceMappingCuda, SameMapAndMaskedMassExecuteOnDevice) {
  DeviceValue<Results> device;
  ASSERT_EQ(device.allocation,cudaSuccess);
  EvaluateOnDevice<<<1,1>>>(device.data);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Results gpu;
  ASSERT_EQ(cudaMemcpy(&gpu,device.data,sizeof(gpu),cudaMemcpyDeviceToHost),cudaSuccess);
  const auto host=EvaluateFixture();
  ASSERT_EQ(gpu.point_status,sc::Status::kOk);
  ASSERT_EQ(gpu.force_status,sc::Status::kOk);
  ASSERT_EQ(gpu.mass_status,sc::Status::kOk);
  Near(gpu.point.position,{1.66,4.94,-2.94});
  Near(gpu.point.velocity,{3.58,1.38,1.54});
  Near(gpu.point.position,host.point.position); Near(gpu.point.velocity,host.point.velocity);
  EXPECT_NEAR(gpu.mass.inverse_effective_mass,.04185,1e-15);
  EXPECT_NEAR(gpu.mass.inverse_effective_mass,host.mass.inverse_effective_mass,1e-15);
  EXPECT_EQ(gpu.mass.base_epoch,9u); EXPECT_EQ(gpu.mass.attempt,7u);
  EXPECT_TRUE(gpu.mass.valid); ASSERT_EQ(gpu.mass.count,4u);
  const double weights[4]={.18,.12,.28,.42};
  sc::Vec3 total;
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(gpu.nodal.nodes[i],i); EXPECT_EQ(gpu.mass.nodes[i],i);
    EXPECT_NEAR(gpu.point.shape[i],weights[i],1e-15);
    Near(gpu.nodal.forces[i],sc::Scale({3,-4,9},weights[i]));
    Near(gpu.nodal.forces[i],host.nodal.forces[i]); Near(gpu.nodal.couples[i],{},0);
    Near(gpu.mass.values[i],{-weights[i],0,0});
    EXPECT_NEAR(gpu.mass.normalized_norm[i],host.mass.normalized_norm[i],1e-15);
    total=sc::Add(total,gpu.nodal.forces[i]);
  }
  Near(total,{3,-4,9});
  EXPECT_EQ(gpu.mass.normalized_norm[2],0);
}

TEST_F(Q4SurfaceMappingCuda, RejectedLateEntriesPreserveCompleteResultsAndRetry) {
  DeviceValue<Results> device;
  ASSERT_EQ(device.allocation,cudaSuccess);
  EvaluateOnDevice<<<1,1>>>(device.data);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Results gpu;
  ASSERT_EQ(cudaMemcpy(&gpu,device.data,sizeof(gpu),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(gpu.point_status,sc::Status::kOk);
  ASSERT_EQ(gpu.force_status,sc::Status::kOk);
  ASSERT_EQ(gpu.mass_status,sc::Status::kOk);
  EXPECT_EQ(gpu.rejected_point_status,sc::Status::kInvalidArgument);
  EXPECT_EQ(gpu.rejected_force_status,sc::Status::kInvalidArgument);
  EXPECT_EQ(gpu.rejected_mass_status,sc::Status::kUnsupportedInterpolation);
  Same(gpu.after_rejected_point,gpu.point);
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(gpu.after_rejected_force.nodes[i],gpu.nodal.nodes[i]);
    Near(gpu.after_rejected_force.forces[i],gpu.nodal.forces[i],0);
    Near(gpu.after_rejected_force.couples[i],gpu.nodal.couples[i],0);
  }
  EXPECT_EQ(gpu.after_rejected_mass.count,gpu.mass.count);
  EXPECT_EQ(gpu.after_rejected_mass.valid,gpu.mass.valid);
  EXPECT_EQ(gpu.after_rejected_mass.base_epoch,gpu.mass.base_epoch);
  EXPECT_EQ(gpu.after_rejected_mass.attempt,gpu.mass.attempt);
  EXPECT_EQ(gpu.after_rejected_mass.inverse_effective_mass,gpu.mass.inverse_effective_mass);
  for(unsigned i=0;i<sc::kMaxNormalNodes;++i) {
    EXPECT_EQ(gpu.after_rejected_mass.nodes[i],gpu.mass.nodes[i]);
    Near(gpu.after_rejected_mass.values[i],gpu.mass.values[i],0);
    EXPECT_EQ(gpu.after_rejected_mass.normalized_norm[i],gpu.mass.normalized_norm[i]);
  }
  ASSERT_EQ(gpu.retry_status,sc::Status::kOk); Same(gpu.retried_point,gpu.point);
}
}  // namespace
