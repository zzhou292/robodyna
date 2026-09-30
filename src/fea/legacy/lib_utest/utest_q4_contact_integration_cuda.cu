#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include "lib_utest/q4_contact_integration_fixture.h"

#include <cmath>

namespace {
namespace sc=tlfea::contact;
using q4_contact_test::Fixture;
using q4_contact_test::Limits;

struct BoundaryResult {
  sc::Q4IntegralInterval positive,negative;
  bool positive_ok=false,negative_ok=false;
};
struct DeviceStorage {
  sc::Q4IntegrationCell leaves[sc::MaxQ4IntegrationLeaves];
  std::uint32_t heap[sc::MaxQ4IntegrationLeaves];
  sc::Q4IntegrationResult result,before;
  sc::Q4IntegrationReport initial,report;
  sc::Status center_status=sc::Status::kInvalidArgument;
  BoundaryResult boundary;
  __device__ sc::Q4IntegrationScratch Scratch() {
    return {leaves,heap,sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};
  }
};
static_assert(sizeof(DeviceStorage) < 512*1024,"Keep this real CUDA gate below half a MiB");
struct DeviceValue {
  DeviceValue() : allocation(cudaMalloc(&data,sizeof(DeviceStorage))) {}
  ~DeviceValue() { if (data) cudaFree(data); }
  DeviceValue(const DeviceValue&)=delete;
  DeviceValue& operator=(const DeviceValue&)=delete;
  DeviceStorage* data=nullptr;
  cudaError_t allocation;
};
struct Readback {
  sc::Q4IntegrationResult result,before;
  sc::Q4IntegrationReport initial,report;
  sc::Status center_status=sc::Status::kInvalidArgument;
};
__global__ void RunFixture(DeviceStorage* storage,unsigned kind) {
  storage->result={}; storage->before={}; storage->initial={};
  storage->center_status=sc::Status::kInvalidArgument;
  Fixture fixture;
  auto limits=Limits();
  if (kind == 0) fixture.Gaps(.625,-.375,-.375,.625);
  if (kind == 1) {
    const double e=1./64; fixture.Gaps(e-2,e-1,e,e-1); limits=Limits(e);
  }
  if (kind == 2) fixture.Gaps(1,-1,1,-1);
  storage->report=sc::IntegrateQ4NormalContact(fixture.Input(),limits,storage->Scratch(),&storage->result);
}
__global__ void RunFailure(DeviceStorage* storage,unsigned kind) {
  storage->result={};
  storage->center_status=sc::Status::kInvalidArgument;
  Fixture fixture; fixture.Gaps(1,1,1,1);
  storage->initial=sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),storage->Scratch(),&storage->result);
  storage->before=storage->result;
  auto limits=Limits();
  if (kind == 0) { fixture.Gaps(1,-1048575,-1048575,1); limits.max_leaves=1; }
  if (kind == 1) fixture.Gaps(1e-200,1e-200,1e-200,1e-200);
  if (kind == 2) {
    for (unsigned i=0;i<4;++i)
      if (fixture.fixed[i] == 6) fixture.inverse[i]=16*::nextafter(0.,1.);
    sc::NormalJacobian center;
    storage->center_status=sc::BuildQ4NormalXJacobian(fixture.Input().mass,fixture.parent,0,0,7,&center);
  }
  storage->report=sc::IntegrateQ4NormalContact(fixture.Input(),limits,storage->Scratch(),&storage->result);
}
__global__ void RunUniformRetry(DeviceStorage* storage) {
  Fixture fixture; fixture.Gaps(1,1,1,1);
  storage->report=sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),storage->Scratch(),&storage->result);
}
__global__ void RunBoundary(DeviceStorage* storage,double below_twice) {
  storage->boundary={};
  storage->boundary.positive_ok=sc::q4_bounds::Scale({below_twice,below_twice},.5,&storage->boundary.positive);
  storage->boundary.negative_ok=sc::q4_bounds::Scale({-below_twice,-below_twice},.5,&storage->boundary.negative);
}
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(const sc::Q4IntegrationResult& a,const sc::Q4IntegrationResult& b) {
  for (unsigned i=0;i<4;++i) {
    Same(a.force[i],b.force[i]); EXPECT_EQ(a.nodal.nodes[i],b.nodal.nodes[i]);
    Same(a.nodal.forces[i],b.nodal.forces[i]); Same(a.nodal.couples[i],b.nodal.couples[i]);
  }
  Same(a.resultant,b.resultant); Same(a.potential,b.potential);
  EXPECT_EQ(a.active_area.lower,b.active_area.lower); EXPECT_EQ(a.active_area.upper,b.active_area.upper);
  EXPECT_EQ(a.feature_id,b.feature_id); EXPECT_EQ(a.parent_element_id,b.parent_element_id);
  EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
  EXPECT_EQ(a.leaf_count,b.leaf_count); EXPECT_EQ(a.visited,b.visited); EXPECT_EQ(a.deepest_leaf,b.deepest_leaf);
  EXPECT_EQ(a.valid,b.valid);
}
void Encloses(sc::Q4CertifiedIntegral value,long double truth,double budget) {
  EXPECT_LE(static_cast<long double>(value.lower),truth);
  EXPECT_GE(static_cast<long double>(value.upper),truth);
  EXPECT_LE(std::abs(static_cast<long double>(value.value)-truth),static_cast<long double>(value.error));
  EXPECT_LE(value.error,budget);
}
class Q4ContactIntegrationCuda : public ::testing::Test {
 protected:
  void SetUp() override {
    int devices=0;
    ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
    ASSERT_GT(devices,0) << "Actual CUDA execution is required for this gate";
  }
  void Read(DeviceValue& device,Readback* output) {
    ASSERT_EQ(cudaMemcpy(&output->result,&device.data->result,sizeof(output->result),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&output->before,&device.data->before,sizeof(output->before),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&output->initial,&device.data->initial,sizeof(output->initial),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&output->report,&device.data->report,sizeof(output->report),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&output->center_status,&device.data->center_status,sizeof(output->center_status),cudaMemcpyDeviceToHost),cudaSuccess);
  }
};

TEST_F(Q4ContactIntegrationCuda, PartialCutCornerAndSaddleEncloseIndependentExactIntegrals) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  for (unsigned kind=0;kind<3;++kind) {
    RunFixture<<<1,1>>>(device.data,kind); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Readback result; Read(device,&result);
    ASSERT_EQ(result.report.status,sc::Q4IntegrationStatus::Ok) << kind;
    const auto oracle=kind == 0 ? q4_contact_test::Cut(.375) : kind == 1 ?
        q4_contact_test::Corner(1.L/64) : q4_contact_test::Saddle();
    const auto limits=kind == 1 ? Limits(1./64) : Limits();
    long double force=0;
    for (unsigned i=0;i<4;++i) {
      Encloses(result.result.force[i],oracle.force[i],limits.force_error); force+=oracle.force[i];
      Same(result.result.nodal.couples[i],{});
    }
    Encloses(result.result.resultant,force,limits.force_error);
    Encloses(result.result.potential,oracle.potential,limits.energy_error);
    EXPECT_TRUE(result.result.valid); EXPECT_GT(result.result.resultant.value,0);
  }
}

TEST_F(Q4ContactIntegrationCuda, CapacityRejectionPreservesCompleteCallerResultAndRetry) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  RunFailure<<<1,1>>>(device.data,0); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Readback result; Read(device,&result);
  ASSERT_EQ(result.initial.status,sc::Q4IntegrationStatus::Ok);
  EXPECT_EQ(result.report.status,sc::Q4IntegrationStatus::LeafLimit);
  Same(result.result,result.before);
  RunFixture<<<1,1>>>(device.data,2); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Read(device,&result); ASSERT_EQ(result.report.status,sc::Q4IntegrationStatus::Ok);
  Encloses(result.result.resultant,1.L/8,Limits().force_error);
  Encloses(result.result.potential,1.L/36,Limits().energy_error);
}

TEST_F(Q4ContactIntegrationCuda, PositiveEnergyUnderflowIsDiagnosedWithoutPublication) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  RunFailure<<<1,1>>>(device.data,1); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Readback result; Read(device,&result);
  ASSERT_EQ(result.initial.status,sc::Q4IntegrationStatus::Ok);
  EXPECT_EQ(result.report.status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
  Same(result.result,result.before);
}

TEST_F(Q4ContactIntegrationCuda, SignedNormalBoundaryProductsRemainEnclosedOnDevice) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  const double below_twice=std::nextafter(2*DBL_MIN,0.);
  RunBoundary<<<1,1>>>(device.data,below_twice); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  BoundaryResult result;
  ASSERT_EQ(cudaMemcpy(&result,&device.data->boundary,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_TRUE(result.positive_ok); ASSERT_TRUE(result.negative_ok);
  const long double exact=static_cast<long double>(below_twice)/2;
  EXPECT_LE(static_cast<long double>(result.positive.lower),exact);
  EXPECT_GE(static_cast<long double>(result.positive.upper),exact);
  EXPECT_LE(static_cast<long double>(result.negative.lower),-exact);
  EXPECT_GE(static_cast<long double>(result.negative.upper),-exact);
  EXPECT_LT(result.positive.lower,DBL_MIN); EXPECT_GT(result.negative.upper,-DBL_MIN);
}

TEST_F(Q4ContactIntegrationCuda, LateGaussMassFailurePreservesSeededResultAndNormalMassRetry) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  RunFailure<<<1,1>>>(device.data,2); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Readback result; Read(device,&result);
  ASSERT_EQ(result.initial.status,sc::Q4IntegrationStatus::Ok);
  ASSERT_EQ(result.center_status,sc::Status::kOk);
  EXPECT_EQ(result.report.status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
  EXPECT_EQ(result.report.cell,0u); EXPECT_EQ(result.report.visited,1u);
  Same(result.result,result.before);
  RunUniformRetry<<<1,1>>>(device.data); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Read(device,&result); ASSERT_EQ(result.report.status,sc::Q4IntegrationStatus::Ok);
  Same(result.result,result.before);
}
}  // namespace
