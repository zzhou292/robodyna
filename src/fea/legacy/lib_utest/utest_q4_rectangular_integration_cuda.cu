#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include "lib_utest/q4_rectangular_integration_fixture.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {
namespace sc=tlfea::contact;
namespace test=q4_rectangular_test;
using test::Fixture;
using test::Limits;

struct Readback {
  sc::Q4RectangularResult result;
  sc::Q4IntegrationResult scalar;
  unsigned char before[sizeof(sc::Q4RectangularResult)]{};
  sc::Q4IntegrationReport initial,report,scalar_report;
  sc::Status center_status=sc::Status::kInvalidArgument;
};
struct DeviceStorage {
  sc::Q4RectangularCell leaves[sc::MaxQ4IntegrationLeaves];
  std::uint32_t heap[sc::MaxQ4IntegrationLeaves];
  sc::Q4IntegrationCell scalar_leaves[sc::MaxQ4IntegrationLeaves];
  std::uint32_t scalar_heap[sc::MaxQ4IntegrationLeaves];
  Readback output;
  __device__ sc::Q4RectangularScratch Scratch() {
    return {leaves,heap,sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};
  }
  __device__ sc::Q4IntegrationScratch ScalarScratch() {
    return {scalar_leaves,scalar_heap,sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};
  }
};
static_assert(sizeof(DeviceStorage)<1024*1024,"Keep all CUDA allocations below one MiB per test");
struct DeviceValue {
  DeviceValue():allocation(cudaMalloc(&data,sizeof(DeviceStorage))) {}
  ~DeviceValue() { if (data) cudaFree(data); }
  DeviceValue(const DeviceValue&)=delete;
  DeviceValue& operator=(const DeviceValue&)=delete;
  DeviceStorage* data=nullptr;
  cudaError_t allocation;
};
struct HostScratch {
  HostScratch():leaves(sc::MaxQ4IntegrationLeaves),heap(sc::MaxQ4IntegrationLeaves) {}
  std::vector<sc::Q4RectangularCell> leaves;
  std::vector<std::uint32_t> heap;
  sc::Q4RectangularScratch View() {
    return {leaves.data(),heap.data(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};
  }
};

// Parent-array position differs from both its physical element ID and each
// mapped node ID. The first parent must never supply the result association.
TL_SURFACE_HD sc::Q4NormalIntegrationInput SelectedParent(const Fixture& fixture,sc::SurfaceQ4 parents[2]) {
  parents[0]={{0,1,2,3},991,992,3,0}; parents[1]=fixture.parent;
  auto input=fixture.Input(); input.surface.parents=parents;
  input.surface.parent_count=2; input.parent_index=1;
  return input;
}
__global__ void RunFixture(DeviceStorage* storage,unsigned kind,bool scalar) {
  storage->output={};
  Fixture fixture; sc::Q4IntegrationLimits limits; test::Configure(kind,fixture,limits);
  sc::SurfaceQ4 parents[2]; const auto input=SelectedParent(fixture,parents);
  storage->output.report=sc::IntegrateQ4NormalContactRectangular(input,limits,storage->Scratch(),&storage->output.result);
  if (scalar)
    storage->output.scalar_report=sc::IntegrateQ4NormalContact(input,limits,storage->ScalarScratch(),&storage->output.scalar);
}
__global__ void RunFailure(DeviceStorage* storage,unsigned kind) {
  storage->output={};
  Fixture fixture; fixture.Gaps(1,1,1,1);
  storage->output.initial=sc::IntegrateQ4NormalContactRectangular(fixture.Input(),Limits(),storage->Scratch(),&storage->output.result);
  const auto* bytes=reinterpret_cast<const unsigned char*>(&storage->output.result);
  for (unsigned i=0;i<sizeof(storage->output.result);++i) storage->output.before[i]=bytes[i];
  auto limits=Limits();
  if (kind<4) {
    const double e=1./64; fixture.Gaps(e-2,e-1,e,e-1); limits=Limits(e);
    if (kind==0) limits.max_leaves=1;
    if (kind==1) limits.max_visited=1;
    if (kind==2) limits.max_depth=0;
    if (kind==3) limits.max_depth=sc::MaxQ4IntegrationDepth+1;
  }
  if (kind==4)
    for (unsigned n=0;n<4;++n)
      if (fixture.fixed[n]==6) fixture.inverse[n]=16*::nextafter(0.,1.);
  if (kind==5) fixture.Gaps(1e-200,1e-200,1e-200,1e-200);
  sc::NormalJacobian center;
  storage->output.center_status=sc::BuildQ4NormalXJacobian(fixture.Input().mass,fixture.parent,0,0,7,&center);
  storage->output.report=sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,storage->Scratch(),&storage->output.result);
}
__global__ void RunUniformRetry(DeviceStorage* storage) {
  Fixture fixture; fixture.Gaps(1,1,1,1);
  storage->output.report=sc::IntegrateQ4NormalContactRectangular(fixture.Input(),Limits(),storage->Scratch(),&storage->output.result);
}

void Encloses(const sc::Q4CertifiedIntegral& value,long double exact,double limit) {
  EXPECT_LE(static_cast<long double>(value.lower),exact);
  EXPECT_GE(static_cast<long double>(value.upper),exact);
  EXPECT_LE(std::abs(static_cast<long double>(value.value)-exact),static_cast<long double>(value.error));
  EXPECT_LE(value.error,limit);
}
void Overlap(const sc::Q4CertifiedIntegral& a,const sc::Q4CertifiedIntegral& b) {
  EXPECT_LE(a.lower,b.upper); EXPECT_LE(b.lower,a.upper);
  EXPECT_LE(std::abs(static_cast<long double>(a.value)-b.value),static_cast<long double>(a.error)+b.error);
}
void Check(const sc::Q4IntegrationResult& r,const test::Oracle& oracle,const sc::Q4IntegrationLimits& limits) {
  ASSERT_TRUE(r.valid); long double sum=0; const Fixture fixture;
  for (unsigned n=0;n<4;++n) {
    Encloses(r.force[n],oracle.force[n],limits.force_error); sum+=oracle.force[n];
    EXPECT_EQ(r.nodal.nodes[n],fixture.parent.nodes[n]);
    EXPECT_EQ(r.nodal.forces[n].x,-r.force[n].value);
    EXPECT_EQ(r.nodal.forces[n].y,0); EXPECT_EQ(r.nodal.forces[n].z,0);
    EXPECT_EQ(r.nodal.couples[n].x,0); EXPECT_EQ(r.nodal.couples[n].y,0); EXPECT_EQ(r.nodal.couples[n].z,0);
  }
  Encloses(r.resultant,sum,limits.force_error); Encloses(r.potential,oracle.potential,limits.energy_error);
  EXPECT_EQ(r.feature_id,73u); EXPECT_EQ(r.parent_element_id,42u);
  EXPECT_EQ(r.base_epoch,9u); EXPECT_EQ(r.attempt,7u);
  EXPECT_GT(r.leaf_count,0u); EXPECT_LE(r.leaf_count,limits.max_leaves);
  EXPECT_LE(r.visited,limits.max_visited); EXPECT_LE(r.deepest_leaf,limits.max_depth);
}
void Check(const sc::Q4RectangularResult& r,const test::Oracle& oracle,const sc::Q4IntegrationLimits& limits) {
  Check(r.integration,oracle,limits);
  EXPECT_EQ(r.integration.visited,2*r.integration.leaf_count-1);
  EXPECT_EQ(r.integration.deepest_leaf,std::max(r.deepest_u,r.deepest_v));
  EXPECT_LE(r.deepest_u,limits.max_depth); EXPECT_LE(r.deepest_v,limits.max_depth);
}
void Overlap(const sc::Q4IntegrationResult& a,const sc::Q4IntegrationResult& b) {
  for (unsigned n=0;n<4;++n) Overlap(a.force[n],b.force[n]);
  Overlap(a.resultant,b.resultant); Overlap(a.potential,b.potential);
  EXPECT_LE(a.active_area.lower,b.active_area.upper); EXPECT_LE(b.active_area.lower,a.active_area.upper);
}
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(const sc::Q4RectangularResult& first,const sc::Q4RectangularResult& second) {
  const auto& a=first.integration; const auto& b=second.integration;
  for (unsigned n=0;n<4;++n) {
    Same(a.force[n],b.force[n]); EXPECT_EQ(a.nodal.nodes[n],b.nodal.nodes[n]);
    Same(a.nodal.forces[n],b.nodal.forces[n]); Same(a.nodal.couples[n],b.nodal.couples[n]);
  }
  Same(a.resultant,b.resultant); Same(a.potential,b.potential);
  EXPECT_EQ(a.active_area.lower,b.active_area.lower); EXPECT_EQ(a.active_area.upper,b.active_area.upper);
  EXPECT_EQ(a.feature_id,b.feature_id); EXPECT_EQ(a.parent_element_id,b.parent_element_id);
  EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
  EXPECT_EQ(a.leaf_count,b.leaf_count); EXPECT_EQ(a.visited,b.visited); EXPECT_EQ(a.deepest_leaf,b.deepest_leaf);
  EXPECT_EQ(a.valid,b.valid); EXPECT_EQ(first.deepest_u,second.deepest_u); EXPECT_EQ(first.deepest_v,second.deepest_v);
}
class Q4RectangularIntegrationCuda : public ::testing::Test {
 protected:
  void SetUp() override {
    int devices=0; ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
    ASSERT_GT(devices,0) << "Actual CUDA execution is required for this qualification";
  }
  void Read(DeviceValue& device,Readback* output) {
    ASSERT_EQ(cudaMemcpy(output,&device.data->output,sizeof(*output),cudaMemcpyDeviceToHost),cudaSuccess);
  }
  void Retry(DeviceValue& device,const Readback& failed) {
    sc::Q4RectangularResult seed; std::memcpy(&seed,failed.before,sizeof(seed));
    RunUniformRetry<<<1,1>>>(device.data); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Readback output; ASSERT_NO_FATAL_FAILURE(Read(device,&output));
    ASSERT_EQ(output.report.status,sc::Q4IntegrationStatus::Ok);
    Same(output.result,seed); Check(output.result,q4_contact_test::Uniform(1),Limits());
  }
};

TEST_F(Q4RectangularIntegrationCuda, CpuAndGpuEncloseIndependentIntegralsForFivePhysicalCases) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess); HostScratch scratch;
  for (unsigned kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind);
    Fixture fixture; sc::Q4IntegrationLimits limits; test::Configure(kind,fixture,limits);
    sc::SurfaceQ4 parents[2]; const auto input=SelectedParent(fixture,parents);
    sc::Q4RectangularResult host;
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(input,limits,scratch.View(),&host).status,sc::Q4IntegrationStatus::Ok);
    RunFixture<<<1,1>>>(device.data,kind,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Readback gpu; ASSERT_NO_FATAL_FAILURE(Read(device,&gpu));
    ASSERT_EQ(gpu.report.status,sc::Q4IntegrationStatus::Ok);
    const auto oracle=test::Expected(kind); Check(host,oracle,limits); Check(gpu.result,oracle,limits);
    // Independent enclosure overlap permits different rounded sample estimates
    // and valid refinement decisions; it makes no cross-device bitwise claim.
    Overlap(host.integration,gpu.result.integration);
    const long double areas[5]={.625L,1.L/8192,.5L,.625L+std::ldexp(1.L,-41),1.L};
    EXPECT_LE(static_cast<long double>(gpu.result.integration.active_area.lower),areas[kind]);
    EXPECT_GE(static_cast<long double>(gpu.result.integration.active_area.upper),areas[kind]);
    EXPECT_EQ(gpu.report.leaves,gpu.result.integration.leaf_count);
    EXPECT_EQ(gpu.report.depth,gpu.result.integration.deepest_leaf);
    EXPECT_EQ(gpu.report.visited,gpu.result.integration.visited);
  }
}

TEST_F(Q4RectangularIntegrationCuda, ExactAndNonzeroTransverseVariationReduceScalarLeafWork) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  for (const unsigned kind:{0u,3u}) {
    SCOPED_TRACE(kind);
    RunFixture<<<1,1>>>(device.data,kind,true); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Readback output; ASSERT_NO_FATAL_FAILURE(Read(device,&output));
    ASSERT_EQ(output.report.status,sc::Q4IntegrationStatus::Ok);
    ASSERT_EQ(output.scalar_report.status,sc::Q4IntegrationStatus::Ok);
    Check(output.result,test::Expected(kind),Limits()); Check(output.scalar,test::Expected(kind),Limits());
    Overlap(output.result.integration,output.scalar);
    EXPECT_LT(output.result.integration.leaf_count,output.scalar.leaf_count);
    EXPECT_GT(output.result.deepest_u,0u); EXPECT_EQ(output.result.deepest_v,0u);
    // This is a prescribed partition work-count gate, not a timing or C4 owner gate.
  }
}

TEST_F(Q4RectangularIntegrationCuda, EveryHardCapPreservesEveryOutputByteAndCleanRetry) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  const sc::Q4IntegrationStatus expected[4]={sc::Q4IntegrationStatus::LeafLimit,sc::Q4IntegrationStatus::VisitLimit,
      sc::Q4IntegrationStatus::DepthLimit,sc::Q4IntegrationStatus::InvalidInput};
  for (unsigned kind=0;kind<4;++kind) {
    SCOPED_TRACE(kind);
    RunFailure<<<1,1>>>(device.data,kind); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Readback output; ASSERT_NO_FATAL_FAILURE(Read(device,&output));
    ASSERT_EQ(output.initial.status,sc::Q4IntegrationStatus::Ok);
    EXPECT_EQ(output.report.status,expected[kind]);
    EXPECT_EQ(std::memcmp(&output.result,output.before,sizeof(output.result)),0);
    ASSERT_NO_FATAL_FAILURE(Retry(device,output));
  }
}

TEST_F(Q4RectangularIntegrationCuda, LateGaussMassAndPositiveEnergyFailuresNeverPublish) {
  DeviceValue device; ASSERT_EQ(device.allocation,cudaSuccess);
  for (const unsigned kind:{4u,5u}) {
    SCOPED_TRACE(kind);
    RunFailure<<<1,1>>>(device.data,kind); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Readback output; ASSERT_NO_FATAL_FAILURE(Read(device,&output));
    ASSERT_EQ(output.initial.status,sc::Q4IntegrationStatus::Ok);
    ASSERT_EQ(output.center_status,sc::Status::kOk);
    EXPECT_EQ(output.report.status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
    if (kind==4) { EXPECT_EQ(output.report.cell,0u); EXPECT_EQ(output.report.visited,1u); }
    EXPECT_EQ(std::memcmp(&output.result,output.before,sizeof(output.result)),0);
    ASSERT_NO_FATAL_FAILURE(Retry(device,output));
  }
}
}  // namespace
