#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include "lib_utest/parametric_contact_cuda_fixture.h"

#include <array>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace sc=tlfea::contact;
namespace test=parametric_cuda_test;
struct Q4Readback {
  sc::Q4RectangularResult raw,expanded;
  sc::Q4IntegrationReport report;
  sc::Q4CertifiedIntegral density;
  sc::SurfaceMeasureStatus density_status=sc::SurfaceMeasureStatus::NotPrepared;
  sc::Status center_status=sc::Status::kInvalidArgument;
  bool expanded_ok=false;
  unsigned char before_raw[sizeof(raw)]{},before_expanded[sizeof(expanded)]{};
};
struct T3Readback {
  sc::T3IntegrationResult result;
  sc::T3IntegrationReport report;
  sc::Status center_status=sc::Status::kInvalidArgument;
  unsigned char before[sizeof(result)]{};
};
struct Storage {
  test::Q4Fixture q4;
  test::T3Fixture t3;
  sc::Q4RectangularCell leaves[sc::MaxQ4IntegrationLeaves];
  std::uint32_t heap[sc::MaxQ4IntegrationLeaves];
  Q4Readback q4_read;
  T3Readback t3_read;
  __device__ sc::Q4RectangularScratch Scratch() {
    return {leaves,heap,sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};
  }
};
static_assert(sizeof(Storage)<1024*1024,"One-thread contact parity allocation must remain under 1MiB");
struct Device {
  Storage* data=nullptr;
  cudaError_t allocated=cudaMalloc(&data,sizeof(Storage));
  ~Device() { if (data) cudaFree(data); }
  Device()=default;
  Device(const Device&)=delete;
  Device& operator=(const Device&)=delete;
};
struct HostScratch {
  std::vector<sc::Q4RectangularCell> leaves=std::vector<sc::Q4RectangularCell>(sc::MaxQ4IntegrationLeaves);
  std::vector<std::uint32_t> heap=std::vector<std::uint32_t>(sc::MaxQ4IntegrationLeaves);
  sc::Q4RectangularScratch View() { return {leaves.data(),heap.data(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves}; }
};
template<class T> __device__ void SaveBytes(const T& value,unsigned char* bytes) {
  const auto* from=reinterpret_cast<const unsigned char*>(&value);
  for (unsigned i=0;i<sizeof(T);++i) bytes[i]=from[i];
}
// These invoke only pure HD operations. Immutable reference preparation and
// finite-wall coverage remain host operations and are not promoted by this gate.
__global__ void RunQ4(Storage* storage,bool capture_before) {
  auto& output=storage->q4_read; const auto& fixture=storage->q4;
  if (capture_before) { SaveBytes(output.raw,output.before_raw); SaveBytes(output.expanded,output.before_expanded); }
  const auto input=fixture.Input(); sc::NormalJacobian center;
  output.center_status=sc::BuildQ4NormalXJacobian(input.mass,fixture.parents[1],0,0,31,&center);
  output.density_status=sc::EvaluateQ4MaterialDensity(fixture.intrinsic,.375,-.25,&output.density);
  output.report=sc::IntegrateQ4NormalContactRectangular(input,fixture.limits,storage->Scratch(),&output.raw);
  output.expanded_ok=false;
  if (output.report.status==sc::Q4IntegrationStatus::Ok) {
    auto next=output.raw;
    output.expanded_ok=sc::ExpandQ4IntegralMeasure(fixture.area.value,{fixture.area.lower,fixture.area.upper},&next.integration) &&
                       sc::WithinQ4IntegralBudgets(next.integration,fixture.limits);
    if (output.expanded_ok) output.expanded=next;
  }
}
__global__ void RunT3(Storage* storage,bool capture_before) {
  auto& output=storage->t3_read; const auto& fixture=storage->t3;
  if (capture_before) SaveBytes(output.result,output.before);
  const auto input=fixture.Input(); const double weights[3]={1./3,1./3,1./3}; sc::NormalJacobian center;
  output.center_status=sc::BuildLinearTriangleNormalJacobian(input.mass,fixture.parents[1],weights,nullptr,nullptr,
                                                           {-1,0,0},31,&center);
  output.report=sc::IntegrateT3NormalContact(input,fixture.limits,&output.result);
}

void Encloses(sc::Q4CertifiedIntegral cert,long double truth,double limit) {
  EXPECT_LE(static_cast<long double>(cert.lower),truth); EXPECT_GE(static_cast<long double>(cert.upper),truth);
  EXPECT_LE(std::abs(static_cast<long double>(cert.value)-truth),static_cast<long double>(cert.error));
  EXPECT_LE(cert.error,limit);
}
void Overlap(sc::Q4CertifiedIntegral cpu,sc::Q4CertifiedIntegral gpu) {
  EXPECT_LE(cpu.lower,gpu.upper); EXPECT_LE(gpu.lower,cpu.upper);
  EXPECT_LE(std::abs(static_cast<long double>(cpu.value)-gpu.value),static_cast<long double>(cpu.error)+gpu.error);
}
void CheckQ4(const test::Q4Fixture& fixture,unsigned kind,const sc::Q4RectangularResult& result,
             const sc::Q4RectangularResult& cpu) {
  const auto truth=test::Q4Truth(fixture,kind); const auto& r=result.integration;
  ASSERT_TRUE(r.valid); long double sum=0;
  for (unsigned n=0;n<4;++n) {
    Encloses(r.force[n],truth.force[n],fixture.limits.force_error); Overlap(cpu.integration.force[n],r.force[n]); sum+=truth.force[n];
    EXPECT_EQ(r.nodal.nodes[n],fixture.parents[1].nodes[n]); EXPECT_EQ(r.nodal.forces[n].x,-r.force[n].value);
    EXPECT_EQ(r.nodal.forces[n].y,0); EXPECT_EQ(r.nodal.forces[n].z,0);
    EXPECT_EQ(r.nodal.couples[n].x,0); EXPECT_EQ(r.nodal.couples[n].y,0); EXPECT_EQ(r.nodal.couples[n].z,0);
  }
  Encloses(r.resultant,sum,fixture.limits.force_error); Encloses(r.potential,truth.potential,fixture.limits.energy_error);
  Overlap(cpu.integration.resultant,r.resultant); Overlap(cpu.integration.potential,r.potential);
  EXPECT_EQ(r.feature_id,103u); EXPECT_EQ(r.parent_element_id,fixture.parents[1].parent_element_id);
  EXPECT_EQ(r.base_epoch,17u); EXPECT_EQ(r.attempt,31u);
  EXPECT_EQ(result.deepest_u,cpu.deepest_u); EXPECT_EQ(result.deepest_v,cpu.deepest_v);
}
void CheckT3(const test::T3Fixture& fixture,unsigned kind,const sc::T3IntegrationResult& r,const sc::T3IntegrationResult& cpu) {
  const auto truth=test::T3Truth(kind); ASSERT_TRUE(r.valid); long double sum=0;
  for (unsigned n=0;n<3;++n) {
    Encloses(r.force[n],truth.force[n],fixture.limits.force_error); Overlap(cpu.force[n],r.force[n]); sum+=truth.force[n];
    EXPECT_EQ(r.nodal.nodes[n],fixture.parents[1].nodes[n]); EXPECT_EQ(r.nodal.forces[n].x,-r.force[n].value);
    EXPECT_EQ(r.nodal.forces[n].y,0); EXPECT_EQ(r.nodal.forces[n].z,0);
  }
  Encloses(r.resultant,sum,fixture.limits.force_error); Encloses(r.potential,truth.potential,fixture.limits.energy_error);
  Overlap(cpu.resultant,r.resultant); Overlap(cpu.potential,r.potential);
  EXPECT_EQ(r.feature_id,103u); EXPECT_EQ(r.parent_element_id,203u); EXPECT_EQ(r.parent_face_id,9u);
  EXPECT_EQ(r.base_epoch,17u); EXPECT_EQ(r.attempt,31u);
  EXPECT_EQ(r.subtriangle_count,cpu.subtriangle_count); EXPECT_EQ(r.sample_count,cpu.sample_count);
  EXPECT_LE(r.subtriangle_count,2u); EXPECT_LE(r.sample_count,6u);
}

TEST(ParametricContactCuda, Q4GenuineMassAreaCertificateSourceMappingAndDensityParity) {
  Device device; ASSERT_EQ(device.allocated,cudaSuccess); ASSERT_EQ(cudaMemset(device.data,0,sizeof(Storage)),cudaSuccess);
  HostScratch scratch;
  for (unsigned kind=0;kind<4;++kind) {
    SCOPED_TRACE(kind); test::Q4Fixture fixture; ASSERT_TRUE(fixture.Prepare(kind));
    sc::Q4RectangularResult cpu;
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),fixture.limits,scratch.View(),&cpu).status,
              sc::Q4IntegrationStatus::Ok);
    ASSERT_TRUE(sc::ExpandQ4IntegralMeasure(fixture.area.value,{fixture.area.lower,fixture.area.upper},&cpu.integration));
    ASSERT_TRUE(sc::WithinQ4IntegralBudgets(cpu.integration,fixture.limits));
    ASSERT_EQ(cudaMemcpy(&device.data->q4,&fixture,sizeof(fixture),cudaMemcpyHostToDevice),cudaSuccess);
    RunQ4<<<1,1>>>(device.data,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    Q4Readback actual; ASSERT_EQ(cudaMemcpy(&actual,&device.data->q4_read,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.report.status,sc::Q4IntegrationStatus::Ok); ASSERT_TRUE(actual.expanded_ok);
    ASSERT_EQ(actual.density_status,sc::SurfaceMeasureStatus::Ok);
    Encloses(actual.density,test::Q4Density(fixture,.375,-.25),DBL_MAX);
    CheckQ4(fixture,kind,actual.expanded,cpu);
  }
  RecordProperty("explicit_device_allocation_bytes",static_cast<int>(sizeof(Storage)));
  RecordProperty("threads_per_launch",1);
}

TEST(ParametricContactCuda, T3NativeClippedSimplexMassAndEdgeOnParity) {
  Device device; ASSERT_EQ(device.allocated,cudaSuccess); ASSERT_EQ(cudaMemset(device.data,0,sizeof(Storage)),cudaSuccess);
  for (unsigned kind=0;kind<6;++kind) {
    SCOPED_TRACE(kind); test::T3Fixture fixture; ASSERT_TRUE(fixture.Prepare(kind)); sc::T3IntegrationResult cpu;
    ASSERT_EQ(sc::IntegrateT3NormalContact(fixture.Input(),fixture.limits,&cpu).status,sc::T3IntegrationStatus::Ok);
    ASSERT_EQ(cudaMemcpy(&device.data->t3,&fixture,sizeof(fixture),cudaMemcpyHostToDevice),cudaSuccess);
    RunT3<<<1,1>>>(device.data,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    T3Readback actual; ASSERT_EQ(cudaMemcpy(&actual,&device.data->t3_read,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.report.status,sc::T3IntegrationStatus::Ok); CheckT3(fixture,kind,actual.result,cpu);
  }
}

TEST(ParametricContactCuda, Q4FailedDeviceOutputRemainsByteExactAndCleanRetryMatches) {
  Device device; ASSERT_EQ(device.allocated,cudaSuccess); ASSERT_EQ(cudaMemset(device.data,0,sizeof(Storage)),cudaSuccess);
  test::Q4Fixture good; ASSERT_TRUE(good.Prepare(0));
  ASSERT_EQ(cudaMemcpy(&device.data->q4,&good,sizeof(good),cudaMemcpyHostToDevice),cudaSuccess);
  RunQ4<<<1,1>>>(device.data,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  Q4Readback clean; ASSERT_EQ(cudaMemcpy(&clean,&device.data->q4_read,sizeof(clean),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(clean.report.status,sc::Q4IntegrationStatus::Ok); ASSERT_TRUE(clean.expanded_ok);
  for (unsigned fault=0;fault<3;++fault) {
    SCOPED_TRACE(fault); auto bad=good;
    if (fault==0) bad.mass_model=sc::TranslationMassModel::kUnspecified;
    if (fault==1) bad.position[3*bad.parents[1].nodes[3]]=.25;
    if (fault==2) {
      for (unsigned node:bad.parents[1].nodes) { bad.fixed[node]=0; bad.inverse[node]=16*std::numeric_limits<double>::denorm_min(); }
      const auto fixed=bad.parents[1].nodes[0]; bad.fixed[fixed]=1; bad.inverse[fixed]=0;
    }
    ASSERT_EQ(cudaMemcpy(&device.data->q4,&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
    RunQ4<<<1,1>>>(device.data,true); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    Q4Readback actual; ASSERT_EQ(cudaMemcpy(&actual,&device.data->q4_read,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_NE(actual.report.status,sc::Q4IntegrationStatus::Ok);
    EXPECT_EQ(std::memcmp(actual.before_raw,&actual.raw,sizeof(actual.raw)),0);
    EXPECT_EQ(std::memcmp(actual.before_expanded,&actual.expanded,sizeof(actual.expanded)),0);
    if (fault==2) { EXPECT_EQ(actual.center_status,sc::Status::kOk); EXPECT_EQ(actual.report.status,sc::Q4IntegrationStatus::NonFiniteArithmetic); }
    ASSERT_EQ(cudaMemcpy(&device.data->q4,&good,sizeof(good),cudaMemcpyHostToDevice),cudaSuccess);
    RunQ4<<<1,1>>>(device.data,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&actual,&device.data->q4_read,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.report.status,sc::Q4IntegrationStatus::Ok); ASSERT_TRUE(actual.expanded_ok);
    CheckQ4(good,0,actual.expanded,clean.expanded);
  }
}

TEST(ParametricContactCuda, T3EarlyAndLateDeviceFailuresPreserveResultAndRetry) {
  Device device; ASSERT_EQ(device.allocated,cudaSuccess); ASSERT_EQ(cudaMemset(device.data,0,sizeof(Storage)),cudaSuccess);
  test::T3Fixture good; ASSERT_TRUE(good.Prepare(2));
  ASSERT_EQ(cudaMemcpy(&device.data->t3,&good,sizeof(good),cudaMemcpyHostToDevice),cudaSuccess);
  RunT3<<<1,1>>>(device.data,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  T3Readback clean; ASSERT_EQ(cudaMemcpy(&clean,&device.data->t3_read,sizeof(clean),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(clean.report.status,sc::T3IntegrationStatus::Ok);
  for (unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault); auto bad=good;
    if (fault==0) bad.mass_model=sc::TranslationMassModel::kUnspecified;
    if (fault==1) bad.parents[1].feature_id++;
    if (fault==2) bad.position[3*7]=5;
    if (fault==3) {
      bad.position[3*5]=1e-10; bad.position[3*1]=bad.position[3*7]=-1;
      bad.fixed[5]=1; bad.inverse[5]=0; bad.inverse[1]=bad.inverse[7]=1e-308;
    }
    ASSERT_EQ(cudaMemcpy(&device.data->t3,&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
    RunT3<<<1,1>>>(device.data,true); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    T3Readback actual; ASSERT_EQ(cudaMemcpy(&actual,&device.data->t3_read,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_NE(actual.report.status,sc::T3IntegrationStatus::Ok);
    EXPECT_EQ(std::memcmp(actual.before,&actual.result,sizeof(actual.result)),0);
    if (fault==3) {
      EXPECT_EQ(actual.center_status,sc::Status::kOk); EXPECT_EQ(actual.report.status,sc::T3IntegrationStatus::NonFiniteArithmetic);
      EXPECT_NE(actual.report.sample,UINT32_MAX);
    }
    ASSERT_EQ(cudaMemcpy(&device.data->t3,&good,sizeof(good),cudaMemcpyHostToDevice),cudaSuccess);
    RunT3<<<1,1>>>(device.data,false); ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&actual,&device.data->t3_read,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.report.status,sc::T3IntegrationStatus::Ok); CheckT3(good,2,actual.result,clean.result);
  }
}
} // namespace
