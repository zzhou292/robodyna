// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cuda_runtime.h>
namespace t3_one_point_test {
__global__ void Evaluate(const t3::ReferenceData* reference,const t3::OnePointMaterial* material,
    const t3::OnePointFailure* failure,const t3::OnePointHistory* accepted,
    const t3::PrescribedInterval* in,t3::OnePointForceTrial* out,t3::Status* status) {
  *status=t3::EvaluateOnePointLaw44Force(*reference,*material,*failure,*accepted,*in,*out);
}
template<class T> struct Device {
  T* p=nullptr;
  Device() { EXPECT_EQ(cudaMalloc(&p,sizeof(T)),cudaSuccess); }
  ~Device() { if(p) cudaFree(p); }
  void Put(const T& x) { ASSERT_EQ(cudaMemcpy(p,&x,sizeof(T),cudaMemcpyHostToDevice),cudaSuccess); }
  T Get() const {
    T x;
    EXPECT_EQ(cudaMemcpy(&x,p,sizeof(T),cudaMemcpyDeviceToHost),cudaSuccess);
    return x;
  }
  std::array<unsigned char,sizeof(T)> RawBytes() const {
    std::array<unsigned char,sizeof(T)> bytes{};
    EXPECT_EQ(cudaMemcpy(bytes.data(),p,sizeof(T),cudaMemcpyDeviceToHost),cudaSuccess);
    return bytes;
  }
};
struct DeviceCase {
  Device<t3::ReferenceData> reference;
  Device<t3::OnePointMaterial> material;
  Device<t3::OnePointFailure> failure;
  Device<t3::OnePointHistory> accepted;
  Device<t3::PrescribedInterval> interval;
  Device<t3::OnePointForceTrial> trial;
  Device<t3::Status> status;
  explicit DeviceCase(const Fixture& f) {
    reference.Put(f.reference);
    material.Put(f.material);
    failure.Put(f.failure);
  }
  t3::Status Step(const t3::PrescribedInterval& input) {
    interval.Put(input);
    Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
    EXPECT_EQ(cudaGetLastError(),cudaSuccess);
    return status.Get();
  }
};
void DeviceEqual(const t3::OnePointForceTrial& a,const t3::OnePointForceTrial& b) {
  const auto x=StateValues(a.proposed_history.values()),y=StateValues(b.proposed_history.values());
  for(unsigned i=0;i<x.size();++i) EXPECT_NEAR(x[i],y[i],1e-10+2e-12*std::max(std::abs(x[i]),std::abs(y[i])));
  for(unsigned i=0;i<3;++i) {
    EXPECT_NEAR(a.internal_force[i].x,b.internal_force[i].x,1e-9);
    EXPECT_NEAR(a.internal_force[i].y,b.internal_force[i].y,1e-9);
    EXPECT_NEAR(a.internal_force[i].z,b.internal_force[i].z,1e-9);
    EXPECT_DOUBLE_EQ(a.internal_couple[i].x,b.internal_couple[i].x);
    EXPECT_DOUBLE_EQ(a.internal_couple[i].y,b.internal_couple[i].y);
    EXPECT_DOUBLE_EQ(a.internal_couple[i].z,b.internal_couple[i].z);
  }
  EXPECT_DOUBLE_EQ(a.diagnostics.native_sound_speed,b.diagnostics.native_sound_speed);
  EXPECT_NEAR(a.diagnostics.unscaled_element_dt,b.diagnostics.unscaled_element_dt,1e-15);
  EXPECT_NEAR(a.diagnostics.rotational_stiffness,b.diagnostics.rotational_stiffness,1e-10);
  EXPECT_EQ(a.removed_now,b.removed_now);
}
TEST(T3OnePointCuda, ActualIndependentDeviceHistoryCyclesAndRemoval) {
  Fixture f;
  DeviceCase device(f);
  auto host=f.Virgin();
  device.accepted.Put(host);
  for(unsigned step=0;step<256;++step) {
    SCOPED_TRACE(step);
    const auto in=Path(f,step);
    t3::OnePointForceTrial expected;
    ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,host,in,expected),t3::Status::kSuccess);
    ASSERT_EQ(device.Step(in),t3::Status::kSuccess);
    const auto actual=device.trial.Get();
    DeviceEqual(actual,expected);
    device.accepted.Put(actual.proposed_history);
    host=expected.proposed_history;
  }
  host=NearFailure(f);
  device.accepted.Put(host);
  for(unsigned step=0;step<4;++step) {
    t3::OnePointForceTrial expected;
    const auto in=Path(f,step);
    ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,host,in,expected),t3::Status::kSuccess);
    ASSERT_EQ(device.Step(in),t3::Status::kSuccess);
    const auto actual=device.trial.Get();
    DeviceEqual(actual,expected);
    device.accepted.Put(actual.proposed_history);
    host=expected.proposed_history;
  }
}
TEST(T3OnePointCuda, LateMaterialFailurePreservesBothBuffersAndExactRetry) {
  Fixture f;
  DeviceCase device(f);
  const auto good=f.Virgin();
  device.accepted.Put(good);
  ASSERT_EQ(device.Step(Path(f,0)),t3::Status::kSuccess);
  const auto first=device.trial.Get();
  const auto output_bytes=device.trial.RawBytes();
  auto bad=good.values();
  bad.point.stress[0]=1e308;
  bad.shell.stress[0]=1e308;
  bad.shell.material_stress[0]=1e308;
  t3::OnePointHistory huge;
  ASSERT_EQ(t3::PrepareOnePointLaw44History(f.reference,f.material,f.failure,bad,{0,0},huge),t3::Status::kSuccess);
  device.accepted.Put(huge);
  const auto accepted_bytes=device.accepted.RawBytes();
  ASSERT_NE(device.Step(Path(f,0)),t3::Status::kSuccess);
  EXPECT_EQ(device.trial.RawBytes(),output_bytes);
  EXPECT_EQ(device.accepted.RawBytes(),accepted_bytes);
  device.accepted.Put(good);
  auto wrong=Path(f,0);
  wrong.sample_index=2;
  EXPECT_NE(device.Step(wrong),t3::Status::kSuccess);
  EXPECT_EQ(device.trial.RawBytes(),output_bytes);
  ASSERT_EQ(device.Step(Path(f,0)),t3::Status::kSuccess);
  const auto retry=device.trial.Get();
  EXPECT_EQ(StateValues(retry.proposed_history.values()),StateValues(first.proposed_history.values()));
  for(unsigned i=0;i<3;++i) {
    EXPECT_DOUBLE_EQ(retry.internal_force[i].x,first.internal_force[i].x);
    EXPECT_DOUBLE_EQ(retry.internal_force[i].y,first.internal_force[i].y);
    EXPECT_DOUBLE_EQ(retry.internal_force[i].z,first.internal_force[i].z);
  }
}
} // namespace t3_one_point_test
