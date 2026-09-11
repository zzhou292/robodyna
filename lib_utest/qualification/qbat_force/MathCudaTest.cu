// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cuda_runtime.h>
namespace qbat_force_test {
__global__ void Evaluate(const qb::Reference* r,const qb::Material* m,const qb::Failure* failure,
    const qb::History* accepted,const qb::PrescribedInterval* in,qb::ForceTrial* out,qb::Status* status) {
  *status=qb::EvaluateForce(*r,*m,*failure,*accepted,*in,*out);
}
template<class T> struct Device {
  T* p=nullptr;
  Device() { EXPECT_EQ(cudaMalloc(&p,sizeof(T)),cudaSuccess); }
  ~Device() { if(p) cudaFree(p); }
  void Put(const T& x) { ASSERT_EQ(cudaMemcpy(p,&x,sizeof(T),cudaMemcpyHostToDevice),cudaSuccess); }
  T Get() const { T x; EXPECT_EQ(cudaMemcpy(&x,p,sizeof(T),cudaMemcpyDeviceToHost),cudaSuccess); return x; }
  std::array<unsigned char,sizeof(T)> RawBytes() const {
    std::array<unsigned char,sizeof(T)> bytes{};
    EXPECT_EQ(cudaMemcpy(bytes.data(),p,sizeof(T),cudaMemcpyDeviceToHost),cudaSuccess);
    return bytes;
  }
};
void DeviceEqual(const qb::ForceTrial& a,const qb::ForceTrial& b) {
  const auto x=StateValues(a.proposed_history.data()),y=StateValues(b.proposed_history.data());
  for(unsigned i=0;i<x.size();++i) Close(x[i],y[i],1e-10,2e-12);
  for(unsigned i=0;i<4;++i) {
    Close(a.internal_force_n[i].x,b.internal_force_n[i].x,1e-10,2e-12);
    Close(a.internal_force_n[i].y,b.internal_force_n[i].y,1e-10,2e-12);
    Close(a.internal_force_n[i].z,b.internal_force_n[i].z,1e-10,2e-12);
    EXPECT_EQ(a.internal_couple_nm[i].x,b.internal_couple_nm[i].x);
    EXPECT_EQ(a.internal_couple_nm[i].y,b.internal_couple_nm[i].y);
    EXPECT_EQ(a.internal_couple_nm[i].z,b.internal_couple_nm[i].z);
    Close(a.point[i].thickness_after_m,b.point[i].thickness_after_m,1e-16,2e-12);
  }
}
TEST(QbatForceCuda, ActualFourPointIndependentDeviceHistoryAndRemovalPackets) {
  Fixture f;
  Device<qb::Reference> reference; reference.Put(f.reference);
  Device<qb::Material> material; material.Put(f.material);
  Device<qb::Failure> failure; failure.Put(f.failure);
  Device<qb::History> accepted;
  Device<qb::PrescribedInterval> interval;
  Device<qb::ForceTrial> trial;
  Device<qb::Status> status;
  auto host=f.Virgin();
  accepted.Put(host);
  for(unsigned step=0;step<256;++step) {
    SCOPED_TRACE(step);
    auto in=Path(f,step);
    qb::ForceTrial expected;
    ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,host,in,expected),qb::Status::kSuccess);
    interval.Put(in);
    Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(status.Get(),qb::Status::kSuccess);
    const auto actual=trial.Get();
    DeviceEqual(actual,expected);
    accepted.Put(actual.proposed_history);
    host=expected.proposed_history;
  }
  for(unsigned last=0;last<4;++last) {
    auto h=NearRemoval(f,last);
    accepted.Put(h);
    for(unsigned step=0;step<2;++step) {
      const auto in=Path(f,step);
      interval.Put(in);
      qb::ForceTrial expected;
      ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,h,in,expected),qb::Status::kSuccess);
      Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(status.Get(),qb::Status::kSuccess);
      const auto actual=trial.Get();
      DeviceEqual(actual,expected);
      accepted.Put(actual.proposed_history);
      h=expected.proposed_history;
    }
  }
}
TEST(QbatForceCuda, LateFourthPointOverflowPreservesBothBuffersAndExactRetry) {
  Fixture f;
  Device<qb::Reference> reference; reference.Put(f.reference);
  Device<qb::Material> material; material.Put(f.material);
  Device<qb::Failure> failure; failure.Put(f.failure);
  Device<qb::History> accepted;
  Device<qb::PrescribedInterval> interval; interval.Put(Path(f,0));
  Device<qb::ForceTrial> trial;
  Device<qb::Status> status;
  auto good=f.Virgin();
  accepted.Put(good);
  Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(status.Get(),qb::Status::kSuccess);
  const auto first=trial.Get();
  auto bad=good.data();
  bad.point[3].material.stress[0]=1e308;
  qb::History imported;
  ASSERT_EQ(qb::PreparePrescribedHistory(f.reference,f.material,f.failure,bad,{0,0},imported),
      qb::Status::kSuccess);
  accepted.Put(imported);
  const auto prior_output=trial.RawBytes();
  const auto prior_accepted=accepted.RawBytes();
  Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_NE(status.Get(),qb::Status::kSuccess);
  EXPECT_EQ(trial.RawBytes(),prior_output);
  EXPECT_EQ(accepted.RawBytes(),prior_accepted);
  auto cached=good.data();
  cached.point[3].force_stress_pa[0]=1e308;
  cached.force_stress_pa[0]=.25*1e308;
  cached.internal_work_j[0]=std::numeric_limits<double>::max();
  qb::History energy_overflow;
  ASSERT_EQ(qb::PreparePrescribedHistory(f.reference,f.material,f.failure,cached,{0,0},energy_overflow),
      qb::Status::kSuccess);
  accepted.Put(energy_overflow);
  const auto energy_before=accepted.RawBytes();
  Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_NE(status.Get(),qb::Status::kSuccess);
  EXPECT_EQ(trial.RawBytes(),prior_output);
  EXPECT_EQ(accepted.RawBytes(),energy_before);
  accepted.Put(good);
  auto wrong=Path(f,0);
  wrong.sample_index=2;
  interval.Put(wrong);
  Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_NE(status.Get(),qb::Status::kSuccess);
  EXPECT_EQ(trial.RawBytes(),prior_output);
  interval.Put(Path(f,0));
  Evaluate<<<1,1>>>(reference.p,material.p,failure.p,accepted.p,interval.p,trial.p,status.p);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(status.Get(),qb::Status::kSuccess);
  const auto retry=trial.Get();
  EXPECT_EQ(StateValues(retry.proposed_history.data()),StateValues(first.proposed_history.data()));
  for(unsigned i=0;i<4;++i) {
    EXPECT_DOUBLE_EQ(retry.internal_force_n[i].x,first.internal_force_n[i].x);
    EXPECT_DOUBLE_EQ(retry.internal_force_n[i].y,first.internal_force_n[i].y);
    EXPECT_DOUBLE_EQ(retry.internal_force_n[i].z,first.internal_force_n[i].z);
  }
}
} // namespace qbat_force_test
