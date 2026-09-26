// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cuda_runtime.h>
namespace qbat_private_test {
template<class T> struct Device {
  T* value=nullptr;
  Device(){EXPECT_EQ(cudaMalloc(&value,sizeof(T)),cudaSuccess);}
  ~Device(){if(value)cudaFree(value);}
  void Put(const T& data){ASSERT_EQ(cudaMemcpy(value,&data,sizeof(T),cudaMemcpyHostToDevice),cudaSuccess);}
  T Get() const {T data;EXPECT_EQ(cudaMemcpy(&data,value,sizeof(T),cudaMemcpyDeviceToHost),cudaSuccess);return data;}
};
__global__ void Private(const batch::Element* element,const qb::BatchResult* accepted,
    const qb::PrescribedInterval* interval,qb::BatchResult* output,qb::Status* status) {
  *status=batch::AdvanceIntoTrial(*element,*accepted,*interval,*output);
}
__global__ void Public(const batch::Element* element,const qb::BatchResult* accepted,
    const qb::PrescribedInterval* interval,qb::BatchResult* output,qb::Status* status) {
  *status=batch::Advance(*element,*accepted,*interval,*output);
}
TEST(QbatPrivateTrialCuda, IndependentDeviceHistoriesAndAllDefinedOutputsMatchPublicValue) {
  Fixture fixture;const auto element=Element(fixture);
  Device<batch::Element> model;model.Put(element);
  Device<qb::BatchResult> accepted,public_accepted,output,public_output;Device<qb::PrescribedInterval> interval;
  Device<qb::Status> private_status,public_status;
  for(unsigned mode=0;mode<5;++mode) {
    SCOPED_TRACE(mode);
    qb::BatchResult state;
    ASSERT_EQ(batch::InitializeResult(element,state),qb::Status::kSuccess);
    if(mode){const auto h=qbat_force_test::NearRemoval(fixture,mode-1);state.history=h.data();state.stamp=h.stamp();}
    accepted.Put(state);public_accepted.Put(state);
    for(unsigned step=0;step<(mode?3u:160u);++step) {
      SCOPED_TRACE(step);
      interval.Put(Path(fixture,step));qb::BatchResult dirty;Dirty(dirty);output.Put(dirty);
      Public<<<1,1>>>(model.value,public_accepted.value,interval.value,public_output.value,public_status.value);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      Private<<<1,1>>>(model.value,accepted.value,interval.value,output.value,private_status.value);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(public_status.Get(),qb::Status::kSuccess);ASSERT_EQ(private_status.Get(),qb::Status::kSuccess);
      const auto actual=output.Get();const auto expected=public_output.Get();
      EXPECT_EQ(ResultValues(actual),ResultValues(expected));
      accepted.Put(actual);public_accepted.Put(expected);
    }
  }
}
TEST(QbatPrivateTrialCuda, PrivatePartialFailureCannotAlterAcceptedAndCleanRetryClearsFields) {
  Fixture fixture;const auto element=Element(fixture);qb::BatchResult good;
  ASSERT_EQ(batch::InitializeResult(element,good),qb::Status::kSuccess);
  Device<batch::Element> model;model.Put(element);
  Device<qb::BatchResult> accepted,trial,expected;Device<qb::PrescribedInterval> interval;
  Device<qb::Status> status,reference_status;interval.Put(Path(fixture,0));accepted.Put(good);
  Public<<<1,1>>>(model.value,accepted.value,interval.value,expected.value,reference_status.value);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(reference_status.Get(),qb::Status::kSuccess);
  const auto reference=expected.Get();
  auto bad=good;bad.history.point[3].material.stress[0]=1e308;accepted.Put(bad);
  qb::BatchResult dirty;Dirty(dirty);trial.Put(dirty);
  Private<<<1,1>>>(model.value,accepted.value,interval.value,trial.value,status.value);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_NE(status.Get(),qb::Status::kSuccess);
  EXPECT_EQ(ResultValues(accepted.Get()),ResultValues(bad));
  accepted.Put(good);
  Private<<<1,1>>>(model.value,accepted.value,interval.value,trial.value,status.value);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(status.Get(),qb::Status::kSuccess);
  EXPECT_EQ(ResultValues(trial.Get()),ResultValues(reference));
  Private<<<1,1>>>(model.value,accepted.value,interval.value,accepted.value,status.value);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);EXPECT_EQ(status.Get(),qb::Status::kInvalidInput);
  EXPECT_EQ(ResultValues(accepted.Get()),ResultValues(good));
}
}
