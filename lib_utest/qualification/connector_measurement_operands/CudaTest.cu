// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
#include <stdexcept>
namespace connector_operand_test {
namespace {
inline void Check(cudaError_t result) { if(result!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(result)); }
struct DeviceMemory {
  std::vector<void*> blocks;
  ~DeviceMemory() { for(auto* p:blocks) cudaFree(p); }
  template<class T> T* Copy(const T* input,std::size_t count) {
    void* allocation=nullptr; Check(cudaMalloc(&allocation,count*sizeof(T))); blocks.push_back(allocation);
    Check(cudaMemcpy(allocation,input,count*sizeof(T),cudaMemcpyHostToDevice)); return static_cast<T*>(allocation);
  }
};
template<class T> struct Rig {
  DeviceMemory memory;
  typename T::State* current=nullptr;
  typename T::State* reference=nullptr;
  fe::NodalPreparedView view;
  typename T::Evaluation* trial_values=nullptr;
  typename T::Status* statuses=nullptr;
  std::size_t count=0;
  explicit Rig(Fixture<T>& fixture,bool unavailable=false) {
    auto state=fixture.state;
    state.control=StaleControl<T>();
    count=fixture.status.size();
    auto* accepted=unavailable ? nullptr : memory.Copy(fixture.accepted.data(),count);
    auto* trial=unavailable ? nullptr : memory.Copy(fixture.trial.data(),count);
    T::Bind(state,count,accepted,trial);
    state.model.elements=unavailable ? nullptr : memory.Copy(fixture.elements.data(),count);
    trial_values=trial;
    statuses=memory.Copy(fixture.status.data(),count);
    state.candidate_status=statuses;
    state.measurement=memory.Copy(fixture.operands.data(),count);
    current=memory.Copy(&state,1); reference=memory.Copy(&state,1);
    view=fixture.view;
    if(unavailable) view={};
    else {
      view.base_kinematics.position_xyz=memory.Copy(fixture.x0.data(),fixture.x0.size());
      view.kinematics.position_xyz=memory.Copy(fixture.x.data(),fixture.x.size());
      view.base_kinematics.velocity_xyz=memory.Copy(fixture.v0.data(),fixture.v0.size());
      view.kinematics.velocity_xyz=memory.Copy(fixture.v.data(),fixture.v.size());
      view.base_kinematics.angular_velocity_xyz=memory.Copy(fixture.omega0.data(),fixture.omega0.size());
      view.kinematics.angular_velocity_xyz=memory.Copy(fixture.omega.data(),fixture.omega.size());
    }
  }
  void UpdateTrial(const Fixture<T>& fixture) {
    ASSERT_EQ(fixture.status.size(),count);
    Check(cudaMemcpy(trial_values,fixture.trial.data(),count*sizeof(*trial_values),cudaMemcpyHostToDevice));
    Check(cudaMemcpy(statuses,fixture.status.data(),count*sizeof(*statuses),cudaMemcpyHostToDevice));
  }
};
template<class T> __global__ void Stage(typename T::State* state,fe::NodalPreparedView view,std::size_t count,bool reverse) {
  for(std::size_t ordinal=blockIdx.x*blockDim.x+threadIdx.x;ordinal<count;ordinal+=blockDim.x*gridDim.x) {
    const auto e=reverse ? count-1-ordinal : ordinal;
    state->measurement[e]=T::Prepare(*state,view,e);
  }
}
template<class T> __global__ void Finish(typename T::State* state,fe::NodalPreparedView view,
    typename T::Diagnostics seed,bool reference) {
  if(reference) T::Reference(*state,view,seed); else T::Current(*state,view,seed);
}
template<class T> void CompareDevice(Fixture<T>& fixture,Rig<T>& rig,unsigned repeats=1) {
  Finish<T><<<1,1>>>(rig.reference,rig.view,fixture.seed,true); Check(cudaPeekAtLastError());
  typename T::State expected;
  Check(cudaMemcpy(&expected,rig.reference,sizeof(expected),cudaMemcpyDeviceToHost));
  for(unsigned i=0;i<repeats;++i) {
    Stage<T><<<3,32u<<(i%3)>>>(rig.current,rig.view,fixture.status.size(),i%2!=0); Check(cudaPeekAtLastError());
    Finish<T><<<1,1>>>(rig.current,rig.view,fixture.seed,false); Check(cudaPeekAtLastError());
    typename T::State actual;
    Check(cudaMemcpy(&actual,rig.current,sizeof(actual),cudaMemcpyDeviceToHost));
    Same<T>(actual.control,expected.control);
    std::vector<typename T::Measurement> operands(fixture.status.size());
    Check(cudaMemcpy(operands.data(),actual.measurement,operands.size()*sizeof(operands[0]),cudaMemcpyDeviceToHost));
    for(std::size_t e=0;e<operands.size();++e) if(fixture.status[e]!=T::Status::Success) {
      for(double x:operands[e].kick) EXPECT_EQ(Bits(x),Bits(0));
      for(double x:operands[e].drift) EXPECT_EQ(Bits(x),Bits(0));
      for(double x:operands[e].work) EXPECT_EQ(Bits(x),Bits(0));
    }
  }
}
template<class T> void CompareDevice(Fixture<T>& fixture,unsigned repeats=1,bool unavailable=false) {
  Rig<T> rig(fixture,unavailable);
  CompareDevice(fixture,rig,repeats);
}
template<class T> class ConnectorOperandCuda : public ::testing::Test {
  void SetUp() override { int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0); }
};
using Families=::testing::Types<Type13,Type25>;
TYPED_TEST_SUITE(ConnectorOperandCuda,Families);
TYPED_TEST(ConnectorOperandCuda, SourceOrderedDiagnosticsMatchFrozenAcrossSchedulingAndRepeatedUse) {
  Fixture<TypeParam> fixture(257); CompareDevice(fixture,16);
}
TYPED_TEST(ConnectorOperandCuda, CompleteElementFailureScanWinsOverEarlierDerivedOverflowThenFreshRetry) {
  Fixture<TypeParam> fixture(17);
  TypeParam::Work(fixture.trial[0],0)=DBL_MAX; TypeParam::Work(fixture.trial[1],0)=DBL_MAX;
  fixture.status.back()=TypeParam::Status::NonfiniteResult; CompareDevice(fixture,2);
  fixture.status.back()=TypeParam::Status::Success; CompareDevice(fixture,2);
  TypeParam::Work(fixture.trial[1],0)=-DBL_MAX; CompareDevice(fixture,2);
}
TYPED_TEST(ConnectorOperandCuda, FailedRowsDoNotReadUnavailableInputsAndOverwritePoisonedPackets) {
  Fixture<TypeParam> fixture(1031);
  for(auto& status:fixture.status) status=TypeParam::Status::DegenerateGeometry;
  for(auto& packet:fixture.operands) { for(auto& value:packet.work) value=NAN; for(auto& value:packet.kick) value=NAN; }
  CompareDevice(fixture,2,true);
}
TYPED_TEST(ConnectorOperandCuda, CompleteControlMultipleErrorsAndNonfiniteRepairReuseSameDeviceState) {
  Fixture<TypeParam> fixture(18); NonzeroIdentity(fixture);
  const auto original=fixture.trial;
  Rig<TypeParam> rig(fixture);
  CompareDevice(fixture,rig,2);
  TypeParam::Work(fixture.trial[0],0)=DBL_MAX;
  TypeParam::Work(fixture.trial[1],0)=DBL_MAX;
  fixture.status[7]=TypeParam::Status::DegenerateGeometry;
  fixture.status.back()=TypeParam::Status::NonfiniteResult;
  rig.UpdateTrial(fixture); CompareDevice(fixture,rig,2);
  fixture.status[7]=TypeParam::Status::Success;
  rig.UpdateTrial(fixture); CompareDevice(fixture,rig,2);
  fixture.status.back()=TypeParam::Status::Success;
  rig.UpdateTrial(fixture); CompareDevice(fixture,rig,2);
  for(double invalid : {double(NAN),double(INFINITY)}) {
    TypeParam::Work(fixture.trial[0],0)=invalid;
    rig.UpdateTrial(fixture); CompareDevice(fixture,rig,2);
  }
  std::copy(original.begin(),original.end(),fixture.trial.begin());
  rig.UpdateTrial(fixture); CompareDevice(fixture,rig,2);
}
} // namespace
} // namespace connector_operand_test
