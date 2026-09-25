#pragma once
#include "Fixture.h"
#include <cuda_runtime.h>
#include <stdexcept>
namespace type25_search_test {
inline void Cuda(cudaError_t error){if(error!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(error));}
struct Stream {cudaStream_t value=nullptr;Stream(){Cuda(cudaStreamCreateWithFlags(&value,cudaStreamNonBlocking));}
  ~Stream(){if(value){cudaStreamSynchronize(value);cudaStreamDestroy(value);}}};
struct DeviceFixture {
  Stream stream;
  s::Maintenance owner;
  Fixture fixture;
  double* data=nullptr;
  s::Current current;
  explicit DeviceFixture(bool compact=true,bool gaps=true,bool si=false,std::size_t nodes=32)
      : fixture(compact,gaps,nodes) {
    if(si)fixture.ToSi();
    try {
      Cuda(cudaMalloc(reinterpret_cast<void**>(&data),Bytes()));
      Upload();
    } catch (...) {if(data)cudaFree(data);data=nullptr;throw;}
  }
  ~DeviceFixture(){cudaStreamSynchronize(stream.value);if(data)cudaFree(data);}
  std::size_t Bytes() const {return sizeof(double)*(fixture.positions.size()+fixture.velocities.size()+fixture.stiffness.size()+fixture.gaps.size());}
  void Upload() {
    current=fixture.Current();std::size_t offset=0;
    for(const auto* row:{&fixture.positions,&fixture.velocities,&fixture.stiffness,&fixture.gaps}) {
      Cuda(cudaMemcpy(data+offset,row->data(),row->size()*sizeof(double),cudaMemcpyHostToDevice));offset+=row->size();
    }
    current.positions.data=data;current.velocities.data=data+fixture.positions.size();
    current.secondary_stiffness=current.velocities.data+fixture.velocities.size();
    if(current.main_gap_count)current.main_gaps=current.secondary_stiffness+fixture.stiffness.size();
  }
  void Initialize(){ASSERT_EQ(owner.Initialize(fixture.source,{},stream.value),s::Status::Ok);}
  s::ReferenceToken Stage() {
    s::ReferenceToken token;
    if(owner.StageReference(current,token)!=s::Status::Ok)throw std::runtime_error("Reference stage failed");return token;
  }
  void Reference(){const auto token=Stage();ASSERT_EQ(owner.PublishReference(token),s::Status::Ok);}
  void Compare(const s::Current& host_reference,double dt=1e-5,bool forced=false) {
    s::Report result;
    ASSERT_EQ(owner.Evaluate(current,dt,forced,result),s::Status::Ok);
    const auto e=NativeExtrema(fixture.source,fixture.Current(),host_reference);
    const double native_dt=fixture.source.input_units==s::InputUnits::Si ? dt/fixture.source.units.time_s : dt;
    Same(result.extrema,e);Same(result.budget,NativeBudget(e,fixture.source.margin,native_dt,forced));
    EXPECT_EQ(result.reference_generation,owner.reference_generation());
    EXPECT_EQ(result.stamp.epoch,fixture.stamp.epoch);
  }
};
class Type25SearchCuda:public ::testing::Test {
 void SetUp() override {int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);}
};
} // namespace type25_search_test
