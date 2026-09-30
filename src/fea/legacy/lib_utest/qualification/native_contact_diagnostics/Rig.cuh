// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include <cuda_runtime.h>
#include "DiagnosticBodies.h"
namespace native_diagnostic_test {
inline void Check(cudaError_t code) {
  if(code!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(code));
}
class Rig {
 public:
  explicit Rig(std::size_t count):shape_(count),count_(count) {
    try {
      Check(cudaStreamCreateWithFlags(&stream_,cudaStreamNonBlocking));
      for(unsigned i=0;i<2;++i) {
        Check(cudaMalloc(&arena_[i],shape_.layout.bytes));
        Check(cudaMemsetAsync(arena_[i],0,shape_.layout.bytes,stream_));
        device_[i]=rd::Bind(arena_[i],shape_.layout,shape_.source,shape_.limits);
      }
      Check(cudaStreamSynchronize(stream_));
    } catch(...){Release();throw;}
  }
  ~Rig(){Release();}
  Rig(const Rig&)=delete;Rig& operator=(const Rig&)=delete;
  rd::Control Compare(const Input& input,bool unread_operands=false) {
    if(input.flags.size()!=count_||input.slots.size()!=count_||input.response.size()!=count_)
      throw std::runtime_error("Diagnostic operand extent mismatch");
    for(auto& d:device_) {
      Copy(d.control,&input.seed,sizeof(input.seed));
      Copy(d.positive_flags,input.flags.data(),count_*sizeof(input.flags[0]));
      Copy(d.sorted_slots,input.slots.data(),count_*sizeof(input.slots[0]));
      Copy(d.responses,input.response.data(),count_*sizeof(input.response[0]));
    }
    auto reference=device_[0],current=device_[1];
    if(unread_operands) {
      reference.responses=current.responses=nullptr;
      reference.sorted_slots=current.sorted_slots=nullptr;
    }
    rd::diagnostic_reference::Diagnostics<<<1,1,0,stream_>>>(reference,count_,input.units);
    Check(cudaPeekAtLastError());
    rd::diagnostic_current::Diagnostics<<<1,1,0,stream_>>>(current,count_,input.units);
    Check(cudaPeekAtLastError());
    rd::Control expected,actual;
    Check(cudaMemcpyAsync(&expected,device_[0].control,sizeof(expected),cudaMemcpyDeviceToHost,stream_));
    Check(cudaMemcpyAsync(&actual,device_[1].control,sizeof(actual),cudaMemcpyDeviceToHost,stream_));
    Check(cudaStreamSynchronize(stream_));Same(actual,expected);
    EXPECT_EQ(actual.required_sliding,input.seed.required_sliding);
    EXPECT_EQ(actual.required_candidates,input.seed.required_candidates);EXPECT_EQ(actual.kept,input.seed.kept);
    // The four aggregate fields and failure key are the only permitted writes.
    if(count_) {
      std::vector<n::NativeFrictionResult> response(count_);
      std::vector<std::uint32_t> flags(count_),slots(count_);
      Check(cudaMemcpyAsync(response.data(),device_[1].responses,count_*sizeof(response[0]),cudaMemcpyDeviceToHost,stream_));
      Check(cudaMemcpyAsync(flags.data(),device_[1].positive_flags,count_*sizeof(flags[0]),cudaMemcpyDeviceToHost,stream_));
      Check(cudaMemcpyAsync(slots.data(),device_[1].sorted_slots,count_*sizeof(slots[0]),cudaMemcpyDeviceToHost,stream_));
      Check(cudaStreamSynchronize(stream_));
      EXPECT_EQ(std::memcmp(response.data(),input.response.data(),count_*sizeof(response[0])),0);
      EXPECT_EQ(flags,input.flags);EXPECT_EQ(slots,input.slots);
    }
    return actual;
  }
 private:
  void Copy(void* to,const void* from,std::size_t bytes) {
    if(bytes)Check(cudaMemcpyAsync(to,from,bytes,cudaMemcpyHostToDevice,stream_));
  }
  void Release() noexcept {
    if(stream_)cudaStreamSynchronize(stream_);
    for(auto*& p:arena_)if(p){cudaFree(p);p=nullptr;}
    if(stream_){cudaStreamDestroy(stream_);stream_=nullptr;}
  }
  LayoutFixture shape_;std::size_t count_;
  cudaStream_t stream_=nullptr;void* arena_[2]{};rd::Device device_[2];
};
class NativeDiagnosticCuda:public ::testing::Test {
  void SetUp() override {
    int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);
  }
};
} // namespace native_diagnostic_test
