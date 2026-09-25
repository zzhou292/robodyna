// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <cstring>
#include <memory>
namespace normal_activation_test {
namespace {
constexpr unsigned Nodes=64,Mains=16,Rows=8,References=64,Incidences=128,Optimized=64;
struct Image {
  a::Input input;
  l::Node nodes[Nodes];l::Main mains[Mains];l::Secondary secondary[Rows];l::NormalReference normals[References];
  l::OptimizedRow rows[Rows];std::uint32_t optimized[Optimized],free[Mains];
  std::uint32_t no[References+1],ne[Incidences],ro[Rows+1],re[Incidences];
};
struct Output {a::Status status=a::Status::InvalidInput;std::uint32_t mains[Mains],nodes[Nodes];};
__device__ a::Input Bind(const Image& x) {
  auto in=x.input;in.source.nodes=x.nodes;in.source.mains=x.mains;in.source.secondary=x.secondary;in.source.normals=x.normals;
  in.source.normal_to_main.offsets=x.no;in.source.normal_to_main.entries=x.ne;
  in.source.removed_main_by_secondary.offsets=x.ro;in.source.removed_main_by_secondary.entries=x.re;
  in.rows=x.rows;in.optimized_main_ids=x.optimized;in.free_main_ids=x.free;return in;
}
__global__ void Admit(const Image* image,Output* out,a::Limits cap) {
  if(blockIdx.x||threadIdx.x)return;
  const auto in=Bind(*image);
  out->status=a::detail::Validate(in,cap,{out->mains,in.source.main_count,out->nodes,in.source.node_count});
}
__global__ void Clear(const Image* image,Output* out) {
  if(out->status!=a::Status::Ok)return;
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<image->input.source.main_count;i+=blockDim.x*gridDim.x)out->mains[i]=0;
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<image->input.source.node_count;i+=blockDim.x*gridDim.x)out->nodes[i]=0;
}
struct Writer {
  Output* out;
  __device__ void Main(std::uint32_t i){atomicExch(out->mains+i,1u);}
  __device__ void Node(std::uint32_t i){atomicExch(out->nodes+i,1u);}
};
__global__ void Emit(const Image* image,Output* out,bool reverse) {
  if(out->status!=a::Status::Ok)return;
  const auto in=Bind(*image);const auto total=in.row_count+in.optimized_count+in.free_count;
  Writer writer{out};
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<total;i+=blockDim.x*gridDim.x) {
    const auto item=reverse?total-1-i:i;
    if(item<in.row_count)a::detail::RetainedRow(in,item,writer);
    else if(item<in.row_count+in.optimized_count)a::detail::OptimizedMain(in,item-in.row_count,writer);
    else a::detail::FreeMain(in,item-in.row_count-in.optimized_count,writer);
  }
}
void Check(cudaError_t code){if(code!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(code));}
template<class T,std::size_t N>void Copy(T(&target)[N],const std::vector<T>& source) {
  if(source.size()>N)throw std::runtime_error("CUDA activation fixture exceeds typed extent");
  std::copy(source.begin(),source.end(),target);
}
struct Device {
  Image* image=nullptr;Output* out=nullptr;cudaStream_t stream=nullptr;
  Device(){try{Check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));Check(cudaMalloc(&image,sizeof(Image)));Check(cudaMalloc(&out,sizeof(Output)));}catch(...){Release();throw;}}
  ~Device(){Release();}
  Device(const Device&)=delete;Device& operator=(const Device&)=delete;
  void Release(){if(stream)cudaStreamSynchronize(stream);if(out)cudaFree(out);if(image)cudaFree(image);if(stream)cudaStreamDestroy(stream);out=nullptr;image=nullptr;stream=nullptr;}
  Output Evaluate(const Fixture& f,unsigned threads,bool reverse,a::Limits limits=Fixture::Limits()) {
    auto host=std::make_unique<Image>();host->input=f.Input();
    Copy(host->nodes,f.scene.nodes);Copy(host->mains,f.scene.mains);Copy(host->secondary,f.scene.secondary);Copy(host->normals,f.scene.normals);
    Copy(host->rows,f.rows);Copy(host->optimized,f.optimized);Copy(host->free,f.free);
    Copy(host->no,f.scene.normal_offsets);Copy(host->ne,f.scene.normal_entries);
    Copy(host->ro,f.scene.removed_offsets);Copy(host->re,f.scene.removed_entries);
    Output result;std::fill(std::begin(result.mains),std::end(result.mains),17u);std::fill(std::begin(result.nodes),std::end(result.nodes),29u);
    auto after=std::make_unique<Image>();
    struct Drain{cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{stream};
    Check(cudaMemcpyAsync(image,host.get(),sizeof(Image),cudaMemcpyHostToDevice,stream));
    Check(cudaMemcpyAsync(out,&result,sizeof(Output),cudaMemcpyHostToDevice,stream));
    Admit<<<1,1,0,stream>>>(image,out,limits);Check(cudaGetLastError());
    Clear<<<2,threads,0,stream>>>(image,out);Check(cudaGetLastError());
    Emit<<<2,threads,0,stream>>>(image,out,reverse);Check(cudaGetLastError());
    Check(cudaMemcpyAsync(&result,out,sizeof(Output),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(after.get(),image,sizeof(Image),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));EXPECT_EQ(std::memcmp(host.get(),after.get(),sizeof(Image)),0);
    return result;
  }
};
}
TEST(NormalActivationCuda, ParallelIntegerUnionMatchesNativeAcrossOrdersAndLaunches) {
  Device gpu;
  for(unsigned scenario=0;scenario<12;++scenario) {
    auto f=Scenario(scenario);const auto expected=Oracle(f.Input());
    for(unsigned threads:{1u,7u,32u})for(bool reverse:{false,true})for(unsigned repeat=0;repeat<4;++repeat) {
      SCOPED_TRACE(scenario);SCOPED_TRACE(threads);SCOPED_TRACE(reverse);
      const auto result=gpu.Evaluate(f,threads,reverse);ASSERT_EQ(result.status,a::Status::Ok);
      EXPECT_EQ(std::vector<std::uint32_t>(result.mains,result.mains+f.scene.mains.size()),expected.main_active);
      EXPECT_EQ(std::vector<std::uint32_t>(result.nodes,result.nodes+f.scene.nodes.size()),expected.node_tag);
      EXPECT_EQ(f.free,expected.free_main_ids);
    }
  }
}
TEST(NormalActivationCuda, RejectedAdmissionPreservesBothArraysAndValidRetry) {
  Device gpu;const auto clean=Scenario(7);
  for(unsigned fault=0;fault<4;++fault) {
    auto f=clean;auto limits=Fixture::Limits();
    if(fault==0)limits.nodes=6;
    if(fault==1)f.free.pop_back();
    if(fault==2)f.rows[0].stage.report.count_complete=true;
    if(fault==3)f.scene.mains.back().coefficient=std::numeric_limits<double>::infinity();
    const auto result=gpu.Evaluate(f,32,false,limits);EXPECT_NE(result.status,a::Status::Ok);
    for(auto value:result.mains)EXPECT_EQ(value,17u);for(auto value:result.nodes)EXPECT_EQ(value,29u);
  }
  const auto result=gpu.Evaluate(clean,32,true);ASSERT_EQ(result.status,a::Status::Ok);
  const auto expected=Oracle(clean.Input());
  EXPECT_EQ(std::vector<std::uint32_t>(result.mains,result.mains+clean.scene.mains.size()),expected.main_active);
  EXPECT_EQ(std::vector<std::uint32_t>(result.nodes,result.nodes+clean.scene.nodes.size()),expected.node_tag);
}
} // namespace normal_activation_test
