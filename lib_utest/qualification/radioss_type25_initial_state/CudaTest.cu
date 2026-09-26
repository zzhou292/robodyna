// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "lib_src/collision/radioss_type25/initial_state/Rows.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <memory>
#include <limits>
namespace initial_state_test {
namespace device {
constexpr std::size_t Nodes=256,Mains=160,References=640,Rows=128,Pairs=128;
struct Image {
  init::RowInput in;
  n::selection::lifecycle::Node nodes[Nodes];
  n::selection::lifecycle::Main mains[Mains];
  n::selection::lifecycle::Secondary secondary[Rows];
  n::selection::lifecycle::NormalReference references[References];
  n::candidates::Pair pairs[Pairs];std::uint64_t offsets[Rows+1];double x[3*Nodes];
};
struct Values {init::Winner before[Rows],after[Rows];};
struct State {init::RowReport report;init::RowReport rows[Rows];Values staged,published;};
__device__ init::RowInput Bind(const Image& image) {
  auto in=image.in;in.source.nodes=image.nodes;in.source.mains=image.mains;
  in.source.secondary=image.secondary;in.source.normals=image.references;
  in.native_positions.data=image.x;in.pairs=image.pairs;in.row_offsets=image.offsets;return in;
}
__global__ void Admit(const Image* image,State* state) {
  if(blockIdx.x||threadIdx.x)return;
  const auto& in=image->in;const auto& s=in.source;
  state->report={init::Status::Ok,SIZE_MAX};
  if(!s.node_count||s.node_count>Nodes||!s.main_count||s.main_count>Mains||
      !s.secondary_count||s.secondary_count>Rows||s.normal_count>References||in.pair_count>Pairs||
      in.offset_count!=s.secondary_count+1)state->report={init::Status::InvalidInput,SIZE_MAX};
}
__global__ void Reduce(const Image* image,State* state,bool reverse) {
  if(state->report.status!=init::Status::Ok)return;
  const auto in=Bind(*image);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.source.secondary_count;i+=gridDim.x*blockDim.x) {
    const auto row=reverse?in.source.secondary_count-1-i:i;
    state->rows[row]=init::ReduceRow(in,row,&state->staged.before[row]);
  }
}
__global__ void Fold(const Image* image,State* state) {
  if(blockIdx.x||threadIdx.x||state->report.status!=init::Status::Ok)return;
  for(std::size_t i=0;i<image->in.source.secondary_count;++i)
    if(state->rows[i].status!=init::Status::Ok){state->report=state->rows[i];return;}
}
__global__ void Pwr(const Image* image,State* state) {
  if(state->report.status!=init::Status::Ok)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<image->in.source.secondary_count;i+=gridDim.x*blockDim.x) {
    state->staged.after[i]=state->staged.before[i];init::FinalizeInacti5(state->staged.after[i]);
  }
}
__global__ void Publish(const Image* image,State* state) {
  if(state->report.status!=init::Status::Ok)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<image->in.source.secondary_count;i+=gridDim.x*blockDim.x) {
    state->published.before[i]=state->staged.before[i];state->published.after[i]=state->staged.after[i];
  }
}
void Pack(const Fixture& f,int sharp,Image& out) {
  const auto count=f.mesh.ids.size(),g=f.topology.mains.size(),s=f.secondary.size(),r=f.topology.starter_references.size();
  if(count>Nodes||g>Mains||s>Rows||r>References||f.candidates.size()>Pairs)throw std::runtime_error("Initial-state CUDA fixture bound");
  out.in.profile.sharp=sharp;auto& source=out.in.source;
  source.node_count=count;source.main_count=g;source.secondary_count=s;source.normal_count=r;source.generation=7;
  out.in.native_positions={out.x,std::uint32_t(count),3,1};out.in.offset_count=s+1;
  for(std::size_t i=0;i<count;++i)out.nodes[i].source_id=f.mesh.ids[i];
  std::copy(f.mesh.positions.begin(),f.mesh.positions.end(),out.x);
  std::copy(f.topology.starter_references.begin(),f.topology.starter_references.end(),out.references);
  for(std::size_t i=0;i<s;++i)out.secondary[i]={f.secondary[i].node,f.secondary[i].stiffness,f.secondary[i].gap,0};
  for(std::size_t i=0;i<g;++i) {
    const auto& a=f.topology.mains[i];auto& b=out.mains[i];b.global_id=a.global_id;b.segment_type=a.segment_type;
    b.coefficient=f.mesh.coefficients[i];
    for(unsigned k=0;k<4;++k){b.nodes[k]=a.nodes[k];b.normal_slot[k]=f.topology.starter_normals[4*i+k];
      b.normal_reference[k]=a.normal_reference[k];b.neighbors[k]=a.neighbors[k];b.gap[k]=f.gaps[i][k];}
  }
  std::size_t pairs=0;out.offsets[0]=0;
  for(std::size_t row=0;row<s;++row) {
    // Test packing only. The source-sized producer uses GPU inventory offsets.
    for(const auto& pair:f.candidates)if(std::size_t(pair[0]-1)==row)
      out.pairs[pairs++]={std::uint32_t(row),std::uint32_t(pair[1]-1)};
    out.offsets[row+1]=pairs;
  }
  out.in.pair_count=pairs;
}
class InitialStateCuda:public type25_friction_test::PacketCuda<> {
 protected:
  Values last_{};bool initialized_=false;
  static void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
  std::unique_ptr<State> Run(const Image& host,unsigned threads,bool reverse) {
    static_assert(sizeof(Image)<=Capacity*RowBytes&&sizeof(State)<=Capacity*RowBytes);
    auto start=std::make_unique<State>();auto result=std::make_unique<State>();auto copied=std::make_unique<Image>();
    if(!initialized_){std::memset(&last_,0xa5,sizeof(last_));initialized_=true;}start->published=last_;
    auto* image=static_cast<Image*>(input);auto* state=static_cast<State*>(output);
    type25_friction_test::Drain drain{stream};
    Check(cudaMemcpyAsync(image,&host,sizeof(Image),cudaMemcpyHostToDevice,stream));
    Check(cudaMemcpyAsync(state,start.get(),sizeof(State),cudaMemcpyHostToDevice,stream));
    Admit<<<1,1,0,stream>>>(image,state);Check(cudaGetLastError());
    Reduce<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Fold<<<1,1,0,stream>>>(image,state);Check(cudaGetLastError());
    Pwr<<<3,threads,0,stream>>>(image,state);Check(cudaGetLastError());
    Publish<<<3,threads,0,stream>>>(image,state);Check(cudaGetLastError());
    Check(cudaMemcpyAsync(result.get(),state,sizeof(State),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(copied.get(),image,sizeof(Image),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));EXPECT_EQ(std::memcmp(copied.get(),&host,sizeof(Image)),0);
    if(result->report.status!=init::Status::Ok)EXPECT_EQ(std::memcmp(&result->published,&last_,sizeof(last_)),0);
    else last_=result->published;
    return result;
  }
};
}
using device::InitialStateCuda;
TEST_F(InitialStateCuda, WholeNativeWarmHistoryMatchesAcrossLaunchWidthsAndRowOrders) {
  RecordProperty("explicit_device_bytes",std::to_string(2*Capacity*RowBytes));
  for(unsigned mode:{0u,1u,2u})for(int sharp:{1,2})for(unsigned threads:{1u,7u,32u,64u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(sharp);
    SCOPED_TRACE(threads);
    SCOPED_TRACE(reverse);
    Fixture f(mode,true);auto image=std::make_unique<device::Image>();device::Pack(f,sharp,*image);
    const auto expected=InitialHistory(f.Input(),f.topology,f.gaps,f.candidates,sharp);const auto actual=Run(*image,threads,reverse);
    ASSERT_EQ(actual->report.status,init::Status::Ok);
    for(std::size_t row=0;row<f.secondary.size();++row) {
      Same(actual->published.before[row],expected.before_pwr[row]);Same(actual->published.after[row],expected.rows[row]);
      EXPECT_EQ(Bits(actual->published.before[row].distance_squared),Bits(expected.nearest_distance[row]));
    }
  }
}
TEST_F(InitialStateCuda, NativeRoleEligibilityNegativeStiffnessAndSignedZeroGapBitsMatch) {
  for(int role_case:{0,1,2,3})for(bool zeros:{false,true}) {
    SCOPED_TRACE(role_case);
    SCOPED_TRACE(zeros);
    Fixture f;
    const int encoded=int(f.topology.mains.size()+1);
    for(auto& main:f.topology.mains)main.segment_type=role_case==0?0:role_case==1?1:role_case==2?encoded:-encoded;
    std::fill(f.mesh.coefficients.begin(),f.mesh.coefficients.end(),-210000.);
    if(zeros) {
      for(auto& row:f.secondary)row.gap=-0.;
      for(auto& gap:f.gaps)gap.fill(-0.);
      for(const auto& row:f.secondary)f.mesh.positions[3*row.node+2]=0.;
    }
    auto image=std::make_unique<device::Image>();device::Pack(f,1,*image);
    const auto expected=InitialHistory(f.Input(),f.topology,f.gaps,f.candidates);const auto actual=Run(*image,7,true);
    ASSERT_EQ(actual->report.status,init::Status::Ok);
    for(std::size_t row=0;row<f.secondary.size();++row) {
      Same(actual->published.before[row],expected.before_pwr[row]);Same(actual->published.after[row],expected.rows[row]);
      EXPECT_EQ(Bits(actual->published.before[row].distance_squared),Bits(expected.nearest_distance[row]));
    }
  }
}
TEST_F(InitialStateCuda, LateValueFailureAndEmptyRowWrongProfilePreservePublishedStateThenRetry) {
  Fixture f(1,true);auto image=std::make_unique<device::Image>();device::Pack(f,1,*image);
  ASSERT_EQ(Run(*image,32,false)->report.status,init::Status::Ok);
  auto invalid=std::make_unique<device::Image>(*image);invalid->x[3*f.secondary.back().node]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Run(*invalid,7,true)->report.status,init::Status::InvalidInput);
  *invalid=*image;invalid->in.profile.initial_penetration=0;
  invalid->in.pair_count=0;std::fill_n(invalid->offsets,invalid->in.offset_count,0);
  EXPECT_EQ(Run(*invalid,64,false)->report.status,init::Status::UnsupportedProfile);
  EXPECT_EQ(Run(*image,1,true)->report.status,init::Status::Ok);
}
}
