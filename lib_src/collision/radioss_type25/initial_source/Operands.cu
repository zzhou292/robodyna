// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../candidates/PenetrationFilter.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::initial_source::detail {
namespace {
namespace v=tl::math::fixed3;
namespace d=candidates::detail;
unsigned Blocks(std::size_t n){return unsigned(std::max<std::size_t>(1,std::min<std::size_t>(256,(n+127)/128)));}
__device__ void Fail(Device a,std::size_t row,Status status) {
  atomicMin(&a.control->failure,(static_cast<unsigned long long>(row)<<8)|unsigned(status));
}
__device__ double Distance(Vector a,Vector b){const auto q=v::Subtract(a,b);return ::sqrt(v::Dot(q,q));}
__global__ void MainOperands(Device a) {
  for(std::size_t m=blockIdx.x*blockDim.x+threadIdx.x;m<a.mains_count;m+=gridDim.x*blockDim.x) {
    auto& main=a.mains[m];Vector x[4];
    for(unsigned k=0;k<4;++k){x[k]=a.positions[main.nodes[k]];atomicExch(a.main_node_tags+main.nodes[k],1u);a.original_gap[4*m+k]=main.gap[k];}
    const double e1=Distance(x[0],x[1]),e2=Distance(x[0],x[3]),e3=Distance(x[2],x[1]),e4=Distance(x[3],x[2]);
    const bool solid=main.segment_type==0||main.segment_type>int(a.mains_count);
    if(solid) {
      const double largest=d::Max(d::Max(d::Max(e1,e2),e3),e4),gap=d::Min(largest,main.gap[0]);
      if(!d::Nonnegative(gap)){Fail(a,m,Status::NonfiniteResult);continue;}
      if(__double_as_longlong(gap)!=__double_as_longlong(main.gap[0]))atomicAdd(&a.control->changed_gaps,1ull);
      main.gap[0]=gap;
    }
    if(main.coefficient>0)for(unsigned k=0;k<4;++k) {
      const auto diff=v::Subtract(x[(k+1)%4],x[k]);const double square=v::Dot(diff,diff);
      if(!d::Nonnegative(square)){Fail(a,m,Status::NonfiniteResult);continue;}
      // Native squared edge lengths are nonnegative; integer bitwise maximum
      // is the same exact maximum, with no floating reduction association.
      const auto bits=static_cast<unsigned long long>(__double_as_longlong(square==0?0.:square));
      atomicMax(a.edge_squared+main.nodes[k],bits);atomicMax(a.edge_squared+main.nodes[(k+1)%4],bits);
      if(solid){atomicExch(a.solid_tags+main.nodes[k],1u);atomicExch(a.solid_tags+main.nodes[(k+1)%4],1u);}
    }
    if(a.support_solid[m]==UINT32_MAX) {
      std::uint64_t largest=0;std::uint32_t selected=UINT32_MAX;
      // Literal INSOL25 complete eight-slot branch: all four main node
      // memberships, then largest original native user EID, not storage order.
      const auto node=main.nodes[0];
      for(auto j=a.solid_offsets[node];j<a.solid_offsets[node+1];++j) {
        const auto index=a.solid_incidence[j];const auto& candidate=a.solids[index];bool all=true;
        for(auto corner:main.nodes){bool hit=false;for(auto raw:candidate.nodes)hit=hit||raw==corner;all=all&&hit;}
        if(all&&candidate.native_source_id>largest){largest=candidate.native_source_id;selected=index;}
      }
      a.support_solid[m]=selected;
    }
  }
}
__global__ void NodeOperands(Device a) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<a.secondary_count;i+=gridDim.x*blockDim.x) {
    const auto& row=a.secondary[i];
    double edge=0;if(row.coefficient!=0)edge=.5*::sqrt(__longlong_as_double(static_cast<long long>(a.edge_squared[row.node])));
    if(!d::Nonnegative(edge)){Fail(a,i,Status::NonfiniteResult);continue;}
    a.edge_length[i]=edge;a.large_tags[i]=0;
  }
}
__global__ void FoldOperands(Device a) {
  if(blockIdx.x||threadIdx.x||a.control->failure!=~0ull)return;
  std::uint64_t nodes=0,solid_nodes=0;double edge_sum=0,maximum_edge=0;
  double low[3]{1.e30,1.e30,1.e30},high[3]{-1.e30,-1.e30,-1.e30};
  for(std::size_t i=0;i<a.nodes_count;++i) {
    if(a.solid_tags[i])++solid_nodes;
  }
  for(std::size_t i=0;i<a.main_node_count;++i) {
    ++nodes;const auto x=a.positions[a.main_nodes[i]];
    low[0]=d::Min(low[0],x.x);low[1]=d::Min(low[1],x.y);low[2]=d::Min(low[2],x.z);
    high[0]=d::Max(high[0],x.x);high[1]=d::Max(high[1],x.y);high[2]=d::Max(high[2],x.z);
  }
  double maximum_secondary_gap=0;
  for(std::size_t i=0;i<a.secondary_count;++i) {
    const auto& row=a.secondary[i];maximum_secondary_gap=d::Max(maximum_secondary_gap,row.gap);
    if(row.coefficient==0)continue;
    maximum_edge=d::Max(maximum_edge,a.edge_length[i]);
    if(a.solid_tags[row.node])edge_sum=edge_sum+a.edge_length[i];
  }
  const double gap=a.global_search_gap;
  const double padding=a.initial_margin+d::Max(gap+0.,0.);
  for(unsigned k=0;k<3;++k){low[k]=low[k]-padding;high[k]=high[k]+padding;}
  for(std::size_t i=0;i<a.secondary_count;++i) {
    const auto x=a.positions[a.secondary[i].node];
    low[0]=d::Min(low[0],x.x);low[1]=d::Min(low[1],x.y);low[2]=d::Min(low[2],x.z);
    high[0]=d::Max(high[0],x.x);high[1]=d::Max(high[1],x.y);high[2]=d::Max(high[2],x.z);
  }
  double size[3];for(unsigned k=0;k<3;++k){size[k]=high[k]-low[k];if(!d::Nonnegative(size[k])||size[k]==0){Fail(a,k,Status::NonfiniteResult);return;}}
  const double area=(size[0]*size[1]+size[1]*size[2])+size[2]*size[0];
  double factor=::sqrt(double(nodes)/area);factor=.75*factor;
  if(!d::Nonnegative(factor)){Fail(a,0,Status::NonfiniteResult);return;}
  int dims[3];std::uint64_t cells=1;
  for(unsigned k=0;k<3;++k) {
    const double raw=factor*size[k];if(!d::Nonnegative(raw)||raw>2147483644.){Fail(a,k,Status::ResourceLimit);return;}
    dims[k]=max(1,int(::floor(raw+.5)));
    if(cells>std::uint64_t(INT64_MAX)/(std::uint64_t(dims[k])+2)){Fail(a,k,Status::ResourceLimit);return;}
    cells*=std::uint64_t(dims[k])+2;
  }
  if(cells>a.controls.native_voxel_capacity) {
    double ratio=double(a.controls.native_voxel_capacity);ratio=ratio/double(cells);ratio=::pow(ratio,1./3.);
    for(unsigned k=0;k<3;++k)dims[k]=max(1,int((dims[k]+2)*ratio)-2);
  }
  cells=(std::uint64_t(dims[0])+2)*(std::uint64_t(dims[1])+2)*(std::uint64_t(dims[2])+2);
  if(cells>a.controls.native_voxel_capacity)for(auto& dim:dims)dim=min(100,max(dim,1));
  for(unsigned k=0;k<3;++k){a.control->minimum[k]=low[k];a.control->maximum[k]=high[k];a.control->grid[k]=dims[k];}
  a.control->solid_nodes=solid_nodes;a.control->edge_average=solid_nodes?.5*edge_sum/double(solid_nodes):edge_sum;
  a.control->maximum_edge=maximum_edge;a.control->maximum_secondary_gap=maximum_secondary_gap;
  // The pinned BUC never populates LARGE_NODE after its zero initialization.
  a.control->large_nodes=0;
}
}
__global__ void ResetControls(Device a) {
  if(blockIdx.x||threadIdx.x)return;*a.control={};*a.sweep.control={};
  a.sweep.task_counts[a.mains_count]=0;
}
cudaError_t PrepareOperands(Device a,cudaStream_t stream) noexcept {
  ResetControls<<<1,1,0,stream>>>(a);auto reset=cudaPeekAtLastError();if(reset!=cudaSuccess)return reset;
  MainOperands<<<Blocks(a.mains_count),128,0,stream>>>(a);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  NodeOperands<<<Blocks(a.secondary_count),128,0,stream>>>(a);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  FoldOperands<<<1,1,0,stream>>>(a);return cudaPeekAtLastError();
}
} // namespace tlfea::contact::radioss_type25::initial_source::detail
