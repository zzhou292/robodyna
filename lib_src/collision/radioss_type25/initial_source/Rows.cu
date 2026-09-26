// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::initial_source::detail {
namespace {
unsigned Blocks(std::size_t n){return unsigned(std::max<std::size_t>(1,std::min<std::size_t>(256,(n+127)/128)));}
__global__ void Reduce(Device a,std::size_t pairs) {
  if(a.control->failure!=~0ull||a.sweep.control->failure!=~0ull)return;
  auto rows=a.rows;rows.pair_count=pairs;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<a.secondary_count;i+=gridDim.x*blockDim.x)
    a.row_status[i]=initial_state::ReduceRow(rows,i,a.winners+i);
}
__global__ void Fold(Device a) {
  if(blockIdx.x||threadIdx.x||a.control->failure!=~0ull||a.sweep.control->failure!=~0ull)return;
  for(std::size_t row=0;row<a.secondary_count;++row)if(a.row_status[row].status!=initial_state::Status::Ok) {
    a.control->failure=(static_cast<unsigned long long>(row)<<8)|unsigned(Status::NonfiniteResult);return;
  }
}
__global__ void Normalize(Device a) {
  if(a.control->failure!=~0ull||a.sweep.control->failure!=~0ull)return;
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<a.secondary_count;row+=gridDim.x*blockDim.x) {
    auto& winner=a.winners[row];initial_state::FinalizeInacti5(winner);
    if(winner.row.irtlm[0]>0)atomicAdd(&a.control->warm_before,1ull);
    if(a.added_removals)for(auto j=a.final_offsets[row];j<a.final_offsets[row+1];++j) {
      const auto main=a.final_mains[j]-1;
      if(winner.row.irtlm[0]==a.mains[main].global_id) {
        winner.row={};atomicAdd(&a.control->reset,1ull);break;
      }
    }
    if(winner.row.irtlm[0]>0) {
      const auto main=std::size_t(winner.row.irtlm[2]-1);
      if(main>=a.mains_count||initial_state::MapPreparedMain(int(main+1),1,winner)!=initial_state::Status::Ok) {
        atomicMin(&a.control->failure,(static_cast<unsigned long long>(row)<<8)|unsigned(Status::InvalidInput));continue;
      }
      atomicAdd(&a.control->warm_after,1ull);
      const double coefficient=a.mains[main].coefficient;
      if(coefficient>0)atomicAdd(&a.control->warm_positive,1ull);
      else if(coefficient<0)atomicAdd(&a.control->warm_negative,1ull);
      else atomicAdd(&a.control->warm_zero,1ull);
    }
  }
}
__global__ void Publish(Device a) {
  if(a.control->failure!=~0ull||a.sweep.control->failure!=~0ull)return;
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<a.secondary_count;row+=gridDim.x*blockDim.x) {
    a.seed_history[row]={a.nodes[a.secondary[row].node].source_id,a.stamp.topology,a.winners[row].row};
    a.seed_flags[row]=0; // Actual fresh INACTI5 allocation/PWR contract.
  }
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<4*a.mains_count;i+=gridDim.x*blockDim.x)
    a.seed_gaps[i]=a.mains[i/4].gap[i%4];
}
}
cudaError_t ProduceRows(Device a,std::size_t pairs,cudaStream_t stream) noexcept {
  Reduce<<<Blocks(a.secondary_count),128,0,stream>>>(a,pairs);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  Fold<<<1,1,0,stream>>>(a);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  Normalize<<<Blocks(a.secondary_count),128,0,stream>>>(a);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  Publish<<<Blocks(std::max(a.secondary_count,4*a.mains_count)),128,0,stream>>>(a);return cudaPeekAtLastError();
}
}
