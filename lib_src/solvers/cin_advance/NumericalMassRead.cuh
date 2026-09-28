// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceGatherValues.h"
namespace tl::fea::cin_advance::force_gather::mass_read {
inline constexpr unsigned Threads=64;
struct Tile {
  NumericalMassOperand rows[Threads];
  double mass;
  bool valid;
};
// All lanes participate. Only reads are parallel: lane zero retains the exact
// source-row add-then-subtract recurrence and first nonfinite prefix check.
__device__ inline bool Evaluate(cin::StageView source,cin::ForceTrial trial,
    const cin::detail::PreparedForceRow* prepared,Tile& tile,double& output) {
  if(!threadIdx.x){tile.mass=*trial.numerical_mass;tile.valid=true;}
  __syncthreads();
  for(std::uint32_t begin=0;begin<source.row_count;begin+=Threads) {
    const auto row=begin+threadIdx.x;
    tile.rows[threadIdx.x]=row<source.row_count?ReadNumericalMass(prepared[row]):NumericalMassOperand{};
    __syncthreads();
    if(!threadIdx.x) {
      const unsigned count=source.row_count-begin<Threads?source.row_count-begin:Threads;
      for(unsigned lane=0;lane<count;++lane) {
        if(!AppendNumericalMass(tile.rows[lane],tile.mass)){tile.valid=false;break;}
      }
    }
    __syncthreads();
    if(!tile.valid)break;
  }
  if(!threadIdx.x&&tile.valid)output=tile.mass;
  return tile.valid;
}
} // namespace tl::fea::cin_advance::force_gather::mass_read
