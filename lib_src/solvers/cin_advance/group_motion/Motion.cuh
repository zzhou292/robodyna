// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Primary.h"
#include "Wrench.h"
#include "Members.h"
#include "Orientation.h"
namespace tl::fea::cin_advance::group_motion {
template<bool Capture>
__device__ inline void Advance(const Input& input,std::uint32_t group,Tile& tile) {
  const unsigned lane=threadIdx.x;
  if (!lane) tile.state=Begin<Capture>(input,group);
  __syncthreads();
  if (Failed(tile.state)) {
    if (!lane) input.group_reports[group]=tile.state.report;
    return; // Uniform after the leader's admission publication.
  }
  for (std::uint32_t first=0;first<tile.state.count;) {
    const auto remaining=tile.state.count-first;
    const auto count=remaining<Threads?remaining:Threads;
    if (lane<count) tile.wrench[lane]=PrepareWrench(input,tile.state,first+lane);
    __syncthreads();
    if (!lane) for (unsigned local=0;local<count&&!Failed(tile.state);++local)
      FoldWrench(tile.wrench[local],tile.state);
    __syncthreads();
    if (Failed(tile.state)) break;
    first+=count;
  }
  __syncthreads(); // Complete every stop-flag read before primary status changes.
  if (!lane&&!Failed(tile.state)) PreparePrimary(input,tile.state);
  __syncthreads();
  if (Failed(tile.state)) {
    if (!lane) input.group_reports[group]=tile.state.report;
    return;
  }
  for (std::uint32_t first=0;first<tile.state.count;) {
    const auto remaining=tile.state.count-first;
    const auto count=remaining<Threads?remaining:Threads;
    rigid::MemberStepTrial next;
    if (lane<count) tile.status[lane]=PrepareMember(input,tile.state,first+lane,next);
    __syncthreads();
    if (!lane) SelectPrefix(input,first,count,tile);
    __syncthreads();
    if (lane<tile.state.prefix) PublishMember<Capture>(input,tile.state,first+lane,next);
    __syncthreads(); // Prefix writes finish even when this tile rejects.
    if (Failed(tile.state)) break;
    first+=count;
  }
  __syncthreads();
  if (!lane&&!Failed(tile.state)) PublishGroup<Capture>(input,group,tile.state);
  __syncthreads();
  if (Failed(tile.state)) {
    if (!lane) input.group_reports[group]=tile.state.report;
    return;
  }
  // Group and all member/capture values are complete before quaternion work.
  for (std::uint32_t first=0;first<tile.state.count;) {
    const auto remaining=tile.state.count-first;
    const auto count=remaining<Threads?remaining:Threads;
    tl::math::Quaternion next;
    if (lane<count) tile.status[lane]=PrepareOrientation(input,tile.state,first+lane,next);
    __syncthreads();
    if (!lane) SelectPrefix(input,first,count,tile);
    __syncthreads();
    if (lane<tile.state.prefix) PublishOrientation(input,tile.state,first+lane,next);
    __syncthreads();
    if (Failed(tile.state)) break;
    first+=count;
  }
  __syncthreads();
  if (!lane) input.group_reports[group]=tile.state.report;
}
} // namespace tl::fea::cin_advance::group_motion
