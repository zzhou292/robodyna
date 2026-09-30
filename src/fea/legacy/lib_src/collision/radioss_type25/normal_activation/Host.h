// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Admission.h"
namespace tlfea::contact::radioss_type25::normal_activation {
namespace detail {
struct HostWriter {
  Output out;
  void Main(std::uint32_t i) noexcept {out.main_active[i]=1;}
  void Node(std::uint32_t i) noexcept {out.node_tag[i]=1;}
};
}
// Allocation-free host value producer. Every input/range/cap check precedes the
// first write. It never reads or refreshes float normals, advances time or grants
// physical publication authority. CUDA orchestration reuses the per-work-item
// emitters with integer atomic OR/Exch after the same complete admission.
inline Status EvaluateNativeNormalActivation(const Input& in,Limits cap,Output out) noexcept {
  const auto admitted=detail::Validate(in,cap,out);if(admitted!=Status::Ok)return admitted;
  for(std::size_t i=0;i<out.main_count;++i)out.main_active[i]=0;
  for(std::size_t i=0;i<out.node_count;++i)out.node_tag[i]=0;
  detail::HostWriter writer{out};
  for(std::size_t i=0;i<in.row_count;++i)detail::RetainedRow(in,i,writer);
  for(std::size_t i=0;i<in.optimized_count;++i)detail::OptimizedMain(in,i,writer);
  for(std::size_t i=0;i<in.free_count;++i)detail::FreeMain(in,i,writer);
  return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::normal_activation
