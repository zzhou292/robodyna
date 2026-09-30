#include "FinalizationInternal.h"
#include <cmath>
namespace tl::constraints::tied_shell::finalization_detail {
namespace {
void Add(std::size_t& total, std::size_t count, std::size_t width, std::size_t cap, bool& valid) {
  if (!valid || !width || total > cap || count > (cap-total)/width) { valid = false; return; }
  total += count*width;
}
template<class T> bool Range(const T* p, std::size_t n) {
  const auto a = reinterpret_cast<std::uintptr_t>(p);
  return p && a%alignof(T) == 0 && n <= SIZE_MAX/sizeof(T) && a <= UINTPTR_MAX-n*sizeof(T);
}
}
FinalizationReport Preflight(const FinalizationInput& in, const FinalizedSearch& old,
    FinalizationLimits l, std::size_t& bytes) noexcept {
  const FinalizationLimits hard;
  if (!l.max_nodes || l.max_nodes > hard.max_nodes || !l.max_masters || l.max_masters > hard.max_masters ||
      !l.max_slaves || l.max_slaves > hard.max_slaves || !l.max_host_bytes || l.max_host_bytes > hard.max_host_bytes)
    return Fail(FinalizationStatus::ResourceLimit, "Invalid finalization limits");
  if (!in.node_count || in.node_count > l.max_nodes || !in.master_count || in.master_count > l.max_masters ||
      !in.slave_count || in.slave_count > l.max_slaves || !in.main_node_count || in.main_node_count > in.node_count)
    return Fail(FinalizationStatus::ResourceLimit, "Finalization count exceeds capacity");
  if (in.profile != FinalizationProfile::SerialFirstType2Level28)
    return Fail(FinalizationStatus::InvalidInput, "Unsupported finalization profile");
  std::size_t next = sizeof(FinalizationMaps);
  bool valid = true;
  if (old.data()) Add(next, old.data()->owned_payload_bytes, 1, l.max_host_bytes, valid);
  Add(next, in.node_count, 2*sizeof(unsigned char), l.max_host_bytes, valid);
  Add(next, in.slave_count, sizeof(FinalizationDisposition)+2*sizeof(std::uint32_t)+
      sizeof(std::uint64_t)+sizeof(std::array<double,2>), l.max_host_bytes, valid);
  Add(next, in.main_node_count, 2*sizeof(std::uint32_t), l.max_host_bytes, valid);
  Add(next, in.slave_count+8, sizeof(FinalizationMessage), l.max_host_bytes, valid);
  if (!valid) return Fail(FinalizationStatus::ResourceLimit, "Finalization host budget exceeded");
  if (!Range(in.masters,in.master_count) || !Range(in.slaves,in.slave_count) ||
      !Range(in.main_nodes,in.main_node_count) || !Range(in.choices,in.slave_count))
    return Fail(FinalizationStatus::InvalidInput, "Invalid finalization borrowed extent");
  bytes = next;
  return {};
}
FinalizationReport Check(const FinalizationInput& in) {
  std::vector<unsigned char> flags(in.node_count,0);
  for (std::size_t i = 0; i < in.main_node_count; ++i) {
    const auto n = in.main_nodes[i];
    if (n >= in.node_count || flags[n])
      return Fail(FinalizationStatus::InvalidInput, "Invalid or duplicate original MSR node", i);
    flags[n] = 1;
  }
  for (std::size_t m = 0; m < in.master_count; ++m) {
    for (const auto n : in.masters[m]) {
      if (n >= in.node_count || !(flags[n]&1))
        return Fail(FinalizationStatus::InvalidInput, "IRECT node missing from original MSR", m);
    }
  }
  for (std::size_t s = 0; s < in.slave_count; ++s) {
    const auto n = in.slaves[s];
    if (n >= in.node_count || (flags[n]&2))
      return Fail(FinalizationStatus::InvalidInput, "Invalid or duplicate original NSV node", s);
    flags[n] |= 2;
    const auto& c = in.choices[s];
    if (c.matched != (c.ordered_master != 0) || c.ordered_master > in.master_count)
      return Fail(FinalizationStatus::InvalidInput, "Invalid finalization selected master", s);
    const auto& p = c.projection;
    if (!std::isfinite(p.s) || !std::isfinite(p.t) || !std::isfinite(p.selection_distance) || p.selection_distance < 0)
      return Fail(FinalizationStatus::InvalidInput, "Nonfinite or negative finalization search field", s);
  }
  return {};
}
}
