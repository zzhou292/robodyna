#include "FinalizationInternal.h"
#include <cmath>
namespace tl::constraints::tied_shell::finalization_detail {
namespace {
void Message(FinalizationMaps& out, unsigned id, NativeMessageAction action,
    std::size_t row = SIZE_MAX, const SearchChoice* choice = nullptr) {
  FinalizationMessage m;
  m.id = id;
  m.action = action;
  m.severity = id == 1078 ? NativeMessageSeverity::Error : NativeMessageSeverity::Warning;
  m.original_slave = row;
  if (choice) {
    m.ordered_master = choice->ordered_master;
    m.s = choice->projection.s;
    m.t = choice->projection.t;
    m.selection_distance = choice->projection.selection_distance;
  }
  out.messages.push_back(m);
}
}
void Build(const FinalizationInput& in, FinalizationMaps& out) {
  out.dispositions.resize(in.slave_count, FinalizationDisposition::Kept);
  out.slave_inverse.resize(in.slave_count, UINT32_MAX);
  out.main_inverse.resize(in.main_node_count, UINT32_MAX);
  out.slaves.reserve(in.slave_count);
  out.main_nodes.reserve(in.main_node_count);
  out.selected_masters.reserve(in.slave_count);
  out.st.reserve(in.slave_count);
  out.messages.reserve(in.slave_count+8);
  std::size_t unmatched = 0;
  for (std::size_t s = 0; s < in.slave_count; ++s) {
    const auto& c = in.choices[s];
    const auto& p = c.projection;
    if (!c.matched) {
      out.dispositions[s] = FinalizationDisposition::Unmatched;
      ++unmatched;
      Message(out,1071,NativeMessageAction::Accumulate,s);
    } else if (p.s > 1.5 || p.t > 1.5 || p.s < -1.5 || p.t < -1.5) {
      out.dispositions[s] = FinalizationDisposition::OutsideParameters;
      Message(out,1158,NativeMessageAction::Accumulate,s,&c);
    } else if (p.s > 1.02 || p.t > 1.02 || p.s < -1.02 || p.t < -1.02) {
      Message(out,1079,NativeMessageAction::Accumulate,s,&c);
    }
  }
  // Native DMIN aliases the first NSN entries of fresh seven-wide DPARA.
  // Its old distance remains a diagnostic on the retained input assessment.
  out.cleared_dmin_entries = in.slave_count;
  Message(out,1071,NativeMessageAction::Flush);
  if (unmatched == in.slave_count) Message(out,1217,NativeMessageAction::Direct);
  for (const auto id : {1078u,1079u,1873u,1157u,1158u,1872u})
    Message(out,id,NativeMessageAction::Flush);
  std::vector<unsigned char> used(in.node_count,0);
  for (std::size_t s = 0; s < in.slave_count; ++s) {
    if (out.dispositions[s] != FinalizationDisposition::Kept) continue;
    const auto& c = in.choices[s];
    out.slave_inverse[s] = static_cast<std::uint32_t>(out.slaves.size());
    out.slaves.push_back(static_cast<std::uint32_t>(s));
    out.selected_masters.push_back(c.ordered_master);
    out.st.push_back({c.projection.s,c.projection.t});
    for (const auto n : in.masters[c.ordered_master-1]) used[n] = 1;
  }
  for (std::size_t m = 0; m < in.main_node_count; ++m) {
    if (!used[in.main_nodes[m]]) continue;
    out.main_inverse[m] = static_cast<std::uint32_t>(out.main_nodes.size());
    out.main_nodes.push_back(static_cast<std::uint32_t>(m));
  }
}
std::size_t Owned(const FinalizationMaps& d) noexcept {
  return sizeof(d) + d.dispositions.capacity()*sizeof(FinalizationDisposition) +
      (d.slaves.capacity()+d.main_nodes.capacity()+d.slave_inverse.capacity()+d.main_inverse.capacity())*sizeof(std::uint32_t) +
      d.selected_masters.capacity()*sizeof(std::uint64_t) + d.st.capacity()*sizeof(std::array<double,2>) +
      d.messages.capacity()*sizeof(FinalizationMessage);
}
}
