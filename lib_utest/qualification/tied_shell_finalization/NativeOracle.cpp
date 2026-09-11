#include "NativeOracle.h"
#include "native/Packet.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <stdexcept>
namespace tied_finalization_test {
namespace {
void Bits(double a, double b) {
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(a)), 0);
}
int Action(ts::NativeMessageAction a) {
  if (a == ts::NativeMessageAction::Accumulate) return 1;
  if (a == ts::NativeMessageAction::Flush) return 2;
  return 3;
}
}
NativeResult Native(const ts::FinalizationInput& in, int mode, int event_capacity) {
  if (in.node_count > 1048576 || in.master_count > 524288 || in.slave_count > 65536 ||
      in.main_node_count > in.node_count)
    throw std::runtime_error("Native finalization fixture exceeds packet bounds");
  if (event_capacity < 0) event_capacity = static_cast<int>(in.slave_count+8);
  NativeResult out;
  std::vector<int> masters(4*in.master_count), slaves(in.slave_count), mains(in.main_node_count);
  std::vector<int> selected(in.slave_count);
  std::vector<double> st(2*in.slave_count), distance(in.slave_count);
  for (std::size_t m = 0; m < in.master_count; ++m)
    for (unsigned j = 0; j < 4; ++j) masters[4*m+j] = static_cast<int>(in.masters[m][j])+1;
  for (std::size_t m = 0; m < in.main_node_count; ++m) mains[m] = static_cast<int>(in.main_nodes[m])+1;
  for (std::size_t s = 0; s < in.slave_count; ++s) {
    slaves[s] = static_cast<int>(in.slaves[s])+1;
    selected[s] = static_cast<int>(in.choices[s].ordered_master);
    st[2*s] = in.choices[s].projection.s;
    st[2*s+1] = in.choices[s].projection.t;
    distance[s] = in.choices[s].projection.selection_distance;
  }
  out.counts.fill(-17);
  out.nsv.assign(in.slave_count, -17);
  out.msr.assign(in.main_node_count, -17);
  out.selected.assign(in.slave_count, -17);
  out.irupt.assign(in.slave_count, -17);
  out.st.assign(2*in.slave_count, -17);
  out.stb = out.st;
  out.dpara.assign(7*in.slave_count, -17);
  out.nmas.assign(2*in.main_node_count, -17);
  out.events.resize(event_capacity);
  out.event_values.resize(event_capacity);
  for (auto& event : out.events) event.fill(-17);
  for (auto& value : out.event_values) value.fill(-17);
  native_tied_finalize(static_cast<int>(in.node_count), static_cast<int>(in.master_count),
      static_cast<int>(in.slave_count), static_cast<int>(in.main_node_count), event_capacity, mode,
      masters.data(), slaves.data(), mains.data(), selected.data(), st.data(), distance.data(),
      out.counts.data(), out.nsv.data(), out.msr.data(), out.selected.data(), out.st.data(),
      out.stb.data(), out.dpara.data(), out.irupt.data(), out.nmas.data(),
      reinterpret_cast<int*>(out.events.data()), reinterpret_cast<double*>(out.event_values.data()), &out.status);
  return out;
}
void Compare(const ts::FinalizationInput& in, const ts::FinalizationMaps& got, const NativeResult& ref) {
  ASSERT_EQ(ref.status, 0);
  ASSERT_EQ(ref.counts[3], 0);
  ASSERT_EQ(got.slaves.size(), std::size_t(ref.counts[0]));
  ASSERT_EQ(got.main_nodes.size(), std::size_t(ref.counts[1]));
  ASSERT_EQ(got.messages.size(), std::size_t(ref.counts[2]));
  for (std::size_t s = 0; s < got.slaves.size(); ++s) {
    SCOPED_TRACE(s);
    EXPECT_EQ(in.slaves[got.slaves[s]]+1, std::uint32_t(ref.nsv[s]));
    EXPECT_EQ(got.slave_inverse[got.slaves[s]], s);
    EXPECT_EQ(got.selected_masters[s], std::uint64_t(ref.selected[s]));
    Bits(got.st[s][0], ref.st[2*s]);
    Bits(got.st[s][1], ref.st[2*s+1]);
    Bits(ref.st[2*s], ref.stb[2*s]);
    Bits(ref.st[2*s+1], ref.stb[2*s+1]);
  }
  for (std::size_t m = 0; m < got.main_nodes.size(); ++m) {
    EXPECT_EQ(in.main_nodes[got.main_nodes[m]]+1, std::uint32_t(ref.msr[m]));
    EXPECT_EQ(got.main_inverse[got.main_nodes[m]], m);
  }
  for (std::size_t s = 0; s < in.slave_count; ++s) {
    const auto found = std::find(ref.nsv.begin(), ref.nsv.begin()+ref.counts[0], int(in.slaves[s]+1));
    const bool kept = found != ref.nsv.begin()+ref.counts[0];
    EXPECT_EQ(got.dispositions[s] == ts::FinalizationDisposition::Kept, kept);
    EXPECT_EQ(got.slave_inverse[s] == UINT32_MAX, !kept);
    if (!kept) {
      bool native_removed = false;
      for (int e = 0; e < ref.counts[2]; ++e)
        native_removed |= ref.events[e][0] == 1158 && ref.events[e][3] == int(in.slaves[s]+1);
      EXPECT_EQ(got.dispositions[s] == ts::FinalizationDisposition::OutsideParameters, native_removed);
    }
  }
  for (std::size_t e = 0; e < got.messages.size(); ++e) {
    SCOPED_TRACE(e);
    const auto& m = got.messages[e];
    const auto& n = ref.events[e];
    EXPECT_EQ(m.id, unsigned(n[0]));
    EXPECT_EQ(Action(m.action), n[1]);
    EXPECT_EQ(m.severity == ts::NativeMessageSeverity::Warning ? 1 : 2, n[2]);
    if (m.action != ts::NativeMessageAction::Accumulate) {
      EXPECT_EQ(n[3], 71); // Supplied interface source ID in the packet.
      continue;
    }
    ASSERT_LT(m.original_slave, in.slave_count);
    EXPECT_EQ(n[3], int(in.slaves[m.original_slave]+1));
    EXPECT_EQ(n[4], int(m.ordered_master));
    if (m.ordered_master) {
      for (unsigned j = 0; j < 4; ++j)
        EXPECT_EQ(n[5+j], int(in.masters[m.ordered_master-1][j]+1));
    }
    Bits(m.s, ref.event_values[e][0]);
    Bits(m.t, ref.event_values[e][1]);
    Bits(m.selection_distance, ref.event_values[e][2]);
  }
  EXPECT_EQ(got.cleared_dmin_entries, in.slave_count);
  for (const auto x : ref.dpara) Bits(x, 0);
  for (const auto x : ref.irupt) EXPECT_EQ(x, 0);
  for (const auto x : ref.nmas) Bits(x, 0);
}
}
