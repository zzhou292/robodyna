// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_force_transfers/Fixture.h"
#include "lib_src/solvers/cin_advance/ForceGather.h"
#include "lib_src/solvers/cin_advance/ForceGatherIncidence.h"
#include "lib_src/solvers/NodalCinGatherLayout.h"

namespace tl::fea::cin_gather_test {
namespace old = cin_transfer_test;
namespace packet = cin_parallel_test;
namespace cin = constraints::tied_shell::cin;
namespace gather = cin_advance::force_gather;
struct Workspace {
  std::vector<std::uint32_t> nodes, offsets, incidence, dense;
  std::vector<gather::Master> masters;
  std::vector<cin::detail::PreparedForceRow> rows;
  gather::Summary summary;
  gather::View view;
  bool Initialize(const cin_advance::Input& input) {
    const auto n = input.model.node_count, r = input.model.row_count;
    const auto cap = std::min<std::size_t>(n, 4ull*r);
    nodes.resize(cap); offsets.resize(cap+1); incidence.resize(4*r);
    dense.resize(n+1); masters.resize(cap); rows.resize(r);
    std::uint32_t count = 0;
    if (!gather::BuildIncidence(input.model,dense.data(),nodes.data(),offsets.data(),
        incidence.data(),cap,count)) return false;
    view = {input.model.rows,nodes.data(),offsets.data(),incidence.data(),masters.data(),
        &summary,n,r,count,std::uint32_t(cap)};
    return true;
  }
};
inline cin::StageReport Prepared(packet::Packet& p, Workspace& scratch,
    bool conservative_fallback = false, bool reverse = true) {
  auto input = p.Input();
  const auto force = cin_advance::force_inputs::ForceView(input);
  const auto checked = cin::detail::CheckForceInputs(input.model,force);
  if (!checked) return checked;
  for (std::uint32_t n=0;n<input.model.node_count;++n) force.entry_inertia[n]=force.inertia[n];
  if (!scratch.Initialize(input)) return {cin::StageStatus::InvalidInput};
  input.force_gather = scratch.view;
  input.prepared_transfers = scratch.rows.data();
  scratch.summary = {};
  for (std::uint32_t i=0;i<input.model.row_count;++i) {
    const auto row=reverse?input.model.row_count-1-i:i;
    scratch.rows[row].report=cin::detail::PrepareForceRow(input.model,force,row,scratch.rows[row]);
  }
  for (std::uint32_t i=0;i<scratch.view.master_count;++i) {
    const auto index=reverse?scratch.view.master_count-1-i:i;
    if (!gather::GatherMaster(input.model,force,input.prepared_transfers,scratch.view,index,scratch.masters[index]))
      scratch.summary.mode=gather::Mode::NeedsSerial;
  }
  if (conservative_fallback) scratch.summary.mode=gather::Mode::NeedsSerial;
  gather::Resolve(input);
  if (scratch.summary.mode==gather::Mode::Publish) {
    for (std::uint32_t i=0;i<scratch.view.master_count;++i) {
      const auto index=reverse?scratch.view.master_count-1-i:i;
      gather::PublishMaster(force,input.model.node_count,scratch.nodes[index],scratch.masters[index]);
    }
    for (std::uint32_t i=0;i<input.model.row_count;++i) {
      const auto row=reverse?input.model.row_count-1-i:i;
      gather::PublishSecondary(input.model,force,input.prepared_transfers,row);
    }
    *force.numerical_mass=scratch.summary.numerical_mass;
  }
  return scratch.summary.report;
}
} // namespace tl::fea::cin_gather_test
