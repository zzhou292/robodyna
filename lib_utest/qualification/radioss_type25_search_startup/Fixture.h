// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <algorithm>
#include <utility>
#include "../radioss_type25_fixed_main_startup/Fixture.h"
namespace type25_search_startup_test {
namespace n=tlfea::contact::radioss_type25;
namespace st=n::startup;
namespace old=type25_startup_test;
struct Fixture {
  old::Case mesh;
  old::Built topology;
  std::vector<s::Secondary> secondary;
  std::vector<double> main_gaps;
  s::Profile profile;
  std::size_t physical_shells=0;
  explicit Fixture(old::Case value=old::Grid(2,2)):mesh(std::move(value)),topology(mesh) {
    physical_shells=mesh.primary.size();
    for(std::size_t i=0;i<mesh.ids.size();++i)secondary.push_back({std::uint32_t(i),1.,.01});
    main_gaps.assign(topology.startup.main_count,.01);
    profile.level=1;profile.gap_mode=1;profile.neighbor_removal=2;profile.initial_penetration=5;
    profile.edge_mode=0;profile.thermal_mode=0;profile.curvature=0;profile.partitions=1;
    profile.gap_load_cards=s::LoadCards::Absent;profile.initialization=s::Initialization::SerialNative;
  }
  s::Input Input() const {
    s::Input value;value.mesh=mesh.Input();value.topology=topology.startup;value.profile=profile;
    value.contributors.census=s::Census::CompleteDeclaredModel;
    value.contributors.physical_nodes=mesh.ids.size();value.contributors.physical_shells=physical_shells;
    value.secondary=secondary.data();value.secondary_count=secondary.size();
    value.main_gaps=main_gaps.data();value.main_count=main_gaps.size();return value;
  }
  void Gaps(double gap) {for(auto& row:secondary)row.gap=gap;std::fill(main_gaps.begin(),main_gaps.end(),gap);}
};
struct Built {
  tl::util::HostArena output,scratch;
  s::Snapshot view;
  s::Forecast plan;
  explicit Built(const Fixture& fixture) {
    const auto in=fixture.Input();plan=s::Preflight(in.mesh.node_count,in.mesh.primary_count,in.secondary_count);
    if(plan.status!=s::Status::Ok || !output.Initialize(plan.output_bytes) || !scratch.Initialize(plan.scratch_bytes))
      throw std::runtime_error("Search startup fixture allocation failed");
    if(s::Build(in,{},output,scratch,&view).status!=s::Status::Ok)
      throw std::runtime_error("Search startup fixture unexpectedly rejected");
  }
};
} // namespace type25_search_startup_test
