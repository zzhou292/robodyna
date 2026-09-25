// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25NormalActivation.h"
#include "../radioss_type25_lifecycle/Fixture.h"
#include <limits>
namespace normal_activation_test {
namespace n=tlfea::contact::radioss_type25;
namespace a=n::normal_activation;
namespace l=n::selection::lifecycle;
struct Fixture {
  type25_lifecycle_test::Fixture scene;
  std::vector<l::OptimizedRow> rows;
  std::vector<std::uint32_t> optimized,free;
  void Rebuild() {
    rows.clear();optimized.clear();free.clear();
    const auto in=scene.Input();n::units_detail::Factors units;
    if(!l::detail::Factors(in.current,units))throw std::runtime_error("Fixture unit declaration failed");
    for(std::size_t i=0;i<scene.secondary.size();++i) {
      const auto row=l::detail::PrepareRowBeforeNormals(in,i,units);
      if(row.stage.report.status!=n::selection::Status::Ok)throw std::runtime_error("Fixture Begin/OPTCD rejected");
      rows.push_back(row);
      for(auto j=scene.spatial_offsets[i];j<scene.spatial_offsets[i+1];++j) {
        const auto main=scene.spatial[scene.spatial_entries[j]].local_main;
        if(l::detail::OptimizedCandidate(in,i,main,row.optimization_main,row.optimization_leave,units))
          optimized.push_back(std::uint32_t(main));
      }
    }
    // Declared fixture input only; independent I25FREE_BOUND checks this whole
    // list. Production authority comes from the actual retained source stage.
    for(std::size_t i=0;i<scene.mains.size();++i) {
      const auto& main=scene.mains[i];if(main.coefficient<=0)continue;
      for(unsigned j=0;j<4;++j)if(main.neighbors[j]==0&&!(j==2&&main.nodes[2]==main.nodes[3])) {
        free.push_back(std::uint32_t(i+1));break;
      }
    }
  }
  a::Input Input() const {
    return {{0,0,1,2,1,a::FreeRosterPolicy::FreshComplete},scene.Input().source,
      rows.data(),rows.size(),optimized.data(),optimized.size(),free.data(),free.size()};
  }
  static a::Limits Limits(){return {64,16,8,64,128,64,128};}
};
inline Fixture Scenario(unsigned scenario) {
  Fixture f;auto& s=f.scene;
  if(scenario>=1&&scenario<=5)s.Retained();
  if(scenario==2){s.removed_entries={1,2,3};s.removed_offsets={0,3};}
  if(scenario==3)s.mains[2].coefficient=-400;
  if(scenario==4)s.mains[0].coefficient=0;
  if(scenario==5)s.secondary[0].coefficient=0;
  if(scenario==6){s.spatial.push_back({1,1});s.Rebuild();}
  if(scenario==7||scenario==8||scenario==9) {
    s.spatial.clear();s.Rebuild();
    s.mains[0].neighbors[0]=0;s.mains[1].neighbors[0]=0;
    if(scenario==8){s.mains[0].coefficient=0;s.mains[1].coefficient=-1;}
    if(scenario==9){s.TrianglePair();s.mains[0].neighbors[0]=1;s.mains[1].neighbors[0]=1;
      s.mains[0].neighbors[2]=0;s.mains[1].neighbors[2]=0;}
  }
  if(scenario==10){s.secondary.clear();s.accepted.clear();s.spatial.clear();s.Rebuild();s.mains[0].neighbors[0]=0;}
  if(scenario==11){s.mains[0].segment_type=int(s.mains.size()+2);s.spatial.push_back({1,1});s.Rebuild();}
  f.Rebuild();return f;
}
} // namespace normal_activation_test
