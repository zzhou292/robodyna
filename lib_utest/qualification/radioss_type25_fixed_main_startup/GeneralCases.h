// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include <array>
namespace type25_startup_test {
inline s::Input GeneralInput(const Case& source) {
  auto in=source.Input();in.profile=s::Profile::OrdinaryExteriorMovingMain;
  in.topology=s::TopologyPolicy::NativeOrdinaryShell;return in;
}
// A bounded source-edge star, with distinct node identities on every ray.
// Modes are all Q4, all T3 or alternating; no expected normal is manufactured.
inline Case EdgeStar(unsigned valence,unsigned mode=0,bool alternating_winding=false) {
  if(valence<2||valence>4)throw std::invalid_argument("Bounded star valence");
  Case c;c.ids={1,2};c.positions={-1,0,0,1,0,0};
  constexpr double ray[4][2]{{1,0},{0,1},{-1,0},{0,-1}};
  for(unsigned i=0;i<valence;++i) {
    const bool tri=mode==1||(mode==2&&i%2);const auto first=std::uint32_t(c.ids.size());
    if(tri){c.ids.push_back(c.ids.size()+1);c.positions.insert(c.positions.end(),{0,ray[i][0],ray[i][1]});}
    else for(double x:{1.,-1.}){c.ids.push_back(c.ids.size()+1);c.positions.insert(c.positions.end(),{x,ray[i][0],ray[i][1]});}
    const auto a=alternating_winding&&i%2?1u:0u,b=1u-a;
    if(tri)c.Add(n::ShellLayout::Triangle3,a,b,first,first);
    else if(a)c.Add(n::ShellLayout::Quad4,a,b,first+1,first);
    else c.Add(n::ShellLayout::Quad4,a,b,first,first+1);
  }
  return c;
}
inline Case DisconnectedFan() {
  Case c;c.ids={1,2,3,4,5};c.positions={0,0,0,1,0,0,0,1,0,-1,0,0,0,-1,0};
  c.Add(n::ShellLayout::Triangle3,0,1,2,2);c.Add(n::ShellLayout::Triangle3,0,3,4,4);return c;
}
inline Case SharedExtraVertex(bool triangle) {
  Case c;c.ids={1,2,3,4,5};c.positions={0,0,0,1,0,0,1,1,0,0,1,0,0,1,1};
  c.Add(n::ShellLayout::Quad4,0,1,2,3);
  if(triangle)c.Add(n::ShellLayout::Triangle3,1,0,2,2);
  else c.Add(n::ShellLayout::Quad4,2,1,0,4);
  return c;
}
struct GeneralBuilt {
  tl::util::HostArena output,scratch;
  s::Snapshot startup;
  s::Forecast forecast;
  s::Report report;
  explicit GeneralBuilt(const Case& source) {
    const auto in=GeneralInput(source);forecast=s::Preflight(in);
    if(forecast.status!=s::Status::Ok||!output.Initialize(forecast.output_bytes)||!scratch.Initialize(forecast.scratch_bytes))
      throw std::runtime_error("General startup fixture allocation failed");
    report=s::BuildStarter(in,{},output,scratch,&startup);
    if(report.status!=s::Status::Ok)throw std::runtime_error("General startup fixture rejected");
  }
};
} // namespace type25_startup_test
