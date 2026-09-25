// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#include <cmath>
#include <stdexcept>
#include <vector>
namespace type25_startup_test {
namespace n=tlfea::contact::radioss_type25;
namespace s=n::startup;
struct Case {
  std::vector<std::uint64_t> ids;
  std::vector<double> positions,coefficients;
  std::vector<s::PrimaryFace> primary;
  s::Coordinates units=s::Coordinates::Native;
  n::UnitScale scale{.001,1000,1};
  s::Profile profile=s::Profile::OrdinaryExteriorFixedMain;
  s::TopologyPolicy topology=s::TopologyPolicy::ManifoldTwoSided;
  s::Input Input() const {
    s::Input result;result.profile=profile;result.topology=topology;
    result.node_source_ids=ids.data();result.node_count=ids.size();
    result.positions={positions.data(),std::uint32_t(ids.size()),3,1};
    result.primary=primary.data();result.primary_count=primary.size();result.coordinates=units;
    result.units=scale;result.source_generation=7;return result;
  }
  void Add(n::ShellLayout type,std::uint32_t a,std::uint32_t b,std::uint32_t c,std::uint32_t d) {
    primary.push_back({100+primary.size(),type,{a,b,c,d}});
    coefficients.assign(2*primary.size(),210000.);
  }
};
inline Case Grid(unsigned nx,unsigned ny,unsigned mode=0) {
  Case result;
  for(unsigned y=0;y<=ny;++y)for(unsigned x=0;x<=nx;++x) {
    result.ids.push_back(result.ids.size()+1);result.positions.insert(result.positions.end(),{double(x),double(y),0.});
  }
  for(unsigned y=0;y<ny;++y)for(unsigned x=0;x<nx;++x) {
    const auto a=y*(nx+1)+x,b=a+1,d=a+nx+1,c=d+1;
    if(mode==1 || (mode==2&&(x+y)%2==0)) {
      result.Add(n::ShellLayout::Triangle3,a,b,c,c);result.Add(n::ShellLayout::Triangle3,a,c,d,d);
    } else result.Add(n::ShellLayout::Quad4,a,b,c,d);
  }
  return result;
}
struct Built {
  tl::util::HostArena output,scratch,ready_output,ready_scratch;
  s::Snapshot startup;
  s::FixedMainView ready;
  s::Forecast forecast;
  explicit Built(const Case& source) {
    forecast=s::Preflight(source.ids.size(),source.primary.size());
    if(forecast.status!=s::Status::Ok || !output.Initialize(forecast.output_bytes) ||
        !scratch.Initialize(forecast.scratch_bytes) || !ready_output.Initialize(forecast.ready_output_bytes) ||
        !ready_scratch.Initialize(forecast.ready_scratch_bytes))throw std::runtime_error("Bounded startup fixture allocation failed");
    const auto input=source.Input();
    if(s::BuildStarter(input,{},output,scratch,&startup).status!=s::Status::Ok)
      throw std::runtime_error("Startup fixture was unexpectedly rejected");
    if(s::BuildFixedMain(input,startup,{source.coefficients.data(),source.coefficients.size()},
        {},ready_output,ready_scratch,&ready).status!=s::Status::Ok)
      throw std::runtime_error("Fixed-main-ready fixture was unexpectedly rejected");
  }
};
} // namespace type25_startup_test
