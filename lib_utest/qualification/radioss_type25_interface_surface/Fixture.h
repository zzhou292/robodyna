#pragma once
#include "NativeOracle.h"
#include "lib_src/collision/RadiossType25InterfaceSurface.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#include "../radioss_type25_surface_source/Fixture.h"
#include <algorithm>
#include <cstring>
namespace type25_interface_surface_test {
namespace old=::type25_surface_source_test;
struct Case {
  old::Case physical;
  std::vector<n::Vector> points;
  std::vector<std::uint64_t> ids;
  std::vector<n::source_surfaces::Face> raw;
  s::Coordinates coordinates=s::Coordinates::Native;
  n::UnitScale units{1,1,1};
  Case() : points(24),ids(24) {
    for(std::size_t i=0;i<points.size();++i) {
      points[i]={double(i+3),double(i%3),double(i%5)};
      ids[i]=1000+i;
    }
    const n::Vector cube[]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},
        {0,0,1},{1,0,1},{1,1,1},{0,1,1}};
    std::copy_n(cube,8,points.begin());
  }
  void Extract() {
    const old::Built built(physical);
    if(built.report.status!=old::s::Status::Ok)throw std::runtime_error("Initial extraction fixture rejected");
    raw.clear();
    if(built.result.face_count)raw.assign(built.result.faces,built.result.faces+built.result.face_count);
  }
  f::Input Input() const {
    static_assert(sizeof(n::Vector)==3*sizeof(double));
    f::Input in;
    in.profile=f::Profile::SingleSurfaceIlev1;in.physical=physical.Input();
    in.raw_faces=raw.empty()?nullptr:raw.data();in.raw_face_count=raw.size();
    in.positions={reinterpret_cast<const double*>(points.data()),std::uint32_t(points.size()),3,1};
    in.coordinates=coordinates;in.units=units;in.source_generation=71;
    return in;
  }
};
inline Case SolidOnly() {
  Case c;c.physical.solids={old::Hex()};c.Extract();return c;
}
inline Case Mixed(bool reversed=false) {
  auto c=SolidOnly();
  c.physical.quads={{201,10,{4,5,6,7}},{202,10,{14,15,16,17}}};
  if(reversed)std::reverse(std::begin(c.physical.quads[0].nodes),std::end(c.physical.quads[0].nodes));
  c.physical.triangles={{203,10,{18,19,20,20}}};
  c.Extract();return c;
}
inline Case Origins() {
  auto c=SolidOnly();
  c.physical.solids.push_back({102,10,old::s::SolidTopology::NativeRaw8,{6,7,3,2,8,8,9,9}});
  c.Extract();return c;
}
inline Case FirstSolid() {
  Case c;
  c.points[0]={0,0,0};c.points[1]={1,0,0};c.points[2]={0,1,0};
  c.points[3]={0,0,1};c.points[4]={1,0,1};c.points[5]={0,1,1};
  c.points[6]={0,0,-1};c.points[7]={1,0,-1};c.points[8]={0,1,-1};
  c.physical.solids={old::Penta(101,10),{102,20,old::s::SolidTopology::DeclaredPenta6,{0,1,2,0,6,7,8,6}}};
  c.physical.triangles={{201,30,{0,1,2,2}}};c.physical.parts={30};
  c.Extract();return c;
}
struct Built {
  tl::util::HostArena output,scratch;
  f::Forecast forecast;
  f::Snapshot result;
  f::Report report;
  explicit Built(const Case& c) {
    const auto in=c.Input();report=f::Preflight(in,{},forecast);
    if(report.status!=f::Status::Ok||!output.Initialize(forecast.output_bytes)||
        !scratch.Initialize(forecast.scratch_bytes))throw std::runtime_error("Interface fixture preflight");
    report=f::Build(in,{},output,scratch,&result);
  }
};
inline s::Input SideInput(const Case& c,const f::Snapshot& value) {
  const auto current=c.Input();
  s::Input in;in.profile=s::Profile::MixedSurface;in.topology=s::TopologyPolicy::NativeMixedSurface;
  in.node_source_ids=c.ids.data();in.node_count=c.ids.size();in.positions=current.positions;
  in.primary=value.primary;in.primary_count=value.primary_count;in.coordinates=c.coordinates;in.units=c.units;
  in.source_generation=current.source_generation;in.primary_identities=value.identities;
  in.primary_identity_count=value.primary_count;in.shell_primary_count=value.shell_primary_count;
  in.raw_origins=value.raw_origins;in.raw_origin_to_primary=value.raw_to_primary;in.raw_origin_count=value.raw_face_count;
  return in;
}
struct Sides {
  tl::util::HostArena output,scratch;
  s::Forecast forecast;
  s::MixedSidesSnapshot result;
  s::Report report;
  explicit Sides(const s::Input& in) {
    forecast=s::PreflightMixedSides(in);
    if(forecast.status!=s::Status::Ok||!output.Initialize(forecast.output_bytes)||
        !scratch.Initialize(forecast.scratch_bytes))throw std::runtime_error("Side fixture preflight");
    report=s::BuildMixedSides(in,{},output,scratch,&result);
  }
};
inline std::vector<unsigned char> Bytes(const tl::util::HostArena& arena) {
  const auto* first=static_cast<const unsigned char*>(arena.data());return {first,first+arena.bytes()};
}
}
