// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_interface_surface/Fixture.h"
#include "../radioss_type25_current_normals/Assertions.h"
#include "../radioss_type25_fixed_main_startup/Assertions.h"
#include "lib_src/collision/RadiossType25CurrentNormals.h"
#include <memory>
#include <functional>
#include <stdexcept>
namespace type25_post_gapm_test {
namespace upstream=type25_interface_surface_test;
namespace current=type25_current_normals_test;
namespace c=n::current_normals;
using Case=upstream::Case;
inline Case Blocks(unsigned mode,bool shells=false) {
  Case value;
  value.physical.mode=n::source_surfaces::SurfaceMode::All;
  for(unsigned z=0;z<2;++z)for(unsigned y=0;y<3;++y)for(unsigned x=0;x<3;++x)
    value.points[x+3*y+9*z]={double(x),double(y),double(z)};
  const auto add=[&](unsigned x,unsigned y) {
    const unsigned a=x+3*y;
    value.physical.solids.push_back({101+value.physical.solids.size(),10,n::source_surfaces::SolidTopology::Hex8,
        {a,a+1,a+4,a+3,a+9,a+10,a+13,a+12}});
  };
  add(0,0);
  if(mode==1)add(1,0); // Shared internal face.
  if(mode==2 || mode==4)add(1,1); // Edge contact: exterior DNE can exceed one.
  if(mode==3){add(1,0);add(0,1);add(1,1);} // Internal faces and a shared central edge.
  if(mode==4) {
    // A separate genuine internal pair keeps native IDEL enabled while the
    // first diagonal pair exercises multiple exterior candidates at one edge.
    value.points.resize(34);value.ids.resize(34);value.physical.nodes=34;
    for(unsigned i=24;i<34;++i)value.ids[i]=1000+i;
    const n::Vector cube[]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
    for(unsigned i=0;i<8;++i)value.points[18+i]={cube[i].x+5,cube[i].y,cube[i].z};
    value.points[26]={5,0,2};value.points[27]={6,0,2};value.points[28]={6,1,2};value.points[29]={5,1,2};
    value.physical.solids.push_back({103,10,n::source_surfaces::SolidTopology::Hex8,{18,19,20,21,22,23,24,25}});
    value.physical.solids.push_back({104,10,n::source_surfaces::SolidTopology::Hex8,{22,23,24,25,26,27,28,29}});
  }
  if(shells) {
    value.points[18]={3,0,0};value.points[19]={4,0,0};value.points[20]={4,1,0};
    value.points[21]={3,1,0};value.points[22]={4,1,1};value.points[23]={3,2,0};
    value.physical.quads.push_back({301,10,{18,19,20,21}});
    value.physical.triangles.push_back({302,10,{21,22,23,23}});
    // Genuine physical coating on the first cube's bottom face.
    value.physical.quads.push_back({303,10,{0,3,4,1}});
  }
  value.Extract();return value;
}
inline Case Penta() {
  auto value=upstream::FirstSolid();
  value.physical.solids.resize(1);value.physical.parts={10,30};
  value.physical.mode=n::source_surfaces::SurfaceMode::All;
  value.Extract();return value;
}
struct Fixture {
  Case mesh;
  upstream::Built classified;
  s::Input input;
  upstream::Sides sides;
  std::vector<s::PrimaryCornerPermutation> corners;
  std::vector<s::PreShellSolidSupport> before;
  std::vector<s::PostGapmMainSupport> supports;
  std::vector<double> coefficients;
  s::PostGapmTopology post;
  tl::util::HostArena output,scratch;
  s::Forecast forecast;
  s::Snapshot startup;
  s::Report report;
  NativeResult native;
  std::vector<double> positions;
  std::vector<std::uint32_t> active,tags,free_ids;
  std::vector<n::StoredNormal> prior,native_prior;
  explicit Fixture(Case source,bool reverse_primary=false,bool erosion=true):mesh(std::move(source)),
      classified(mesh),input(upstream::SideInput(mesh,classified.result)),sides(input) {
    if(classified.report.status!=n::surface_interface::Status::Ok || sides.report.status!=s::Status::Ok)
      throw std::runtime_error("Mixed upstream fixture rejected");
    const auto p=input.primary_count,g=sides.result.main_count;
    corners.resize(p);before.resize(p);supports.resize(g);coefficients.assign(g,100.);
    for(std::size_t i=0;i<p;++i) {
      const auto& face=sides.result.mains[i];
      if(face.nodes[2]==face.nodes[3])corners[i].source_corner[3]=2;
      std::vector<std::uint64_t> matches;
      for(const auto& solid:mesh.physical.solids) {
        bool contains=true;
        for(auto node:face.nodes)contains=contains && std::find(std::begin(solid.nodes),std::end(solid.nodes),node)!=std::end(solid.nodes);
        if(contains)matches.push_back(solid.element_id);
      }
      std::sort(matches.begin(),matches.end(),std::greater<std::uint64_t>());
      if(matches.size()>2)throw std::runtime_error("Fixture requires an explicit native solid order");
      before[i].unique_match_count=std::uint32_t(matches.size());
      if(!matches.empty())before[i].first_solid_source_id=matches[0];
      if(matches.size()==2)before[i].second_solid_source_id=matches[1];
      const auto& identity=input.primary_identities[i];
      if(identity.kind==s::PrimaryFaceKind::Shell) {
        supports[i].first={face.nodes[2]==face.nodes[3]?s::PhysicalSupportKind::ShellTriangle:s::PhysicalSupportKind::ShellQuad,identity.physical_parent_id};
      } else {
        if(matches.empty())throw std::runtime_error("Source-generated solid face lost its support");
        supports[i]={{s::PhysicalSupportKind::EightSlotSolid,matches[0]},matches.size()==2?matches[1]:0};
        if(matches.size()==2)coefficients[i]=-100.;
      }
      // Prescribed upstream I25GAPM permutation coupon. The whole native
      // topology oracle receives the same explicit source-phase operand.
      if(reverse_primary && matches.size()==1) {
        const bool triangle=face.nodes[2]==face.nodes[3];
        const std::uint8_t q[]{3,2,1,0},t[]{1,0,2,2};
        std::copy_n(triangle?t:q,4,corners[i].source_corner);
      }
      const auto partner=sides.result.primary_to_partner[i];
      if(partner)supports[partner-1]=supports[i];
    }
    post.phase=s::PostGapmPhase::FinalizedBeforeNeighbors;
    post.primary_corners=corners.data();post.primary_count=p;
    post.before_shell=before.data();post.before_shell_count=p;
    post.final_support=supports.data();post.main_count=g;
    post.pre_shell_internal_count=std::count_if(before.begin(),before.end(),[](const auto& a){return a.second_solid_source_id!=0;});
    post.incoming_solid_erosion=erosion?s::SolidErosion::Enabled:s::SolidErosion::Disabled;
    post.final_solid_erosion=post.pre_shell_internal_count?post.incoming_solid_erosion:s::SolidErosion::Disabled;
    post.source_generation=input.source_generation;
    forecast=s::PreflightMixedStarter(input,sides.result,post);
    if(forecast.status!=s::Status::Ok || !output.Initialize(forecast.output_bytes) || !scratch.Initialize(forecast.scratch_bytes))
      throw std::runtime_error("Mixed post-GAPM fixture forecast rejected");
    report=s::BuildStarter(input,sides.result,post,{},output,scratch,&startup);
    native=Oracle(input,post,coefficients.data(),coefficients.size());
    if(report.status!=s::Status::Ok)return;
    positions.resize(3*mesh.points.size());
    for(std::size_t i=0;i<mesh.points.size();++i) {
      positions[3*i]=mesh.points[i].x;positions[3*i+1]=mesh.points[i].y;positions[3*i+2]=mesh.points[i].z;
    }
    active.assign(g,1);tags.assign(mesh.points.size(),1);
    prior.assign(startup.starter.face_normals,startup.starter.face_normals+4*g);native_prior=native.starter_normals;
    RefreshFree();
  }
  void RefreshFree() {
    free_ids.clear();
    for(std::size_t i=0;i<startup.main_count;++i)if(coefficients[i]>0)
      for(unsigned k=0;k<4;++k)if(!startup.mains[i].neighbors[k] && !(k==2 && startup.mains[i].nodes[2]==startup.mains[i].nodes[3])) {
        free_ids.push_back(std::uint32_t(i+1));break;
      }
  }
  c::Input Current(bool native_topology=false) const {
    c::Input in;in.profile=c::Profile::MixedSurfaceLocal;
    in.free_roster=n::normal_activation::FreeRosterPolicy::FreshComplete;
    in.topology={startup.mains,startup.node_count,startup.primary_count,startup.main_count,startup.starter.reference_count,
        {startup.normal_offsets,startup.starter.reference_count+1,startup.normal_mains,startup.normal_incidence_count}};
    in.topology.source_profile=s::Profile::MixedSurface;in.topology.source_topology=s::TopologyPolicy::NativeMixedSurface;
    in.topology.primary_roles=startup.primary_roles;in.topology.primary_role_count=startup.primary_count;
    in.topology.mixed_maps={startup.primary_to_partner,startup.primary_count};
    if(native_topology) {
      in.topology.mains=native.mains.data();in.topology.references=native.starter_references.size();
      in.topology.normal_to_main={native.offsets.data(),native.offsets.size(),native.incidence.data(),native.incidence.size()};
      in.topology.primary_roles=native.primary_roles.data();in.topology.mixed_maps.primary_to_partner=native.primary_to_partner.data();
    }
    in.positions={positions.data(),std::uint32_t(mesh.points.size()),3,1};
    in.coordinates=mesh.coordinates;in.units=mesh.units;
    in.main_coefficients=coefficients.data();in.coefficient_count=coefficients.size();
    in.main_active=active.data();in.active_count=active.size();in.node_tag=tags.data();in.tag_count=tags.size();
    in.free_main_ids=free_ids.empty()?nullptr:free_ids.data();in.free_count=free_ids.size();
    const auto& cache=native_topology?native_prior:prior;
    in.prior_normals=cache.data();in.prior_count=cache.size();return in;
  }
  void Deform(unsigned step) {
    const double angle=.13*step,co=std::cos(angle),si=std::sin(angle);
    for(std::size_t i=0;i<mesh.points.size();++i) {
      const auto a=mesh.points[i];const double z=a.z+.025*step*a.x*a.y;
      positions[3*i]=co*a.x-si*z+.2*step;positions[3*i+1]=a.y+.03*step*a.x;positions[3*i+2]=si*a.x+co*z;
    }
  }
  static c::Limits Limits(){return {256,160,1280,1280,8u<<20};}
};
inline current::Result Evaluate(const Fixture& f,const c::Input& in,c::Limits limits=Fixture::Limits()) {
  c::Forecast forecast;
  auto report=c::Preflight(in,f.startup,limits,forecast);
  if(report.status!=c::Status::Ok)return {report,{},{}};
  tl::util::HostArena scratch;
  if(!scratch.Initialize(forecast.scratch_bytes))throw std::runtime_error("Mixed normal scratch allocation failed");
  current::Result out;out.normals.resize(in.prior_count);out.references.resize(in.topology.references);
  out.report=c::Evaluate(in,f.startup,limits,scratch.data(),scratch.bytes(),
      {out.normals.data(),out.normals.size(),out.references.data(),out.references.size()});return out;
}
}
