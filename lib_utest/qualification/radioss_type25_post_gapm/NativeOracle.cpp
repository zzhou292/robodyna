#include "NativeOracle.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <map>
#include <stdexcept>
namespace type25_post_gapm_test {
extern "C" void rd_post_gapm_startup(const int*,const double*,const int*,const int*,const double*,
    int*,int*,int*,int*,int*,int*,int*,int*,float*,float*,int*,int*,int*,float*,float*,float*,int*,
    const int*,const int*,const int*,const int*);
namespace {
void Need(bool value){if(!value)throw std::invalid_argument("Malformed bounded native mixed post-GAPM packet");}
}
NativeResult Oracle(const s::Input& in,const s::PostGapmTopology& post,const double* coefficient,std::size_t coefficient_count) {
  const auto p=in.primary_count,g=p+in.shell_primary_count,cap=4*g;
  Need(in.profile==s::Profile::MixedSurface && in.topology==s::TopologyPolicy::NativeMixedSurface &&
      in.node_count && in.node_count<=256 && p && p<=160 && in.shell_primary_count<=p &&
      in.node_source_ids && in.primary && in.primary_identities && in.primary_identity_count==p &&
      in.positions.valid() && in.positions.node_count==in.node_count && coefficient && coefficient_count==g);
  Need(post.phase==s::PostGapmPhase::FinalizedBeforeNeighbors && post.primary_count==p &&
      post.before_shell_count==p && post.main_count==g && post.primary_corners && post.before_shell && post.final_support);
  Need(post.final_solid_erosion==s::SolidErosion::Disabled || post.final_solid_erosion==s::SolidErosion::Enabled);
  std::vector<double> x(3*in.node_count);
  std::vector<int> ids(in.node_count),primary(4*p),source_roles(p),corners(4*p),physical(2*g);
  double length=1;
  if(in.coordinates==s::Coordinates::Si){length=in.units.length_m;Need(std::isfinite(length)&&length>0);}
  else Need(in.coordinates==s::Coordinates::Native);
  for(std::size_t i=0;i<in.node_count;++i) {
    Need(in.node_source_ids[i] && in.node_source_ids[i]<=INT_MAX);
    ids[i]=int(in.node_source_ids[i]);const auto v=in.positions.at(std::uint32_t(i));
    x[3*i]=v.x/length;x[3*i+1]=v.y/length;x[3*i+2]=v.z/length;
    for(unsigned k=0;k<3;++k)Need(std::isfinite(x[3*i+k]));
  }
  std::size_t shell_count=0;
  for(std::size_t i=0;i<p;++i) {
    const auto& f=in.primary[i];const bool tri=f.layout==n::ShellLayout::Triangle3;
    Need(tri || f.layout==n::ShellLayout::Quad4);
    const auto role=f.side_role;
    Need(role==s::ShellSideRole::Ordinary || role==s::ShellSideRole::CoatingForward || role==s::ShellSideRole::CoatingReversed);
    const bool solid=in.primary_identities[i].kind==s::PrimaryFaceKind::Solid;
    Need(solid || in.primary_identities[i].kind==s::PrimaryFaceKind::Shell);
    if(solid){Need(role==s::ShellSideRole::Ordinary);source_roles[i]=1;}
    else {++shell_count;const int ordinary=tri?7:3;
      source_roles[i]=role==s::ShellSideRole::Ordinary?ordinary:role==s::ShellSideRole::CoatingForward?ordinary+1:-(ordinary+1);}
    for(unsigned k=0;k<4;++k) {
      Need(f.nodes[k]<in.node_count && post.primary_corners[i].source_corner[k]<4);
      primary[4*i+k]=int(f.nodes[k]+1);corners[4*i+k]=int(post.primary_corners[i].source_corner[k]+1);
    }
  }
  Need(shell_count==in.shell_primary_count);
  // NEIGH consumes only the solid-vs-shell ordinal partition and nonzero
  // second support. Private kind-preserving packing preserves those exact predicates;
  // it is not a claimed native vehicle storage order or an owner selection.
  std::map<std::uint64_t,int> solids;
  for(std::size_t i=0;i<g;++i) {
    const auto& a=post.final_support[i];Need(a.first.source_element_id && std::isfinite(coefficient[i]));
    if(a.first.kind==s::PhysicalSupportKind::EightSlotSolid) {
      solids.emplace(a.first.source_element_id,0);
      if(a.second_solid_source_id)solids.emplace(a.second_solid_source_id,0);
    } else Need((a.first.kind==s::PhysicalSupportKind::ShellQuad || a.first.kind==s::PhysicalSupportKind::ShellTriangle) && !a.second_solid_source_id);
  }
  int count=0;for(auto& entry:solids)entry.second=++count;
  for(std::size_t i=0;i<g;++i) {
    const auto& a=post.final_support[i];
    physical[2*i]=a.first.kind==s::PhysicalSupportKind::EightSlotSolid?solids.at(a.first.source_element_id):count+1;
    physical[2*i+1]=a.second_solid_source_id?solids.at(a.second_solid_source_id):0;
  }
  const int counts[]{int(in.node_count),int(p),int(shell_count),count};
  const int erosion=post.final_solid_erosion==s::SolidErosion::Enabled?1:0;
  std::vector<int> connectivity(4*g),roles(g),globals(g),neighbors(4*g),edges(4*g),refs(4*g);
  std::vector<int> start_bound(cap),ready_bound(cap),offsets(cap+1),incidence(cap);
  std::vector<float> start_normals(12*g),ready_normals(12*g),start_bisectors(6*cap),ready_bisectors(6*cap);
  NativeResult out;int reference_count=0;int warnings[4]{};
  rd_post_gapm_startup(counts,x.data(),ids.data(),primary.data(),coefficient,connectivity.data(),roles.data(),
      globals.data(),neighbors.data(),edges.data(),refs.data(),&reference_count,start_bound.data(),start_normals.data(),
      start_bisectors.data(),offsets.data(),incidence.data(),ready_bound.data(),ready_normals.data(),ready_bisectors.data(),
      out.floors.data(),warnings,source_roles.data(),corners.data(),physical.data(),&erosion);
  Need(reference_count>0 && std::size_t(reference_count)<=cap);
  out.warning_count=warnings[0];out.warning_node_ids={warnings[1],warnings[2]};out.selector_calls=warnings[3];
  out.mains.resize(g);out.expanded_to_primary.resize(g);out.primary_to_partner.resize(p);
  for(std::size_t i=0;i<p;++i) {
    out.primary_roles.push_back(in.primary[i].side_role);
    const auto partner=roles[i]>int(g)?roles[i]-int(g):roles[i];
    Need(partner==0 || (partner>int(p)&&std::size_t(partner)<=g));
    out.primary_to_partner[i]=std::uint32_t(partner);
    out.expanded_to_primary[i]=std::uint32_t(i);
    if(partner)out.expanded_to_primary[std::size_t(partner-1)]=std::uint32_t(i);
  }
  out.starter_normals.resize(4*g);out.ready_normals.resize(4*g);
  for(std::size_t m=0;m<g;++m) {
    auto& a=out.mains[m];a.source_id=in.primary[out.expanded_to_primary[m]].source_id;
    a.global_id=globals[m];a.segment_type=roles[m];
    for(unsigned k=0;k<4;++k) {
      const auto i=4*m+k;Need(connectivity[i]>0 && std::size_t(connectivity[i])<=in.node_count);
      a.nodes[k]=std::uint32_t(connectivity[i]-1);a.neighbors[k]=neighbors[i];a.neighbor_edges[k]=edges[i];a.normal_reference[k]=refs[i];
      out.starter_normals[i]={start_normals[3*i],start_normals[3*i+1],start_normals[3*i+2]};
      out.ready_normals[i]={ready_normals[3*i],ready_normals[3*i+1],ready_normals[3*i+2]};
    }
  }
  const auto nr=std::size_t(reference_count);
  out.starter_references.resize(nr);out.ready_references.resize(nr);
  for(std::size_t i=0;i<nr;++i) {
    out.starter_references[i].boundary=start_bound[i];out.ready_references[i].boundary=ready_bound[i];
    for(unsigned k=0;k<2;++k) {
      const auto j=6*i+3*k;
      out.starter_references[i].bisector[k]={start_bisectors[j],start_bisectors[j+1],start_bisectors[j+2]};
      out.ready_references[i].bisector[k]={ready_bisectors[j],ready_bisectors[j+1],ready_bisectors[j+2]};
    }
  }
  out.offsets.assign(offsets.begin(),offsets.begin()+reference_count+1);
  Need(offsets[nr]>=0 && std::size_t(offsets[nr])<=cap);
  out.incidence.assign(incidence.begin(),incidence.begin()+offsets[nr]);
  return out;
}
}
