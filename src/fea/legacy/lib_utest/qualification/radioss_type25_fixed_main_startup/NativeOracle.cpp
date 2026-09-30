// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <climits>
#include <cmath>
#include <stdexcept>
namespace type25_startup_test {
extern "C" void rd_fixed_main_startup(const int*,const double*,const int*,const int*,const double*,
    int*,int*,int*,int*,int*,int*,int*,int*,float*,float*,int*,int*,int*,float*,float*,float*,int*,const int*);
namespace {
void Need(bool value,const char* text){if(!value)throw std::invalid_argument(text);}
}
NativeResult Oracle(const s::Input& in,const double* coefficient,std::size_t coefficient_count) {
  Need(in.node_count&&in.node_count<=256&&in.primary_count&&in.primary_count<=160&&
      in.node_source_ids&&in.primary&&in.positions.valid()&&in.positions.node_count==in.node_count,
      "Native startup fixture exceeds its declared source bounds");
  const auto g=2*in.primary_count,cap=4*g;
  const bool resolved=in.profile==s::Profile::ResolvedShellSides && in.topology==s::TopologyPolicy::NativeResolvedShellSides;
  Need(resolved || ((in.profile==s::Profile::OrdinaryExteriorFixedMain || in.profile==s::Profile::OrdinaryExteriorMovingMain) &&
      (in.topology==s::TopologyPolicy::ManifoldTwoSided || in.topology==s::TopologyPolicy::NativeOrdinaryShell)),
      "Unselected native startup profile");
  std::vector<int> source_roles(in.primary_count);
  Need(coefficient&&coefficient_count==g,"Native ready fixture needs actual expanded coefficients");
  for(std::size_t i=0;i<g;++i)Need(std::isfinite(coefficient[i])&&coefficient[i]>0,"Unsupported native ready activity");
  const int counts[]{int(in.node_count),int(in.primary_count)};
  std::vector<double> x(3*in.node_count);
  std::vector<int> ids(in.node_count),primary(4*in.primary_count);
  double length=1;
  if(in.coordinates==s::Coordinates::Si) {
    Need(std::isfinite(in.units.length_m)&&in.units.length_m>0,"Invalid native length scale");
    length=in.units.length_m;
  } else Need(in.coordinates==s::Coordinates::Native,"Unknown startup coordinate units");
  for(std::size_t i=0;i<in.node_count;++i) {
    Need(in.node_source_ids[i]&&in.node_source_ids[i]<=INT_MAX,"Native source node ID is not a positive integer");
    ids[i]=int(in.node_source_ids[i]);const auto v=in.positions.at(std::uint32_t(i));
    x[3*i]=v.x/length;x[3*i+1]=v.y/length;x[3*i+2]=v.z/length;
    for(unsigned k=0;k<3;++k)Need(std::isfinite(x[3*i+k]),"Nonfinite native startup coordinate");
  }
  for(std::size_t i=0;i<in.primary_count;++i) {
    Need(in.primary[i].layout==n::ShellLayout::Quad4||in.primary[i].layout==n::ShellLayout::Triangle3,
        "Native reference admits shell Q4/T3 only");
    const auto role=in.primary[i].side_role;
    Need(role==s::ShellSideRole::Ordinary || (resolved &&
        (role==s::ShellSideRole::CoatingForward || role==s::ShellSideRole::CoatingReversed)),
        "Unselected native primary role");
    const int ordinary=in.primary[i].layout==n::ShellLayout::Triangle3?7:3;
    source_roles[i]=role==s::ShellSideRole::Ordinary?ordinary:
        role==s::ShellSideRole::CoatingForward?ordinary+1:-(ordinary+1);
    for(unsigned k=0;k<4;++k) {
      Need(in.primary[i].nodes[k]<in.node_count,"Native primary node is outside the table");
      primary[4*i+k]=int(in.primary[i].nodes[k]+1);
    }
  }
  std::vector<int> connectivity(4*g),roles(g),globals(g),neighbors(4*g),edges(4*g),refs(4*g);
  std::vector<int> start_bound(cap),ready_bound(cap),offsets(cap+1),incidence(cap);
  std::vector<float> start_normals(12*g),ready_normals(12*g),start_bisectors(6*cap),ready_bisectors(6*cap);
  NativeResult out;int reference_count=0;int warnings[4]{};
  rd_fixed_main_startup(counts,x.data(),ids.data(),primary.data(),coefficient,connectivity.data(),roles.data(),
      globals.data(),neighbors.data(),edges.data(),refs.data(),&reference_count,start_bound.data(),
      start_normals.data(),start_bisectors.data(),offsets.data(),incidence.data(),ready_bound.data(),
      ready_normals.data(),ready_bisectors.data(),out.floors.data(),warnings,source_roles.data());
  out.warning_count=warnings[0];out.warning_node_ids={warnings[1],warnings[2]};out.selector_calls=warnings[3];
  Need(reference_count>0&&std::size_t(reference_count)<=cap,"Native reference count is invalid");
  out.mains.resize(g);out.expanded_to_primary.resize(g);out.primary_to_partner.resize(in.primary_count);
  if(resolved)for(std::size_t i=0;i<in.primary_count;++i)out.primary_roles.push_back(in.primary[i].side_role);
  out.starter_normals.resize(4*g);out.ready_normals.resize(4*g);
  for(std::size_t m=0;m<g;++m) {
    // Every admitted input role appends exactly one side in original primary
    // order. This is wrapper identity mapping, not production numerical code.
    const auto parent=m<in.primary_count?m:m-in.primary_count;
    Need(parent<in.primary_count,"Native opposite role lost physical primary identity");
    auto& main=out.mains[m];main.source_id=in.primary[parent].source_id;
    main.global_id=globals[m];main.segment_type=roles[m];out.expanded_to_primary[m]=std::uint32_t(parent);
    if(m<in.primary_count) {
      int partner=roles[m];if(partner>int(g))partner-=int(g); // Original I25NORM ISH decode.
      out.primary_to_partner[m]=std::uint32_t(partner);
    }
    for(unsigned k=0;k<4;++k) {
      main.nodes[k]=std::uint32_t(connectivity[4*m+k]-1);main.neighbors[k]=neighbors[4*m+k];
      main.neighbor_edges[k]=edges[4*m+k];main.normal_reference[k]=refs[4*m+k];
      const auto j=3*(4*m+k);
      out.starter_normals[4*m+k]={start_normals[j],start_normals[j+1],start_normals[j+2]};
      out.ready_normals[4*m+k]={ready_normals[j],ready_normals[j+1],ready_normals[j+2]};
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
  Need(offsets[nr]>=0&&std::size_t(offsets[nr])<=cap,"Native CSR extent is invalid");
  out.incidence.assign(incidence.begin(),incidence.begin()+offsets[nr]);
  return out;
}
} // namespace type25_startup_test
