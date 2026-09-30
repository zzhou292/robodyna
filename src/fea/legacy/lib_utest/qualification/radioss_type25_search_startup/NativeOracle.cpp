// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cmath>
#include <stdexcept>
namespace type25_search_startup_test {
extern "C" void rd_search_startup(const int*,const double*,const int*,const int*,const int*,
    const double*,const double*,const double*,double*,double*,int*,int*,int*,int*,int*);
extern "C" void rd_startup_multiplier(const int*,double*);
namespace {
void Need(bool value) {if(!value)throw std::invalid_argument("Native search-startup fixture outside declared bounds");}
}
double OracleMultiplier(int nodes) {Need(nodes>0);double result=0;rd_startup_multiplier(&nodes,&result);return result;}
NativeResult Oracle(const s::Input& in) {
  const auto n=in.mesh.node_count,g=in.main_count,ns=in.secondary_count,p=in.topology.primary_count;
  Need(n&&n<=4096&&g&&g<=512&&ns&&ns<=4096&&g==2*p&&in.topology.mains&&in.secondary&&in.main_gaps);
  Need(in.mesh.positions.valid()&&in.mesh.positions.node_count==n&&(in.profile.curvature==0||in.profile.curvature==1));
  Need(in.contributors.native_auxiliary_nodes<=4096-n);
  const auto native_nodes=n+in.contributors.native_auxiliary_nodes;
  const int counts[]{int(native_nodes),int(g),int(ns),int(p),in.profile.curvature};
  // Auxiliary primaries are absent from every face/secondary. Their coordinates
  // are unused by these original routines; full native node/tag extents remain.
  std::vector<double>x(3*native_nodes),gaps(ns),stiffness(ns);
  std::vector<int>irect(4*g),roles(g),nodes(ns);
  const double length=in.mesh.coordinates==tlfea::contact::radioss_type25::startup::Coordinates::Native?1.:in.mesh.units.length_m;
  Need(std::isfinite(length)&&length>0);
  for(std::size_t i=0;i<n;++i) {
    const auto v=in.mesh.positions.at(std::uint32_t(i));
    x[3*i]=v.x/length;x[3*i+1]=v.y/length;x[3*i+2]=v.z/length;
    for(unsigned k=0;k<3;++k)Need(std::isfinite(x[3*i+k]));
  }
  for(std::size_t i=0;i<g;++i) {
    roles[i]=in.topology.mains[i].segment_type;
    for(unsigned k=0;k<4;++k){const auto node=in.topology.mains[i].nodes[k];Need(node<n);irect[4*i+k]=int(node+1);}
    Need(std::isfinite(in.main_gaps[i])&&in.main_gaps[i]>=0);
  }
  for(std::size_t i=0;i<ns;++i) {
    Need(in.secondary[i].node<n);nodes[i]=int(in.secondary[i].node+1);
    gaps[i]=in.secondary[i].gap;stiffness[i]=in.secondary[i].stiffness;
    Need(std::isfinite(gaps[i])&&gaps[i]>=0&&std::isfinite(stiffness[i]));
  }
  std::vector<int>krem(g+1),rem(g*ns),kremnor(ns+1),remnor(g*ns);
  NativeResult out;out.extent.resize(p);out.contact.resize(ns);
  rd_search_startup(counts,x.data(),irect.data(),roles.data(),nodes.data(),gaps.data(),stiffness.data(),in.main_gaps,
      out.scalar.data(),out.extent.data(),krem.data(),rem.data(),kremnor.data(),remnor.data(),out.contact.data());
  Need(krem.back()>=0&&std::size_t(krem.back())<=g*ns&&krem.back()==kremnor.back());
  const auto count=std::size_t(krem.back());out.main_offsets.assign(krem.begin(),krem.end());
  out.secondary_offsets.assign(kremnor.begin(),kremnor.end());out.removed_nodes.resize(count);
  for(std::size_t i=0;i<count;++i){Need(rem[i]>0&&std::size_t(rem[i])<=n);out.removed_nodes[i]=std::uint32_t(rem[i]-1);}
  out.removed_mains.assign(remnor.begin(),remnor.begin()+count);
  for(double v:out.scalar)Need(std::isfinite(v));for(double v:out.extent)Need(std::isfinite(v));
  return out;
}
} // namespace type25_search_startup_test
