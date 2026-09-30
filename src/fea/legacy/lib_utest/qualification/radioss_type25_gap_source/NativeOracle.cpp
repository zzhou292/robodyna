// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <stdexcept>
extern "C" void rd_gap_source(const int*,const int*,const double*,
  const int*,const double*,const int*,const double*,const int*,const double*,
  const int*,const int*,const double*,const int*,const int*,const int*,const int*,
  double*,double*,double*,double*,double*);
namespace gap_source_test {
Result Oracle(const g::Input& in) {
  if(!in.node_count||in.node_count>32||in.shell_count>32||in.truss_count>16||
      in.beam_count>16||in.spring_count>16||in.secondary_count>32||
      in.main_node_count>32||in.main_count>64||in.primary_count>in.main_count||
      in.profile.property_type!=1||in.profile.level!=1||in.profile.gap_mode!=1||
      in.profile.free_edge_gap!=0||in.profile.contact_thickness_update!=0||
      (in.profile.input_thickness_mode!=0&&in.profile.input_thickness_mode!=1))
    throw std::invalid_argument("Native gap fixture exceeds selected profile/cap");
  int counts[10]{int(in.node_count),0,0,int(in.truss_count),int(in.beam_count),int(in.spring_count),
    int(in.secondary_count),int(in.main_count),int(in.primary_count),int(in.main_node_count)};
  int shn[128]{},tn[32]{},bn[32]{},rn[32]{},rt[16]{},mains[256]{},roles[64]{},sn[32]{},mn[32]{};
  double shv[96]{},tv[32]{},bv[32]{},rv[16]{};
  const auto node=[&](std::uint32_t value) {
    if(value>=in.node_count)throw std::invalid_argument("Native gap fixture node outside domain");
    return int(value+1);
  };
  for(std::size_t i=0;i<in.shell_count;++i) {
    const auto& row=in.shells[i];
    if(row.layout==n::ShellLayout::Quad4) {
      if(counts[2])throw std::invalid_argument("Native shell family order");
      ++counts[1];
    } else if(row.layout==n::ShellLayout::Triangle3) {
      if(row.nodes[2]!=row.nodes[3])throw std::invalid_argument("Native T3 repeat");
      ++counts[2];
    } else throw std::invalid_argument("Native shell layout");
    for(unsigned k=0;k<4;++k)shn[4*i+k]=node(row.nodes[k]);
    shv[3*i]=row.part_contact_thickness;shv[3*i+1]=row.element_thickness;shv[3*i+2]=row.property_thickness;
  }
  for(unsigned f=0;f<2;++f) {
    const auto* rows=f?in.beams:in.trusses;const auto count=f?in.beam_count:in.truss_count;
    auto* nodes=f?bn:tn;auto* values=f?bv:tv;
    for(std::size_t i=0;i<count;++i) {
      nodes[2*i]=node(rows[i].nodes[0]);nodes[2*i+1]=node(rows[i].nodes[1]);
      values[2*i]=rows[i].part_contact_thickness;values[2*i+1]=rows[i].native_area;
    }
  }
  for(std::size_t i=0;i<in.spring_count;++i) {
    const auto& row=in.springs[i];
    if(row.property_type!=13&&row.property_type!=25&&row.property_type!=45)
      throw std::invalid_argument("Native gap fixture spring branch");
    rn[2*i]=node(row.nodes[0]);rn[2*i+1]=node(row.nodes[1]);rt[i]=row.property_type;rv[i]=row.part_contact_thickness;
  }
  for(std::size_t i=0;i<in.main_count;++i) {
    for(unsigned k=0;k<4;++k)mains[4*i+k]=node(in.mains[i].nodes[k]);
    roles[i]=in.mains[i].segment_type;
  }
  for(std::size_t i=0;i<in.secondary_count;++i)sn[i]=node(in.secondary_nodes[i]);
  for(std::size_t i=0;i<in.main_node_count;++i)mn[i]=node(in.main_nodes[i]);
  const int mode=in.profile.input_thickness_mode;
  const double scale[]{in.profile.scale,in.profile.maximum_secondary,in.profile.maximum_main};
  double sec[32]{},msr[32]{},corners[256]{},maximum[64]{},extrema[2]{};
  rd_gap_source(counts,&mode,scale,shn,shv,tn,tv,bn,bv,rn,rt,rv,mains,roles,sn,mn,
    sec,msr,corners,maximum,extrema);
  Result out;out.secondary.assign(sec,sec+in.secondary_count);out.main_nodes.assign(msr,msr+in.main_node_count);
  out.mains.resize(in.main_count);
  for(std::size_t i=0;i<in.main_count;++i) {
    for(unsigned k=0;k<4;++k)out.mains[i].corner[k]=corners[4*i+k];
    out.mains[i].maximum=maximum[i];
  }
  out.minimum_secondary=extrema[0];out.maximum_secondary=extrema[1];
  return out;
}
} // namespace gap_source_test
