// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
namespace type25_selection_test {
struct NewImpactCase {
  std::string name;s::NativeNewImpactInput input;n::NativeGeometryHistory prior;
};
inline void SetImpactRole(NewImpactCase& c,int role) {
  auto& in=c.input;in.pair.segment_type=role;in.opposite={};
  const int local=role>0?(role>in.segment_count?role-in.segment_count:role):0;
  if(!local)return;
  in.opposite.local_main=local;in.opposite.global_main=13;
  constexpr unsigned nodes[]{1,0,3,2},edges[]{0,3,2,1};
  // Explicit synthetic two-side catalogue for qualification. Runtime never
  // generates missing partner metadata by this fixture construction.
  for(unsigned i=0;i<4;++i) {
    in.opposite.main_node_ids[i]=in.pair.main_node_ids[nodes[i]];
    in.opposite.normal_slot[i]=in.pair.normal_slot[edges[i]];
    in.opposite.neighbors[i]=in.pair.neighbors[edges[i]];
    in.opposite.boundary_ids[i]=in.pair.boundary_ids[nodes[i]];
    for(unsigned j=0;j<2;++j)in.opposite.vertex_bisector[i][j]=in.pair.vertex_bisector[nodes[i]][j];
  }
}
inline NewImpactCase Impact(const Case& source) {
  NewImpactCase out;out.name=source.name;out.input.pair=source.input;out.prior=source.prior;
  out.input.segment_count=16;out.input.previous_dt=.001;
  out.prior.row.irtlm[0]=out.prior.row.irtlm[1]=out.prior.row.irtlm[2]=out.prior.row.irtlm[3]=0;
  out.prior.row.selection_metric[0]=-1e20;
  SetImpactRole(out,source.input.segment_type);return out;
}
inline NewImpactCase BasicNewImpact() {return Impact(Basic());}
inline std::vector<NewImpactCase> NewImpactCases() {
  std::vector<NewImpactCase> cases;
  for(const auto& c:Cases())cases.push_back(Impact(c));
  for(bool triangle:{false,true})for(int role:{0,4,-4,17,-17})
    for(double height:{-.2,0.,.2,2.})for(double velocity:{-10000.,0.,10000.}) {
      auto c=Impact(FromGeometry({"motion-role",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
      c.input.pair.secondary.z=height;c.input.secondary_velocity.z=velocity;SetImpactRole(c,role);
      cases.push_back(c);
    }
  for(bool triangle:{false,true})for(int initial:{-1,0,1}) {
    auto c=Impact(FromGeometry({"recontact",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
    SetImpactRole(c,0);c.input.pair.secondary.z=-1e-6;c.input.pair.secondary_gap=0;
    for(auto& gap:c.input.pair.main_gap)gap=0;
    c.input.pair.main_gap_max=0;c.input.pair.initial_contact_flag=initial;cases.push_back(c);
  }
  for(bool triangle:{false,true})for(double dt:{0.,.001})for(bool unequal:{false,true}) {
    auto c=Impact(FromGeometry({"moving-main",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
    c.input.previous_dt=dt;
    for(auto& velocity:c.input.main_velocity)velocity={0,0,-1000};
    if(unequal) {
      c.input.main_velocity[0]={25,-10,-1000};c.input.main_velocity[1]={-50,20,-800};
      c.input.main_velocity[2]={30,-40,-1200};c.input.main_velocity[3]={-10,-20,-900};
      if(triangle)c.input.main_velocity[3]=c.input.main_velocity[2];
    }
    cases.push_back(c);
  }
  return cases;
}
} // namespace type25_selection_test
