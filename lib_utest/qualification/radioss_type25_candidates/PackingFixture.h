#pragma once
#include "Fixture.h"
#include "lib_src/collision/radioss_type25/candidates/Packing.h"
namespace candidate_test {
extern "C" void rd_native_pack(const double*,const double*,const double*,const int*,double*,int*);
struct NativePacking {double gap=0;int symmetry=0;};
inline NativePacking NativePack(const c::LocalRow& row) {
  double xyz[15],v[15];
  for(unsigned i=0;i<4;++i) {
    xyz[3*i]=row.screen.vertices[i].x;xyz[3*i+1]=row.screen.vertices[i].y;xyz[3*i+2]=row.screen.vertices[i].z;
    v[3*i]=row.main_velocities[i].x;v[3*i+1]=row.main_velocities[i].y;v[3*i+2]=row.main_velocities[i].z;
  }
  xyz[12]=row.screen.secondary.x;xyz[13]=row.screen.secondary.y;xyz[14]=row.screen.secondary.z;
  v[12]=row.secondary_velocity.x;v[13]=row.secondary_velocity.y;v[14]=row.secondary_velocity.z;
  double controls[]{row.screen.secondary_gap,row.screen.main_gap,row.screen.curvature,row.screen.drad,
                    row.screen.gap_load,row.previous_dt,0.};
  int flags[]{row.nodes[2]==row.nodes[3]?1:0,row.segment_type,row.constraint_codes[0],
              row.constraint_codes[1],row.constraint_codes[2],row.constraint_codes[3],row.constraint_codes[4]};
  NativePacking result;rd_native_pack(xyz,v,controls,flags,&result.gap,&result.symmetry);return result;
}
inline std::vector<c::LocalRow> PackingCases() {
  std::vector<c::LocalRow> output;
  const auto cases=Cases();
  for(std::size_t i=0;i<cases.size();++i) {
    const auto p=Pack(cases[i]);c::LocalRow r;r.secondary_node=5;r.segment_type=p.segment_type;r.main_count=2;
    for(unsigned j=0;j<4;++j){r.nodes[j]=p.nodes[j];r.screen.vertices[j]=p.vertices[j];
      r.main_velocities[j]={double(r.nodes[j])*.125,-double(r.nodes[j])*.5,double(r.nodes[j])*.25};
      r.constraint_codes[j]=int(r.nodes[j]+i)%8;}
    r.screen.secondary=p.secondary;r.screen.secondary_gap=p.gap;r.screen.margin=p.margin;
    r.screen.main_gap=.125;r.screen.curvature=.03125;r.screen.gap_load=(i%2)?-.0625:.0625;
    r.screen.drad=(i%3)?0.:.5;r.screen.stored_motion=.25;
    r.previous_dt=(i%4)*.125;r.secondary_velocity={-.125,.5,1.};r.constraint_codes[4]=i%8;
    output.push_back(r);
  }
  return output;
}
} // namespace candidate_test
