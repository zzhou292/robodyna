#pragma once
#include "Fixture.h"
namespace candidate_test {
extern "C" void rd_pen3_inventory(const int*,const double*,const double*,const double*,const int*,const int*,double*);
inline double NativeThreshold(const c::PackedRow& row,int count,int role) {
  double xyz[30]{},gaps[]{row.gap,0.},out[2]{};int flags[12]{};
  for(unsigned i=0;i<4;++i){xyz[3*i]=row.vertices[i].x;xyz[3*i+1]=row.vertices[i].y;
    xyz[3*i+2]=row.vertices[i].z;flags[i]=int(row.nodes[i]);}
  xyz[12]=row.secondary.x;xyz[13]=row.secondary.y;xyz[14]=row.secondary.z;
  flags[4]=role;flags[5]=row.symmetry;const int one=1;
  rd_pen3_inventory(&one,xyz,gaps,&row.margin,flags,&count,out);return out[0];
}
struct PrimaryCase {c::PackedRow row;int primary=0,classification=0;};
inline std::vector<PrimaryCase> PrimaryCases() {
  std::vector<PrimaryCase> out;
  for(int primary:{1,2,3,4,8})for(int kind=0;kind<3;++kind)for(int mask=0;mask<8;++mask)
    for(unsigned shape=0;shape<2;++shape)for(unsigned axis=0;axis<3;++axis)
      for(double distance:{0.,std::nextafter(.002,0.),.002,std::nextafter(.002,INFINITY)}) {
        c::PackedRow r;r.main_count=primary;r.segment_type=kind==0?0:(kind==1?primary+1:3*primary+1);
        r.symmetry=mask;r.gap=.5;r.nodes[0]=1;r.nodes[1]=2;r.nodes[2]=3;r.nodes[3]=shape?4:3;
        const c::Vector points[5]={{-1,-1,0},{1,-1,0},{shape?1.:0.,1,0},{-1,1,0},{0,.125,distance}};
        auto rotate=[&](c::Vector p){return axis==0?p:(axis==1?c::Vector{p.z,p.x,p.y}:c::Vector{p.y,p.z,p.x});};
        for(unsigned i=0;i<4;++i)r.vertices[i]=rotate(points[i]);
        if(!shape)r.vertices[3]=r.vertices[2];r.secondary=rotate(points[4]);out.push_back({r,primary,2*primary});
      }
  return out;
}
} // namespace candidate_test
