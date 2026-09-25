#pragma once
#include "Oracle.h"
#include "lib_src/collision/radioss_type25/candidates/Screen.h"
#include <cstring>
namespace candidate_test {
namespace c=tlfea::contact::radioss_type25::candidates;
inline c::PackedRow Pack(const pen3_test::Case& item) {
  c::PackedRow row;
  for(unsigned i=0;i<4;++i) {
    row.nodes[i]=item.target.flags[i];
    row.vertices[i]={item.target.coordinates[3*i],item.target.coordinates[3*i+1],item.target.coordinates[3*i+2]};
  }
  row.secondary={item.target.coordinates[12],item.target.coordinates[13],item.target.coordinates[14]};
  row.gap=item.target.gap;row.margin=item.margin;row.main_count=2;
  row.segment_type=item.target.flags[4];row.symmetry=item.target.flags[5];return row;
}
inline std::vector<pen3_test::Case> Cases() {
  auto cases=pen3_test::Cases();
  for(double scale:{0x1p-30,1.,0x1p30})for(double offset:{0.,0x1p20})
    for(unsigned axis=0;axis<3;++axis)for(unsigned shape=0;shape<4;++shape)
      for(unsigned position=0;position<5;++position)for(unsigned side=0;side<2;++side) {
        pen3_test::Packet p;p.flags={1,2,3,4,1,0};
        double data[15]={-1,-1,0, 1,-1,0, 1,1,0, -1,1,0, 0,0,.25};
        if(shape==1)data[8]=.25; // warped
        if(shape==2){data[7]=-1;data[10]=-1;} // collinear
        if(shape==3)for(unsigned i=1;i<4;++i)for(unsigned j=0;j<3;++j)data[3*i+j]=data[j];
        if(position==1)data[12]=1.125;
        if(position==2){data[12]=-1.125;data[13]=-1.125;}
        if(position==3){data[12]=1.;data[13]=1.;}
        if(position==4)data[12]=std::nextafter(1.,INFINITY);
        if(side)data[14]=-data[14];
        for(unsigned i=0;i<5;++i)for(unsigned j=0;j<3;++j)
          p.coordinates[3*i+(j+axis)%3]=offset+scale*data[3*i+j];
        for(double gap:{0.,std::nextafter(.25*scale,0.),.25*scale,
                        std::nextafter(.25*scale,INFINITY),.5*scale})
          for(double margin:{0.,.125*scale}) {
            p.gap=gap;cases.push_back({p,margin,"quad-"+std::to_string(shape)});
          }
      }
  for(int type:{0,1,3,-3})for(int symmetry=0;symmetry<8;++symmetry) {
    auto p=pen3_test::Unrelated(false);p.coordinates[14]=100.;p.gap=1.;
    p.flags[4]=type;p.flags[5]=symmetry;cases.push_back({p,0.,"quad-symmetry"});
  }
  return cases;
}
extern "C" void rd_native_screen(const double*,const double*,int*);
inline bool NativeScreen(const c::ScreenRow& row) {
  double xyz[15];for(unsigned i=0;i<4;++i) {
    xyz[3*i]=row.vertices[i].x;xyz[3*i+1]=row.vertices[i].y;xyz[3*i+2]=row.vertices[i].z;
  }
  xyz[12]=row.secondary.x;xyz[13]=row.secondary.y;xyz[14]=row.secondary.z;
  double controls[]{row.margin,row.curvature,row.secondary_gap,row.main_gap,
                    row.gap_load,row.drad,row.stored_motion};
  int included=0;rd_native_screen(xyz,controls,&included);return included!=0;
}
inline std::uint64_t Bits(double x) {std::uint64_t out;std::memcpy(&out,&x,sizeof(out));return out;}
} // namespace candidate_test
