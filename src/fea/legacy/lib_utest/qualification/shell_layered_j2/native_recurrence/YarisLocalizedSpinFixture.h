#pragma once
#include "LayeredNativePath.h"
// Original 2010 Yaris v1l, EIDs2214871/2214872, PID/MID/SECID2000145,
// curve2100180. Exact original world geometry, never projected/flattened.
// Source member SHA256 67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301.
// Authenticated assembly inventory SHA256 afbc9cc6b9cbbceec766e1aa106b548fcc7468ce1a0d902d5d7b69afb1873d00.
// Shell source lines131793/131794; node source lines488932-488946.
namespace layered_j2_test::recurrence::source_pair {
inline constexpr unsigned ElementIds[]{2214871,2214872},SelectedLocal[]{1,0};
inline constexpr double Strain[]{0.0,0.025,0.05,0.075,0.1,0.125,0.15,0.175,0.2,0.225,0.25,0.275,0.3,0.325,0.35,0.4,0.5};
inline constexpr double Yield[]{180000000.0,219000000.0,247000000.0,271000000.0,290000000.0,307000000.0,321000000.0,334000000.0,345000000.0,355000000.0,365000000.0,374000000.0,381000000.0,388000000.0,394000000.0,401000000.0,410000000.0};
inline q::ReferenceInput Reference(unsigned parent) {
  q::ReferenceInput r;r.thickness=.000889;r.density=7889.999999999999;
  if(parent==0) {
    r.node_ids[0]=2181593;r.position[0]={-0.17684901,0.4577793,0.6779025900000001};
    r.node_ids[1]=2181592;r.position[1]={-0.17334799,0.45285390999999997,0.6596304300000001};
    r.node_ids[2]=2181580;r.position[2]={-0.17854669,0.45006744,0.65798596};
    r.node_ids[3]=2181581;r.position[3]={-0.18084753,0.45418924,0.6749058800000001};
  }
  if(parent==1) {
    r.node_ids[0]=2181592;r.position[0]={-0.17334799,0.45285390999999997,0.6596304300000001};
    r.node_ids[1]=2181591;r.position[1]={-0.17386976999999998,0.44759161000000003,0.64360052};
    r.node_ids[2]=2181579;r.position[2]={-0.17966743,0.44565161000000003,0.64295227};
    r.node_ids[3]=2181580;r.position[3]={-0.17854669,0.45006744,0.65798596};
  }
  return r;
}
inline sec::PointParameters Material() {
  sec::PointParameters p;
  EXPECT_EQ(tl::material::PrepareTabulatedShellPlasticity(200e9,.3,7889.999999999999,
    {Strain,Yield,17},{true,8000,8,10000},p),sec::PointStatus::Ok);return p;
}
inline oracle::Law44 Law() {return {Strain,Yield,17,8000,8,10000};}
inline Vec3 World(const nq::Matrix3& m,Vec3 v) {
  return {m.v[0]*v.x+m.v[1]*v.y+m.v[2]*v.z,m.v[3]*v.x+m.v[4]*v.y+m.v[5]*v.z,m.v[6]*v.x+m.v[7]*v.y+m.v[8]*v.z};
}
inline Vec3 Local(const nq::Matrix3& m,Vec3 v) {
  return {m.v[0]*v.x+m.v[3]*v.y+m.v[6]*v.z,m.v[1]*v.x+m.v[4]*v.y+m.v[7]*v.z,m.v[2]*v.x+m.v[5]*v.y+m.v[8]*v.z};
}
// Both parents use one reference frame/anchor for compatible shared-node x/v.
// This is a prescribed loading path, not the actual component trajectory.
struct Preload {
  nq::Matrix3 frame;Vec3 anchor;
  q::PrescribedInterval Interval(const q::ReferenceInput& r,unsigned step,double dt) const {
    q::PrescribedInterval in;in.dt=dt;in.base_time=step*dt;in.sample_index=step+1;
    for(unsigned n=0;n<4;++n) {
      const auto x=r.position[n];const Vec3 rel{x.x-anchor.x,x.y-anchor.y,x.z-anchor.z};
      const auto local=Local(frame,rel),u=World(frame,Path::Shape(local));
      in.position_endpoint[n]=Add(x,Scale(u,(step+1)*dt));in.velocity_midpoint[n]=u;
      in.omega_midpoint[n]=World(frame,cv::Path::Omega(local,1));
    }return in;
  }
};
} // namespace layered_j2_test::recurrence::source_pair
