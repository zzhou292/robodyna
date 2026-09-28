#pragma once
#include "NativeOracle.h"
#include "lib_src/collision/RadiossType25Search.h"
#include <gtest/gtest.h>
#include <cstring>
namespace type25_search_test {
struct Fixture {
  std::vector<std::uint32_t> secondary,main,one_d;
  std::vector<double> positions,velocities,stiffness,gaps;
  std::vector<std::uint8_t> main_activity;
  s::Source source;
  s::QueryStamp stamp{{17,3,5},0,1};
  explicit Fixture(bool compact=true,bool updated_gaps=true,std::size_t nodes=32) {
    if(!compact)nodes=4;
    positions.resize(3*nodes);velocities.resize(3*nodes);
    secondary=compact ? std::vector<std::uint32_t>{0,2} : std::vector<std::uint32_t>{0,1,2};
    main=compact ? std::vector<std::uint32_t>{1,3} : std::vector<std::uint32_t>{0,2,3};
    one_d={compact ? 4u : 1u,UINT32_MAX};
    stiffness.assign(secondary.size(),2.);gaps={.5,.75};
    for(std::size_t i=0;i<nodes;++i) {positions[3*i]=double(i);positions[3*i+1]=2*double(i);positions[3*i+2]=3*double(i);}
    source.stamp=stamp.source;source.units={.001,1000,1};source.physical_nodes=nodes;
    source.main_segments=updated_gaps ? gaps.size() : 0;source.margin=4;
    source.gap_mode=updated_gaps ? s::GapMode::CurrentMainGaps : s::GapMode::Fixed;
    Bind();
  }
  void EnableRetirement() {
    source.activity_policy=s::ActivityPolicy::MonotoneRetirement;
    main_activity.assign(source.physical_nodes,1);
  }
  void Bind() {
    source.secondary_nodes=secondary.empty()?nullptr:secondary.data();source.secondaries=secondary.size();
    source.main_nodes=main.empty()?nullptr:main.data();source.mains=main.size();
    source.main_1d_nodes=one_d.empty()?nullptr:one_d.data();source.main_1d=one_d.size();
  }
  s::Current Current() const {
    s::Current c;c.stamp=stamp;
    c.positions={positions.data(),std::uint32_t(source.physical_nodes),3,1};
    c.velocities={velocities.data(),std::uint32_t(source.physical_nodes),3,1};
    c.secondary_stiffness=stiffness.data();c.secondary_count=stiffness.size();
    if(source.gap_mode==s::GapMode::CurrentMainGaps){c.main_gaps=gaps.data();c.main_gap_count=gaps.size();}
    if(!main_activity.empty()){c.main_node_activity=main_activity.data();c.main_node_activity_count=main_activity.size();}
    return c;
  }
  void ToSi() {
    const double length=source.units.length_m,velocity=length/source.units.time_s;
    for(auto& x:positions)x*=length;for(auto& v:velocities)v*=velocity;for(auto& g:gaps)g*=length;
    source.input_units=s::InputUnits::Si;
  }
};
inline void Same(double a,double b) {
  // Exact source arithmetic apart from the sign of equal extrema zeros, whose
  // native OpenMP max/min tie order is not a physical or search decision.
  if(a==0 && b==0)return;
  std::uint64_t x=0,y=0;std::memcpy(&x,&a,8);std::memcpy(&y,&b,8);EXPECT_EQ(x,y)<<a<<" vs "<<b;
}
inline void Same(const s::Budget& a,const s::Budget& b) {
  Same(a.displacement,b.displacement);Same(a.relative_speed,b.relative_speed);
  Same(a.raw_motion,b.raw_motion);Same(a.stored_motion,b.stored_motion);
  Same(a.raw_distance,b.raw_distance);Same(a.distance,b.distance);
  EXPECT_EQ(a.velocity,b.velocity);EXPECT_EQ(a.requires_sort,b.requires_sort);
}
inline void Same(const s::Extrema& a,const s::Extrema& b) {
  const s::MotionExtrema* x[]{&a.secondary_displacement,&a.main_displacement,&a.secondary_velocity,&a.main_velocity};
  const s::MotionExtrema* y[]{&b.secondary_displacement,&b.main_displacement,&b.secondary_velocity,&b.main_velocity};
  for(unsigned i=0;i<4;++i) {
    Same(x[i]->maximum.x,y[i]->maximum.x);Same(x[i]->maximum.y,y[i]->maximum.y);Same(x[i]->maximum.z,y[i]->maximum.z);
    Same(x[i]->minimum.x,y[i]->minimum.x);Same(x[i]->minimum.y,y[i]->minimum.y);Same(x[i]->minimum.z,y[i]->minimum.z);
  }
  Same(a.maximum_gap_change,b.maximum_gap_change);
  EXPECT_EQ(a.secondary_uses,b.secondary_uses);EXPECT_EQ(a.main_uses,b.main_uses);
}
} // namespace type25_search_test
