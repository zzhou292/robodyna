// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CppReplay.h"
#include "NativeReplay.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <sstream>
#include <iomanip>
#include <vector>
namespace qeph_projection_test {
namespace {
std::string Text(double value){std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}
void Append(std::vector<double>& out,double value){out.push_back(value);}
template<class T,std::size_t N>
void Append(std::vector<double>& out,const std::array<T,N>& values){for(const auto& value:values)Append(out,value);}
void Same(const std::vector<double>& actual,const std::vector<double>& expected) {
  ASSERT_EQ(actual.size(),expected.size());
  double scale=1;
  for(double value:expected){ASSERT_TRUE(std::isfinite(value));scale=std::max(scale,std::abs(value));}
  const double tolerance=128*std::numeric_limits<double>::epsilon()*scale;
  for(std::size_t i=0;i<actual.size();++i) {
    SCOPED_TRACE(i);
    ASSERT_TRUE(std::isfinite(actual[i]));
    EXPECT_NEAR(actual[i],expected[i],tolerance);
  }
}
void Same(const Projection& a,const Projection& b) {
  ASSERT_EQ(a.planar,b.planar);
  ASSERT_EQ(a.warped_defined,b.warped_defined);
  Same(std::vector<double>{a.z1},std::vector<double>{b.z1});
  std::vector<double> x,y;
  Append(x,a.di);Append(x,a.db);Append(x,a.vqn);
  Append(y,b.di);Append(y,b.db);Append(y,b.vqn);
  if(a.warped_defined)Same(x,y);
  else for(unsigned i=0;i<x.size();++i){EXPECT_EQ(x[i],0);EXPECT_EQ(y[i],0);}
}
void Same(const RateResult& a,const RateResult& b) {
  Same(a.projection,b.projection);
  std::vector<double> x,y;
  Append(x,a.v13);Append(x,a.v24);Append(x,a.vhi);Append(x,a.rlxyz);
  Append(y,b.v13);Append(y,b.v24);Append(y,b.vhi);Append(y,b.rlxyz);
  Same(x,y);
}
void Same(const ForceResult& a,const ForceResult& b) {
  std::vector<double> x,y;
  Append(x,a.force);Append(x,a.couple);Append(y,b.force);Append(y,b.couple);
  Same(x,y);
}
std::vector<double> Fields(const RateInput& in) {
  const auto& g=in.geometry;
  std::vector<double> out{g.area,g.area_i,g.x13,g.x24,g.y13,g.y24,g.mx13,g.my13,g.z1,g.ll,g.l13,g.l24};
  Append(out,g.corel);Append(out,g.vq);Append(out,in.v13);Append(out,in.v24);Append(out,in.vhi);
  Append(out,in.rlxyz);Append(out,in.world_omega);return out;
}
V3 World(const Geometry& g,V3 x) {
  return {g.vq[0]*x[0]+g.vq[1]*x[1]+g.vq[2]*x[2],
    g.vq[3]*x[0]+g.vq[4]*x[1]+g.vq[5]*x[2],g.vq[6]*x[0]+g.vq[7]*x[1]+g.vq[8]*x[2]};
}
std::array<V3,4> Nodes(const RateResult& in) {
  std::array<V3,4> out;
  for(unsigned c=0;c<3;++c){out[0][c]=.25*in.vhi[c]+.5*in.v13[c];out[2][c]=.25*in.vhi[c]-.5*in.v13[c];
    out[1][c]=-.25*in.vhi[c]+.5*in.v24[c];out[3][c]=-.25*in.vhi[c]-.5*in.v24[c];}
  return out;
}
}
TEST(QephProjectionReplay,CompleteOriginalNativeStagesMatchCapturedDefinedChannels) {
  unsigned warped=0;
  for(const auto& row:CapturedRows()) {
    SCOPED_TRACE(row.cycle);
    SCOPED_TRACE(row.original_row);
    EXPECT_EQ(row.rate_entry.controls.npt,3);
    EXPECT_EQ(row.rate_entry.controls.idril,0);
    const auto native=NativeRates(row.rate_entry);
    Same(native,row.rate_expected);
    Same(NativeForces(row.force_entry),row.force_expected);
    warped+=!native.projection.planar;
  }
  EXPECT_EQ(warped,6u);
}
TEST(QephProjectionReplay,CurrentCppRawProjectionMatchesSameNativeWorkingOperands) {
  for(const auto& row:CapturedRows()) {
    SCOPED_TRACE(row.cycle);
    SCOPED_TRACE(row.original_row);
    Same(CppRates(row.rate_entry),NativeRates(row.rate_entry));
    Same(CppForces(row.force_entry),NativeForces(row.force_entry));
  }
}
TEST(QephProjectionReplay,ActualCallerCoordinatesAndSpinReproduceProjectionEntry) {
  for(const auto& row:CapturedRows()) {
    SCOPED_TRACE(row.cycle);
    SCOPED_TRACE(row.original_row);
    EXPECT_EQ(row.ismstr,2);
    EXPECT_EQ(row.irep,0);
    EXPECT_EQ(row.ixfem,0);
    const auto gathered=CallerRates(row,1.);
    Same(Fields(gathered),Fields(row.rate_entry));
    Same(CppRates(gathered),row.rate_expected);
  }
}
TEST(QephProjectionReplay,SamePhysicalWarpedStateChangesUnderUnadaptedWorkingLength) {
  double force_difference=0,couple_difference=0,omega_difference=0,velocity_difference=0;
  for(const auto& row:CapturedRows())for(double ratio:{.001,.01}) {
    SCOPED_TRACE(row.cycle);
    SCOPED_TRACE(row.original_row);
    SCOPED_TRACE(ratio);
    const auto rate=Scale(row.rate_entry,ratio);
    const auto projected=NativeRates(rate);
    Same(CppRates(rate),projected); // strict same-input test in each working unit
    const auto force=Scale(row.force_entry,ratio,projected.projection);
    const auto result=NativeForces(force);
    Same(CppForces(force),result);
    double df=0,dm=0,dw=0,dv=0;
    for(unsigned i=0;i<4;++i)for(unsigned j=0;j<3;++j){
      df=std::max(df,std::abs(result.force[i][j]-row.force_expected.force[i][j]));
      dm=std::max(dm,std::abs(result.couple[i][j]/ratio-row.force_expected.couple[i][j]));
    }
    for(unsigned i=0;i<4;++i)for(unsigned j=0;j<2;++j)
      dw=std::max(dw,std::abs(projected.rlxyz[i][j]-row.rate_expected.rlxyz[i][j]));
    for(unsigned j=0;j<3;++j)for(const auto pair:{std::pair<V3,V3>{projected.v13,row.rate_expected.v13},
        std::pair<V3,V3>{projected.v24,row.rate_expected.v24},std::pair<V3,V3>{projected.vhi,row.rate_expected.vhi}})
      dv=std::max(dv,std::abs(pair.first[j]/ratio-pair.second[j]));
    if(row.rate_expected.projection.planar){EXPECT_LT(df,1e-10);EXPECT_LT(dm,1e-10);EXPECT_LT(dw,1e-10);EXPECT_LT(dv,1e-8);}
    else {force_difference=std::max(force_difference,df);couple_difference=std::max(couple_difference,dm);
      omega_difference=std::max(omega_difference,dw);velocity_difference=std::max(velocity_difference,dv);}
  }
  EXPECT_GT(force_difference,1e-5);
  EXPECT_GT(omega_difference,1e-5);
  RecordProperty("maximum_physical_force_difference_n",Text(force_difference));
  RecordProperty("maximum_physical_couple_difference_n_mm",Text(couple_difference));
  RecordProperty("maximum_physical_omega_difference_per_s",Text(omega_difference));
  RecordProperty("maximum_physical_local_velocity_difference_mm_s",Text(velocity_difference));
}
TEST(QephProjectionReplay,OriginalRateAndForceProjectionRetainVirtualPowerAtEachWorkingLength) {
  for(const auto& row:CapturedRows())if(!row.rate_expected.projection.planar)for(double ratio:{1.,.001,.01}) {
    SCOPED_TRACE(row.cycle);
    SCOPED_TRACE(row.original_row);
    SCOPED_TRACE(ratio);
    auto input=Scale(row.rate_entry,ratio);
    input.v13={1.25*ratio,-2.5*ratio,.75*ratio};input.v24={-1.5*ratio,.25*ratio,2.75*ratio};input.vhi={.375*ratio,-.875*ratio,1.125*ratio};
    for(unsigned i=0;i<4;++i){const V3 local{.15*double(i+1),-.125*double(i+2),.07*double(i+3)};
      input.world_omega[i]=World(input.geometry,local);input.rlxyz[i]={local[0],local[1]};}
    const auto projected=NativeRates(input);
    auto force=Scale(row.force_entry,ratio,projected.projection);
    for(unsigned i=0;i<4;++i){force.vf[i]={.75*double(i+1),-1.25*double(i+2),.5*double(i+3)};
      force.vm[i]={.625*double(i+1)*ratio,-.875*double(i+2)*ratio};}
    const auto result=NativeForces(force);
    RateResult original;original.v13=input.v13;original.v24=input.v24;original.vhi=input.vhi;
    const auto initial_velocity=Nodes(original),projected_velocity=Nodes(projected);
    double first=0,second=0;
    for(unsigned i=0;i<4;++i) {
      const auto v=World(input.geometry,initial_velocity[i]);const auto sign=i<2?1.:-1.;const auto anti=i%2,symmetric=2+anti;
      for(unsigned j=0;j<3;++j){first+=result.force[i][j]*v[j]+result.couple[i][j]*input.world_omega[i][j];
        const double local=sign*force.vf[anti][j]+force.vf[symmetric][j];second+=local*projected_velocity[i][j];}
      for(unsigned j=0;j<2;++j){const double local=sign*force.vm[anti][j]+force.vm[symmetric][j];second+=local*projected.rlxyz[i][j];}
    }
    EXPECT_NEAR(first,second,512*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(first),std::abs(second)}));
  }
}
} // namespace qeph_projection_test
