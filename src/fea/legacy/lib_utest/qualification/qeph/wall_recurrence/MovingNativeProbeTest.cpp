#include "MovingNativeProbe.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
namespace r=recurrence;
unsigned Index(const r::Model& m,r::Group group,unsigned entity,unsigned component) {
  for(unsigned i=0;i<m.dictionary.size();++i) {
    const auto& c=m.dictionary[i];
    if(c.group==group&&c.entity==entity&&c.component==component) return i;
  }
  ADD_FAILURE()<<"Missing complete dictionary coordinate"; return 0;
}
void Exact(const Eigen::VectorXd& a,const Eigen::VectorXd& b) {
  ASSERT_EQ(a.size(),b.size());
  EXPECT_EQ(std::memcmp(a.data(),b.data(),sizeof(double)*a.size()),0);
}
void Near(double a,double b) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a-b),2e-12*std::max(1.,std::abs(b)));
}
bool Broadside(unsigned cells,r::Model& out,std::string& error) {
  r::Model m;
  if(!r::BuildModel(cells,m,error)) return false;
  // Exact cyclic permutation of BQ3 axes. Native reference is at -gap,
  // disposable current geometry at touching; no source coordinates are repaired.
  constexpr double gap=.000375/4;
  for(unsigned n=0;n<m.nodes;++n)
    m.position[n]={0,r::Length*(static_cast<double>(n/2)-.5*cells),r::Length*(n%2?.5:-.5)};
  m.mass.fill(0); m.inertia.fill(0);
  for(unsigned e=0;e<cells;++e) {
    auto in=m.reference[e].data().input;
    for(unsigned i=0;i<4;++i) {
      in.position[i]=m.position[m.connectivity[e][i]]; in.position[i].x=-gap;
    }
    if(Initialize(in,m.reference[e])!=Status::kSuccess) { error="Broadside reference startup failed"; return false; }
    for(unsigned i=0;i<4;++i) {
      const auto n=m.connectivity[e][i];
      m.mass[n]+=m.reference[e].data().nodal_mass[i];
      m.inertia[n]+=m.reference[e].data().isotropic_inertia[i];
    }
  }
  out=std::move(m); return true;
}
}
TEST(QephWallRecurrenceSupport, ZeroBoostPreservesLegacyMapEveryOutputBitAndAlias) {
  for(unsigned cells:{1u,2u}) {
    r::Model m; std::string error; ASSERT_TRUE(r::BuildModel(cells,m,error))<<error;
    Eigen::VectorXd input=Eigen::VectorXd::Zero(m.dictionary.size());
    for(unsigned pattern=0;pattern<3;++pattern) {
      SCOPED_TRACE(pattern);
      if(pattern==1) {
        input[Index(m,r::Group::ForceCache,0,2)]=r::Amplitudes.back();
        input[Index(m,r::Group::History,cells-1,5)]=r::Amplitudes.back();
      }
      if(pattern==2) {
        input[Index(m,r::Group::OrientationTangent,m.nodes-1,1)]=r::Amplitudes.back();
        input[Index(m,r::Group::Spin,m.nodes-1,1)]=r::Amplitudes.back();
      }
      const Eigen::VectorXd held=input;
      Eigen::VectorXd old_result,new_result;
      ASSERT_TRUE(r::NativeMap(m,r::H0,input,old_result,error))<<error;
      ASSERT_TRUE(r::NativeMapWithUniformVelocity(m,r::H0,{},input,new_result,error))<<error;
      Exact(new_result,old_result); Exact(input,held);
      auto aliased=input;
      ASSERT_TRUE(r::NativeMapWithUniformVelocity(m,r::H0,{},aliased,aliased,error))<<error;
      Exact(aliased,old_result);
    }
  }
}
TEST(QephWallRecurrenceSupport, BroadsideMovingBaselineRetainsDriftAndEveryNativeHistoryField) {
  for(unsigned cells:{1u,2u}) {
    r::Model m; std::string error; ASSERT_TRUE(Broadside(cells,m,error))<<error;
    const Eigen::VectorXd zero=Eigen::VectorXd::Zero(m.dictionary.size());
    for(double h:r::Steps) for(double speed:{-8.,0.,8.}) {
      SCOPED_TRACE(cells);
      SCOPED_TRACE(h);
      SCOPED_TRACE(speed);
      Eigen::VectorXd output;
      ASSERT_TRUE(r::NativeMapWithUniformVelocity(m,h,{speed,0,0},zero,output,error))<<error;
      for(unsigned i=0;i<m.dictionary.size();++i) {
        const auto& c=m.dictionary[i]; SCOPED_TRACE(c.name);
        double expected=0;
        if(c.group==r::Group::Position&&c.component==0) expected=h*speed/c.scale;
        if(c.group==r::Group::Velocity&&c.component==0) expected=speed/c.scale;
        Near(output[i],expected);
      }
      // The actual baseline is intentionally nonzero, not hidden by the map.
      if(speed!=0) EXPECT_NE(output[Index(m,r::Group::Position,0,0)],0);
    }
  }
}
TEST(QephWallRecurrenceSupport, FullMovingProbeRetainsNativeCacheFeedbackAndLegacyCentering) {
  r::Model m; std::string error; ASSERT_TRUE(r::BuildModel(1,m,error))<<error;
  const auto old=r::Differentiate(m,r::H0,r::Amplitudes.back());
  const auto zero=DifferentiateMoving(m,r::H0,r::Amplitudes.back(),{});
  ASSERT_TRUE(old.complete)<<old.diagnostic;
  ASSERT_TRUE(zero.derivative.complete)<<zero.derivative.diagnostic;
  ASSERT_TRUE(zero.baseline_complete);
  EXPECT_EQ(zero.baseline.cwiseAbs().maxCoeff(),0);
  EXPECT_EQ(zero.derivative.full,old.full);
  const auto moving=DifferentiateMoving(m,r::H0,r::Amplitudes.back(),{8,0,0});
  ASSERT_TRUE(moving.baseline_complete);
  ASSERT_TRUE(moving.derivative.complete)<<moving.derivative.diagnostic;
  EXPECT_EQ(moving.derivative.completed_columns,m.dictionary.size());
  EXPECT_EQ(moving.derivative.full.rows(),m.dictionary.size());
  const auto cache=Index(m,r::Group::ForceCache,0,0);
  const auto velocity=Index(m,r::Group::Velocity,0,0),position=Index(m,r::Group::Position,0,0);
  const double kick=-r::H0*m.dictionary[cache].scale/(m.mass[0]*m.dictionary[velocity].scale);
  Near(moving.derivative.full(velocity,cache),kick);
  const double drift=kick*r::H0*m.dictionary[velocity].scale/m.dictionary[position].scale;
  Near(moving.derivative.full(position,cache),drift);
  EXPECT_GT(std::abs(kick),32*r::MatrixTolerance); // Omitted/wrong cache is distinguishable.
  EXPECT_LE((moving.derivative.full-old.full).cwiseAbs().maxCoeff(),
            r::MatrixTolerance*std::max(1.,old.full.cwiseAbs().maxCoeff()));
}
TEST(QephWallRecurrenceSupport, FailedBoostAndLateSecondHistoryPreserveOutputAndCleanRetry) {
  r::Model m; std::string error; ASSERT_TRUE(r::BuildModel(2,m,error))<<error;
  Eigen::VectorXd input=Eigen::VectorXd::Zero(m.dictionary.size());
  Eigen::VectorXd output=Eigen::VectorXd::Constant(7,19),held=output;
  const double nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(r::NativeMapWithUniformVelocity(m,r::H0,{0,nan,0},input,output,error));
  Exact(output,held);
  // Cell zero reaches the native operation; cell one's negative thickness
  // rejects its history. No partial cell-zero result may replace the output.
  input[Index(m,r::Group::History,1,33)]=-2;
  const Eigen::VectorXd original_input=input;
  EXPECT_FALSE(r::NativeMapWithUniformVelocity(m,r::H0,{8,0,0},input,output,error));
  EXPECT_EQ(error,"Native prescribed history rejected");
  Exact(output,held); Exact(input,original_input);
  auto alias=input;
  EXPECT_FALSE(r::NativeMapWithUniformVelocity(m,r::H0,{8,0,0},alias,alias,error));
  Exact(alias,original_input);
  input.setZero(); Eigen::VectorXd clean;
  ASSERT_TRUE(r::NativeMapWithUniformVelocity(m,r::H0,{8,0,0},input,clean,error))<<error;
  ASSERT_TRUE(r::NativeMapWithUniformVelocity(m,r::H0,{8,0,0},input,output,error))<<error;
  Exact(output,clean); EXPECT_TRUE(error.empty());
  const auto incomplete=DifferentiateMoving(m,r::H0,0,{8,0,0});
  EXPECT_FALSE(incomplete.baseline_complete); EXPECT_FALSE(incomplete.derivative.complete);
  EXPECT_EQ(incomplete.derivative.completed_columns,0u);
  EXPECT_TRUE(incomplete.derivative.full.allFinite());
}
} // namespace tl::qualification::qeph::wall_recurrence
