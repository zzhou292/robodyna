#include "lib_src/collision/Q4PlanarStiffness.h"
#include "lib_utest/q4_planar_geometry_fixture.h"
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_planar_test;
using Matrix=Eigen::Matrix<long double,6,6>;
using PStatus=sc::PlanarContactStatus;

struct Prepared {
  fixture::Pair pair;
  sc::PlanarWallGeometry wall;
  sc::Q4PlanarGeometry geometry;
  void Initialize() {
    const auto mesh=fixture::Square();
    ASSERT_EQ(wall.Initialize(mesh.view()).status,PStatus::Ok);
    ASSERT_EQ(geometry.Initialize(wall,pair.surface(),pair.mass(),fixture::Clearance).status,PStatus::Ok);
  }
};

// Independent tensor Gauss quadrature, including physical coordinate Jacobian.
// No production moment matrix, corner restriction or bound arithmetic is used.
Matrix GaussGram(const fixture::Pair& pair,long double stiffness) {
  Matrix result=Matrix::Zero(); const long double a=1/std::sqrt(3.L);
  constexpr int su[4]={1,-1,-1,1},sv[4]={1,1,-1,-1};
  for (const auto& parent:pair.parents) for (long double u:{-a,a}) for (long double v:{-a,a}) {
    long double shape[4]{},dydu=0,dydv=0,dzdu=0,dzdv=0;
    for (unsigned i=0;i<4;++i) {
      shape[i]=(1+su[i]*u)*(1+sv[i]*v)/4;
      const long double du=su[i]*(1+sv[i]*v)/4,dv=sv[i]*(1+su[i]*u)/4;
      const auto x=pair.surface().positions.at(parent.nodes[i]);
      dydu+=du*x.y; dydv+=dv*x.y; dzdu+=du*x.z; dzdv+=dv*x.z;
    }
    const long double weight=stiffness*std::abs(dydu*dzdv-dydv*dzdu);
    for (unsigned i=0;i<4;++i) for (unsigned j=0;j<4;++j)
      result(parent.nodes[i],parent.nodes[j])+=weight*shape[i]*shape[j];
  }
  return result;
}

TEST(Q4PlanarStiffness, SharedGramEnclosesIndependentGaussAndBoundsFreeSpectrum) {
  Prepared f; f.Initialize(); ASSERT_FALSE(HasFatalFailure());
  sc::Q4PlanarStiffness value;
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(f.geometry.view(),f.pair.mass(),16,&value),PStatus::Ok);
  ASSERT_TRUE(value.valid); ASSERT_EQ(value.count,6U);
  const auto exact=GaussGram(f.pair,16); Matrix scaled=exact;
  for (unsigned i=0;i<6;++i) {
    EXPECT_EQ(value.nodes[i],i); EXPECT_EQ(value.inverse_mass[i],f.pair.inverse[i]);
    for (unsigned j=0;j<6;++j) {
      EXPECT_LE(static_cast<long double>(value.entry[i][j].lower),exact(i,j));
      EXPECT_GE(static_cast<long double>(value.entry[i][j].upper),exact(i,j));
      EXPECT_LE(value.entry[i][j].upper-value.entry[i][j].lower,1e-12);
      scaled(i,j)*=std::sqrt(static_cast<long double>(f.pair.inverse[i])*f.pair.inverse[j]);
    }
  }
  EXPECT_GT(value.entry[0][0].lower,0);  // Fixed-node reaction block is retained.
  EXPECT_EQ(value.inverse_mass[0],0);
  Eigen::SelfAdjointEigenSolver<Matrix> spectrum(scaled);
  ASSERT_EQ(spectrum.info(),Eigen::Success);
  EXPECT_GE(static_cast<long double>(value.rate_bound),spectrum.eigenvalues().maxCoeff());
  EXPECT_LE(value.rate_bound,1.5L*spectrum.eigenvalues().maxCoeff());
}

TEST(Q4PlanarStiffness, StiffnessMassAndParentOrderHaveDeclaredScaling) {
  Prepared f; f.Initialize(); ASSERT_FALSE(HasFatalFailure());
  sc::Q4PlanarStiffness base,scaled,permuted;
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(f.geometry.view(),f.pair.mass(),16,&base),PStatus::Ok);
  for (auto& inverse:f.pair.inverse) inverse*=.25;
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(f.geometry.view(),f.pair.mass(),64,&scaled),PStatus::Ok);
  EXPECT_NEAR(scaled.rate_bound,base.rate_bound,1e-12);
  for (unsigned i=0;i<6;++i) for (unsigned j=0;j<6;++j) {
    EXPECT_EQ(scaled.entry[i][j].lower,4*base.entry[i][j].lower);
    EXPECT_EQ(scaled.entry[i][j].upper,4*base.entry[i][j].upper);
  }
  std::swap(f.pair.parents[0],f.pair.parents[1]); sc::Q4PlanarGeometry geometry;
  ASSERT_EQ(geometry.Initialize(f.wall,f.pair.surface(),f.pair.mass(),fixture::Clearance).status,PStatus::Ok);
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(geometry.view(),f.pair.mass(),64,&permuted),PStatus::Ok);
  EXPECT_EQ(permuted.rate_bound,scaled.rate_bound);
  for (unsigned i=0;i<6;++i) for (unsigned j=0;j<6;++j) {
    EXPECT_EQ(permuted.entry[i][j].lower,scaled.entry[i][j].lower);
    EXPECT_EQ(permuted.entry[i][j].upper,scaled.entry[i][j].upper);
  }
}

TEST(Q4PlanarStiffness, SignedIncrementBoundContainsIndependentPotentialAndSubtractionRoundoff) {
  Prepared f; f.Initialize(); ASSERT_FALSE(HasFatalFailure());
  sc::Q4PlanarStiffness value;
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(f.geometry.view(),f.pair.mass(),16,&value),PStatus::Ok);
  const auto gram=GaussGram(f.pair,16);
  for (unsigned pattern=0;pattern<3;++pattern) {
    sc::Q4IntegralInterval delta[sc::MaxQ4PlanarNodes]{};
    Eigen::Matrix<long double,6,1> exact;
    for (unsigned i=0;i<6;++i) {
      const double old=.1*(i+1),next=(pattern==0 ? .3 : pattern==1 ? -.25 : old);
      ASSERT_TRUE(sc::q4_bounds::Difference(next,old,&delta[i]));
      exact(i)=static_cast<long double>(next)-static_cast<long double>(old);
    }
    double upper=-1;
    ASSERT_EQ(sc::BoundQ4PlanarQuadratic(value,delta,&upper),PStatus::Ok);
    const long double expected=.5L*(exact.transpose()*gram*exact)(0,0);
    EXPECT_GE(static_cast<long double>(upper),expected);
    EXPECT_LE(static_cast<long double>(upper)-expected,1e-11L);
    if (pattern==2) { EXPECT_EQ(upper,0); }
  }
}

TEST(Q4PlanarStiffness, OutsideGeometryHasZeroRateAndNoInventedQuadraticWork) {
  Prepared f; for (unsigned n=0;n<6;++n) f.pair.x[n+6]+=10;
  f.Initialize(); ASSERT_FALSE(HasFatalFailure()); sc::Q4PlanarStiffness value;
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(f.geometry.view(),f.pair.mass(),16,&value),PStatus::Ok);
  EXPECT_TRUE(value.valid); EXPECT_EQ(value.rate_bound,0); EXPECT_EQ(value.count,6U);
  sc::Q4IntegralInterval delta[sc::MaxQ4PlanarNodes];
  for (auto& d:delta) d={1e300,1e300};
  double upper=-1;
  ASSERT_EQ(sc::BoundQ4PlanarQuadratic(value,delta,&upper),PStatus::Ok);
  EXPECT_EQ(upper,0);
}

TEST(Q4PlanarStiffness, InvalidMassOverflowAndLateArithmeticPreserveWholeOutput) {
  Prepared f; f.Initialize(); ASSERT_FALSE(HasFatalFailure()); sc::Q4PlanarStiffness before;
  ASSERT_EQ(sc::BuildQ4PlanarStiffness(f.geometry.view(),f.pair.mass(),16,&before),PStatus::Ok);
  for (unsigned failure=0;failure<3;++failure) {
    auto pair=f.pair; auto output=before;
    if (failure==0) pair.inverse[5]=0;
    if (failure==1) pair.fixed[5]=0;
    // The center mobility remains finite, but K55 * inverse_mass cannot fit.
    if (failure==2) pair.inverse[5]=std::numeric_limits<double>::max();
    EXPECT_NE(sc::BuildQ4PlanarStiffness(f.geometry.view(),pair.mass(),16,&output),PStatus::Ok);
    EXPECT_EQ(std::memcmp(&output,&before,sizeof(before)),0);
  }
  sc::Q4IntegralInterval delta[sc::MaxQ4PlanarNodes]{}; double upper=123;
  delta[5]={1e300,1e300};
  EXPECT_EQ(sc::BoundQ4PlanarQuadratic(before,delta,&upper),PStatus::InvalidOutput);
  EXPECT_EQ(upper,123);
  delta[5]={std::numeric_limits<double>::quiet_NaN(),1};
  EXPECT_EQ(sc::BoundQ4PlanarQuadratic(before,delta,&upper),PStatus::InvalidInput);
  EXPECT_EQ(upper,123);
  for (auto& d:delta) d={.125,.125};
  EXPECT_EQ(sc::BoundQ4PlanarQuadratic(before,delta,&upper),PStatus::Ok); EXPECT_GT(upper,0);
}
}  // namespace
