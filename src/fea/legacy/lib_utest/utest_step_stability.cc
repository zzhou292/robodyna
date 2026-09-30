#include "lib_src/collision/SurfaceContactLaw.h"
#include "lib_src/solvers/ExplicitStepStability.h"
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <initializer_list>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace st=tl::fea::stability;
using Status=sc::Status;
struct Mass {
  double inverse[6]={1,.5,.25,.125,2,4};
  std::uint8_t fixed[6]{};
  sc::LumpedTranslationMassView view(std::uint32_t count=6) const {
    return {inverse,fixed,count,7,sc::TranslationMassModel::kIsotropicLumped};
  }
};
sc::SurfaceTriangle Triangle(unsigned a,unsigned b,unsigned c) {
  return {{a,b,c},1,1,0,0,sc::SurfaceInterpolation::kLinearTriangle};
}
sc::NormalJacobian Jacobian(const Mass& mass,std::initializer_list<sc::SignedNodeWeight> weights,
                           sc::Vec3 normal={1,0,0},std::uint64_t attempt=1) {
  sc::NormalJacobian out;
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights.begin(),weights.size(),normal,attempt,&out),Status::kOk);
  return out;
}
st::RowContribution Contribution(const sc::NormalJacobian& j,double k,double c) {
  st::RowContribution out;EXPECT_EQ(st::MakeRankOneContribution(j,k,c,&out),Status::kOk);return out;
}
double SpectralRadius(double k,double c,double h) {
  const double trace=2-h*c-h*h*k,det=1-h*c;
  const auto root=std::sqrt(std::complex<double>(trace*trace-4*det));
  return std::max(std::abs(.5*(trace+root)),std::abs(.5*(trace-root)));
}
void Near(sc::Vec3 a,sc::Vec3 b) {
  EXPECT_NEAR(a.x,b.x,2e-13);EXPECT_NEAR(a.y,b.y,2e-13);EXPECT_NEAR(a.z,b.z,2e-13);
}

TEST(SharedContactMass, SignedSharedJacobianPredictsIndependentNodalImpulse) {
  Mass mass;const auto a=Triangle(0,1,2),b=Triangle(1,2,3);
  const double wa[3]={.25,.5,.25},wb[3]={.25,.5,.25};const sc::Vec3 n{.6,0,.8};
  sc::NormalJacobian j;
  ASSERT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),a,wa,&b,wb,n,1,&j),Status::kOk);
  ASSERT_EQ(j.count,4u);EXPECT_NEAR(j.inverse_effective_mass,.1171875,1e-15);
  const double signed_weights[4]={.25,.25,-.25,-.25};
  for(unsigned i=0;i<4;++i) { EXPECT_EQ(j.nodes[i],i);Near(j.values[i],sc::Scale(n,signed_weights[i])); }
  // Scatter the two original endpoint impulses independently, sharing the SAME
  // physical velocity array. No normalized norms or merged output enter this oracle.
  const double impulse=.75;sc::Vec3 dv[6]{},force[6]{};
  for(int i=0;i<3;++i) {
    force[a.nodes[i]]=sc::Add(force[a.nodes[i]],sc::Scale(n,impulse*wa[i]));
    force[b.nodes[i]]=sc::Add(force[b.nodes[i]],sc::Scale(n,-impulse*wb[i]));
  }
  for(int i=0;i<6;++i) dv[i]=sc::Scale(force[i],mass.inverse[i]);
  double relative_change=0,naive=0;
  for(int i=0;i<3;++i) {
    relative_change+=wa[i]*sc::Dot(n,dv[a.nodes[i]])-wb[i]*sc::Dot(n,dv[b.nodes[i]]);
    naive+=wa[i]*wa[i]*mass.inverse[a.nodes[i]]+wb[i]*wb[i]*mass.inverse[b.nodes[i]];
  }
  EXPECT_NEAR(relative_change,impulse*j.inverse_effective_mass,2e-15);
  EXPECT_GT(naive,2*j.inverse_effective_mass);
  sc::NormalJacobian reversed;
  ASSERT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),b,wb,&a,wa,sc::Scale(n,-1),1,&reversed),Status::kOk);
  EXPECT_DOUBLE_EQ(j.inverse_effective_mass,reversed.inverse_effective_mass);
  for(unsigned i=0;i<j.count;++i) Near(j.values[i],reversed.values[i]);
}

TEST(SharedContactMass, ExactCancellationAndExplicitFixedWallHaveDistinctSemantics) {
  Mass mass;const auto triangle=Triangle(0,1,2);const double w[3]={.25,.5,.25};
  sc::NormalJacobian j;
  EXPECT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,&triangle,w,{1,0,0},1,&j),Status::kNoDynamicDofs);
  EXPECT_FALSE(j.valid);EXPECT_DOUBLE_EQ(j.inverse_effective_mass,0);
  ASSERT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kOk);
  EXPECT_NEAR(j.inverse_effective_mass,.203125,1e-15);
  mass.fixed[1]=1;mass.inverse[1]=0;
  ASSERT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kOk);
  EXPECT_NEAR(j.inverse_effective_mass,.078125,1e-15);EXPECT_DOUBLE_EQ(j.normalized_norm[1],0);
  mass.fixed[0]=mass.fixed[2]=1;mass.inverse[0]=mass.inverse[2]=0;
  EXPECT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kNoDynamicDofs);
}

TEST(SharedContactMass, RejectsUnknownMassOffsetsAndMalformedStencil) {
  Mass mass;sc::NormalJacobian j;const sc::SignedNodeWeight weights[7]={{0,1},{1,-1}};
  auto view=mass.view();view.model=sc::TranslationMassModel::kGeneralizedOrRotational;
  EXPECT_EQ(sc::BuildNormalJacobian(view,weights,2,{1,0,0},1,&j),Status::kUnsupportedInterpolation);
  view=mass.view();view.fixed=nullptr;EXPECT_EQ(sc::BuildNormalJacobian(view,weights,2,{1,0,0},1,&j),Status::kInvalidArgument);
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,7,{1,0,0},1,&j),Status::kOutOfRange);
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,2,{2,0,0},1,&j),Status::kInvalidArgument);
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,2,{0,0,0},1,&j),Status::kInvalidArgument);
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,2,{1,0,0},0,&j),Status::kInvalidArgument);
  for(double inverse:{0.,-1.,std::numeric_limits<double>::infinity()}) {
    mass.inverse[0]=inverse;
    EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,2,{1,0,0},1,&j),Status::kInvalidArgument);
    EXPECT_FALSE(j.valid);EXPECT_EQ(j.count,0u);
  }
  mass=Mass{};mass.fixed[0]=2;
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,2,{1,0,0},1,&j),Status::kInvalidArgument);
  mass.fixed[0]=1;
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),weights,2,{1,0,0},1,&j),Status::kInvalidArgument);
  mass=Mass{};auto triangle=Triangle(0,1,2);double w[3]={.25,.5,.25};
  triangle.half_thickness=.001;
  EXPECT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kUnsupportedInterpolation);
  triangle.half_thickness=0;triangle.interpolation=sc::SurfaceInterpolation::kUnspecified;
  EXPECT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kUnsupportedInterpolation);
  triangle=Triangle(0,1,2);w[0]=.5;
  EXPECT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kInvalidArgument);
  w[0]=.25;triangle.nodes[2]=8;
  EXPECT_EQ(sc::BuildLinearTriangleNormalJacobian(mass.view(),triangle,w,nullptr,nullptr,{1,0,0},1,&j),Status::kOutOfRange);
}

TEST(SharedContactMass, OverflowAndLostPositiveMassTermsCannotPublish) {
  Mass mass;sc::NormalJacobian j;
  const sc::SignedNodeWeight huge[2]={{0,1e308},{0,1e308}},tiny[1]={{0,1e-200}};
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),huge,2,{1,0,0},1,&j),Status::kNonFiniteResult);
  EXPECT_FALSE(j.valid);EXPECT_EQ(j.count,0u);
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),tiny,1,{1,0,0},1,&j),Status::kNonFiniteResult);
  EXPECT_FALSE(j.valid);
  const sc::SignedNodeWeight square_overflow[1]={{0,1e200}};
  EXPECT_EQ(sc::BuildNormalJacobian(mass.view(),square_overflow,1,{1,0,0},1,&j),Status::kNonFiniteResult);
}

TEST(ExplicitStepStability, CoupledStructuralAndContactStiffnessDefeatsIsolatedMinima) {
  Mass mass;auto j=Jacobian(mass,{{0,1}});
  const auto structural=Contribution(j,100,0),contact=Contribution(j,100,0);
  double k[1],c[1];st::RowBounds rows{k,c,1,1};
  ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);
  ASSERT_EQ(st::AccumulateRows(&rows,structural),Status::kOk);
  ASSERT_EQ(st::AccumulateRows(&rows,contact),Status::kOk);
  st::StepLimit limit;ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&limit),Status::kOk);
  EXPECT_NEAR(limit.stiffness_bound,200,2e-11);EXPECT_NEAR(limit.dt,.8*std::sqrt(4./200),2e-14);
  EXPECT_TRUE(st::IsCurrentLimit(rows,limit));
  EXPECT_LE(SpectralRadius(100,0,.18),1+1e-14);
  EXPECT_GT(SpectralRadius(200,0,.18),1.5);
  EXPECT_LE(SpectralRadius(200,0,limit.dt),1+1e-14);
  sc::NormalContactResponse local;
  ASSERT_EQ(sc::EvaluateNormalContact({100,0,.8},{-.01,0,1},&local),Status::kOk);
  EXPECT_GT(local.stable_timestep,limit.dt);
}

TEST(ExplicitStepStability, InvalidOrOverflowingLocalContributionIsNeverUsable) {
  Mass mass;auto j=Jacobian(mass,{{0,1},{1,-1}});st::RowContribution contribution;
  EXPECT_EQ(st::MakeRankOneContribution(j,-1,0,&contribution),Status::kInvalidArgument);
  EXPECT_FALSE(contribution.valid);EXPECT_EQ(contribution.count,0u);
  EXPECT_EQ(st::MakeRankOneContribution(j,1,std::numeric_limits<double>::quiet_NaN(),&contribution),Status::kInvalidArgument);
  EXPECT_EQ(st::MakeRankOneContribution(j,std::numeric_limits<double>::max(),0,&contribution),Status::kNonFiniteResult);
  EXPECT_FALSE(contribution.valid);EXPECT_EQ(contribution.count,0u);
  auto duplicate=j;duplicate.nodes[1]=duplicate.nodes[0];
  EXPECT_EQ(st::MakeRankOneContribution(duplicate,100,0,&contribution),Status::kInvalidArgument);
  j.normalized_norm[0]=-1;
  EXPECT_EQ(st::MakeRankOneContribution(j,100,0,&contribution),Status::kInvalidArgument);
  j.valid=false;EXPECT_EQ(st::MakeRankOneContribution(j,100,0,&contribution),Status::kNoTrial);
}

TEST(ExplicitStepStability, BlockRowsBoundNoncommutingPsdMatricesAndAmplification) {
  Mass mass;double kr[3],cr[3];st::RowBounds rows{kr,cr,3,3};
  ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);
  Eigen::Matrix<double,9,9> k=Eigen::Matrix<double,9,9>::Zero(),c=k;
  Eigen::Matrix<double,9,3> active;
  const unsigned endpoints[3][2]={{0,1},{1,2},{2,2}};
  const sc::Vec3 normals[3]={{1,0,0},{.6,.8,0},{0,0,1}};
  const double spring[3]={100,250,30},damper[3]={0,3,8};
  for(int e=0;e<3;++e) {
    const auto j=e==2?Jacobian(mass,{{2,1}},normals[e]):
      Jacobian(mass,{{endpoints[e][0],1},{endpoints[e][1],-1}},normals[e]);
    ASSERT_EQ(st::AccumulateRows(&rows,Contribution(j,spring[e],damper[e])),Status::kOk);
    // Independent dense M^-1/2 J construction from physical input, not returned
    // norms/contribution rows. Rank-one matrices may have different normals.
    Eigen::Matrix<double,9,1> a=Eigen::Matrix<double,9,1>::Zero();
    for(int side=0;side<(e==2?1:2);++side) {
      const auto node=endpoints[e][side];const double f=(side?-1:1)*std::sqrt(mass.inverse[node]);
      a(3*node)=f*normals[e].x;a(3*node+1)=f*normals[e].y;a(3*node+2)=f*normals[e].z;
    }
    k+=spring[e]*a*a.transpose();c+=damper[e]*a*a.transpose();
    active.col(e)=a;
  }
  ASSERT_GT((k*c-c*k).norm(),1);
  st::StepLimit limit;ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&limit),Status::kOk);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9>> ke(k),ce(c);
  EXPECT_GE(limit.stiffness_bound,ke.eigenvalues().maxCoeff());
  EXPECT_GE(limit.damping_bound,ce.eigenvalues().maxCoeff());
  // Restrict to the independently assembled span of the three physical modes.
  // The common null space has legitimate free-translation Jordan blocks; a
  // dense eigensolver there would test conditioning rather than stability.
  for(int i=0;i<3;++i) {
    for(int j=0;j<i;++j) active.col(i)-=active.col(j)*active.col(j).dot(active.col(i));
    ASSERT_GT(active.col(i).norm(),.1);active.col(i).normalize();
  }
  const Eigen::Matrix3d ka=active.transpose()*k*active,ca=active.transpose()*c*active;
  const double h=limit.dt;Eigen::Matrix<double,6,6> amplification;
  const auto identity=Eigen::Matrix3d::Identity();
  amplification.block<3,3>(0,0)=identity-h*h*ka;
  amplification.block<3,3>(0,3)=h*(identity-h*ca);
  amplification.block<3,3>(3,0)=-h*ka;
  amplification.block<3,3>(3,3)=identity-h*ca;
  Eigen::EigenSolver<Eigen::Matrix<double,6,6>> eigen(amplification,false);
  ASSERT_EQ(eigen.info(),Eigen::Success);
  EXPECT_LE(eigen.eigenvalues().cwiseAbs().maxCoeff(),1+2e-10);
  RecordProperty("scope","fixed-step frozen PSD linear modes; free rigid displacement drift allowed");
}

TEST(ExplicitStepStability, ContributionOrderRotationAndCleanRetryPreserveBounds) {
  Mass mass;const auto j=Jacobian(mass,{{0,1},{1,-1}},{.6,0,.8});
  const auto rotated=Jacobian(mass,{{0,1},{1,-1}},{0,.6,.8});
  auto first=Contribution(j,100,3),second=Contribution(j,20,2),rotation=Contribution(rotated,100,3);
  // Mathematical block norms are rotation invariant; elementary outward
  // rounding can differ by a few ulps when Cartesian component order changes.
  for(unsigned i=0;i<first.count;++i) { EXPECT_NEAR(first.stiffness[i],rotation.stiffness[i],2e-12);EXPECT_NEAR(first.damping[i],rotation.damping[i],2e-13); }
  double k[2],c[2],rk[2],rc[2];st::RowBounds rows{k,c,2,2},reverse{rk,rc,2,2};
  ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);ASSERT_EQ(st::ResetRows(&reverse,7,1),Status::kOk);
  ASSERT_EQ(st::AccumulateRows(&rows,first),Status::kOk);ASSERT_EQ(st::AccumulateRows(&rows,second),Status::kOk);
  ASSERT_EQ(st::AccumulateRows(&reverse,second),Status::kOk);ASSERT_EQ(st::AccumulateRows(&reverse,first),Status::kOk);
  const std::array<double,2> expected_k{k[0],k[1]},expected_c{c[0],c[1]};
  for(int i=0;i<2;++i) { EXPECT_NEAR(k[i],rk[i],1e-12);EXPECT_NEAR(c[i],rc[i],1e-13); }
  st::InvalidateRows(&rows);ASSERT_EQ(st::ResetRows(&rows,7,2),Status::kOk);
  first.attempt=second.attempt=2;
  ASSERT_EQ(st::AccumulateRows(&rows,first),Status::kOk);ASSERT_EQ(st::AccumulateRows(&rows,second),Status::kOk);
  for(int i=0;i<2;++i) { EXPECT_DOUBLE_EQ(k[i],expected_k[i]);EXPECT_DOUBLE_EQ(c[i],expected_c[i]); }
}

TEST(ExplicitStepStability, OverflowCannotPartiallyAccumulateOrYieldUsableLimit) {
  double k[2],c[2];st::RowBounds rows{k,c,2,2};ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);
  k[0]=1;k[1]=std::numeric_limits<double>::max()/2;
  st::RowContribution contribution;contribution.valid=true;contribution.count=2;
  contribution.nodes[0]=0;contribution.nodes[1]=1;contribution.base_epoch=7;contribution.attempt=1;
  contribution.stiffness[0]=2;contribution.stiffness[1]=k[1];
  EXPECT_EQ(st::AccumulateRows(&rows,contribution),Status::kNonFiniteResult);
  EXPECT_DOUBLE_EQ(k[0],1);EXPECT_DOUBLE_EQ(k[1],std::numeric_limits<double>::max()/2);
  EXPECT_DOUBLE_EQ(c[0],0);EXPECT_DOUBLE_EQ(c[1],0);EXPECT_FALSE(rows.valid);
  st::StepLimit limit;limit.dt=1;
  EXPECT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&limit),Status::kNoTrial);EXPECT_DOUBLE_EQ(limit.dt,0);
  ASSERT_EQ(st::ResetRows(&rows,7,2),Status::kOk);
  EXPECT_DOUBLE_EQ(k[0],0);EXPECT_DOUBLE_EQ(k[1],0);
}

TEST(ExplicitStepStability, CapacityStaleAndSealedAttemptsInvalidateCachedLimits) {
  Mass mass;auto contribution=Contribution(Jacobian(mass,{{0,1}}),100,0);
  double k[2]={17,19},c[2]={3,5};st::RowBounds rows{k,c,2,1};
  EXPECT_EQ(st::ResetRows(&rows,7,1),Status::kOutOfRange);EXPECT_DOUBLE_EQ(k[0],17);
  rows.capacity=2;ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);
  ASSERT_EQ(st::AccumulateRows(&rows,contribution),Status::kOk);
  st::StepLimit old;ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&old),Status::kOk);
  ASSERT_TRUE(st::IsCurrentLimit(rows,old));
  EXPECT_EQ(st::AccumulateRows(&rows,contribution),Status::kNoTrial);
  EXPECT_FALSE(st::IsCurrentLimit(rows,old));EXPECT_GT(old.dt,0); // Old POD cannot revoke itself.
  ASSERT_EQ(st::ResetRows(&rows,7,2),Status::kOk);EXPECT_FALSE(st::IsCurrentLimit(rows,old));
  EXPECT_EQ(st::AccumulateRows(&rows,contribution),Status::kStaleTrial);EXPECT_FALSE(rows.valid);
  ASSERT_EQ(st::ResetRows(&rows,8,1),Status::kOk);
  contribution.base_epoch=8;contribution.attempt=1;contribution.nodes[0]=2;
  EXPECT_EQ(st::AccumulateRows(&rows,contribution),Status::kOutOfRange);EXPECT_DOUBLE_EQ(k[0],0);
  EXPECT_EQ(st::ResetRows(&rows,7,99),Status::kStaleTrial);
}

TEST(ExplicitStepStability, MinimumStepAndInvalidParametersNeverClampUpward) {
  double k[1],c[1];st::RowBounds rows{k,c,1,1};st::StepLimit limit;
  ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);k[0]=1e12;
  EXPECT_EQ(st::FinalizeRows(&rows,.8,1e-5,1,&limit),Status::kOutOfRange);EXPECT_DOUBLE_EQ(limit.dt,0);EXPECT_FALSE(rows.valid);
  std::uint64_t attempt=2;
  for(double safety:{0.,1.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
    ASSERT_EQ(st::ResetRows(&rows,7,attempt++),Status::kOk);
    EXPECT_EQ(st::FinalizeRows(&rows,safety,1e-8,1,&limit),Status::kInvalidArgument);EXPECT_DOUBLE_EQ(limit.dt,0);
  }
  ASSERT_EQ(st::ResetRows(&rows,7,attempt),Status::kOk);c[0]=-1;
  EXPECT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&limit),Status::kInvalidArgument);EXPECT_FALSE(rows.valid);
}

TEST(ExplicitStepStability, ZeroAndPureDampingBoundsHaveExplicitLimits) {
  double k[1],c[1];st::RowBounds rows{k,c,1,1};st::StepLimit limit;
  ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);
  ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,.25,&limit),Status::kOk);
  EXPECT_FALSE(limit.has_stiffness_or_damping);EXPECT_DOUBLE_EQ(limit.dt,.25);
  ASSERT_EQ(st::ResetRows(&rows,7,2),Status::kOk);c[0]=3;
  ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&limit),Status::kOk);
  EXPECT_NEAR(limit.dt,1.6/3,2e-14);EXPECT_TRUE(limit.has_stiffness_or_damping);
  EXPECT_LE(SpectralRadius(0,3,limit.dt),1+1e-14);
  ASSERT_EQ(st::ResetRows(&rows,7,3),Status::kOk);c[0]=std::numeric_limits<double>::denorm_min();
  ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,.25,&limit),Status::kOk);EXPECT_DOUBLE_EQ(limit.dt,.25);
}

TEST(ExplicitStepStability, UpwardNormAndNearUnitSafetyRespectLongDoubleOracle) {
  const sc::Vec3 vectors[]={{.6,0,.8},{1e150,-2e150,3e150},{1e-150,2e-150,-3e-150}};
  for(const auto v:vectors) {
    double upper=0;ASSERT_TRUE(sc::mass_detail::UpperNorm(v,&upper));
    const long double x=v.x,y=v.y,z=v.z,exact=std::sqrt(x*x+y*y+z*z);
    EXPECT_GE(static_cast<long double>(upper),exact);
    EXPECT_LE(static_cast<long double>(upper),exact*(1+1e-13L));
  }
  double k[1],c[1];st::RowBounds rows{k,c,1,1};std::uint64_t attempt=1;
  const double safety=std::nextafter(1.,0.);
  for(double alpha:{0.,1e-250,1.,1e250}) for(double beta:{0.,1e-120,1.,1e120}) {
    ASSERT_EQ(st::ResetRows(&rows,7,attempt++),Status::kOk);k[0]=alpha;c[0]=beta;
    st::StepLimit limit;
    const auto status=st::FinalizeRows(&rows,safety,1e-200,1e150,&limit);
    // Deliberately enormous scale ratios lose a positive squared ratio. The
    // admitted arithmetic contract fails closed instead of silently deleting it.
    if((alpha==1e-250 && beta==1e120) || (alpha==1e250 && beta==1e-120)) {
      EXPECT_EQ(status,Status::kNonFiniteResult);EXPECT_DOUBLE_EQ(limit.dt,0);continue;
    }
    ASSERT_EQ(status,Status::kOk);
    if(alpha==0 && beta==0) { EXPECT_DOUBLE_EQ(limit.dt,1e150);continue; }
    const long double a=alpha,b=beta;
    const long double exact=4*static_cast<long double>(safety)/(std::sqrt(b*b+4*a)+b);
    EXPECT_LE(static_cast<long double>(limit.dt),exact);
    EXPECT_LT(static_cast<long double>(limit.dt)*limit.dt*a+2*limit.dt*b,4.L);
  }
}
}  // namespace
