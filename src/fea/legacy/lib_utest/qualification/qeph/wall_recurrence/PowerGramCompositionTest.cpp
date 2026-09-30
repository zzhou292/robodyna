#include "lib_utest/qualification/qeph/free_response/RecurrenceAudit.h"
#include "lib_utest/qualification/qeph/free_response/RecurrencePowerGram.h"
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
namespace r=recurrence;
void NearMatrix(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b) {
  ASSERT_EQ(a.rows(),b.rows()); ASSERT_EQ(a.cols(),b.cols());
  ASSERT_TRUE(a.allFinite()); ASSERT_TRUE(b.allFinite());
  EXPECT_LE((a-b).cwiseAbs().maxCoeff(),2e-12*std::max(1.,b.cwiseAbs().maxCoeff()));
}
void SameBlock(const r::PowerGramBlock& a,const r::PowerGramBlock& b) {
  EXPECT_EQ(a.count,b.count); EXPECT_EQ(a.power,b.power); EXPECT_EQ(a.gram,b.gram);
}
// Independent literal time stepping: sum each state once, THEN apply that
// step's map. There is no binary composition or commutation assumption.
r::PowerGramBlock Direct(const std::vector<Eigen::MatrixXd>& sequence) {
  const auto n=sequence.front().rows();
  r::PowerGramBlock b{Eigen::MatrixXd::Identity(n,n),Eigen::MatrixXd::Zero(n,n),0};
  for(const auto& a:sequence) {
    b.gram+=b.power.transpose()*b.power;
    b.power=(a*b.power).eval(); ++b.count;
  }
  return b;
}
}
TEST(QephWallPowerGram, ZeroOneAndEndpointCountsPreserveLegacyContract) {
  std::string error; const Eigen::MatrixXd identity=Eigen::MatrixXd::Identity(3,3);
  Eigen::MatrixXd a=identity; a(0,1)=.125;
  r::PowerGramBlock zero,one,many;
  ASSERT_TRUE(r::BuildPowerGramBlock(a,0,zero,error))<<error;
  EXPECT_EQ(zero.count,0u); EXPECT_EQ(zero.power,identity); EXPECT_EQ(zero.gram,Eigen::MatrixXd::Zero(3,3));
  Eigen::MatrixXd inclusive;
  ASSERT_TRUE(r::EndpointInclusiveGram(zero,inclusive,error)); EXPECT_EQ(inclusive,identity);
  ASSERT_TRUE(r::BuildPowerGramBlock(a,1,one,error));
  EXPECT_EQ(one.count,1u); EXPECT_EQ(one.power,a); EXPECT_EQ(one.gram,identity);
  ASSERT_TRUE(r::EndpointInclusiveGram(one,inclusive,error)); NearMatrix(inclusive,identity+a.transpose()*a);
  r::PowerGramBlock left,right;
  ASSERT_TRUE(r::ComposePowerGramBlocks(zero,one,left,error)); SameBlock(left,one);
  ASSERT_TRUE(r::ComposePowerGramBlocks(one,zero,right,error)); SameBlock(right,one);
  Eigen::MatrixXd old=Eigen::MatrixXd::Constant(2,2,19); const auto held=old;
  EXPECT_FALSE(r::PowerGram(a,0,old,error)); EXPECT_EQ(old,held);
  ASSERT_TRUE(r::BuildPowerGramBlock(identity,r::MaximumPowerGramCount,many,error));
  EXPECT_EQ(many.count,32769u); EXPECT_EQ(many.power,identity); EXPECT_EQ(many.gram,32769*identity);
  for(unsigned count:{1u,13u,257u}) {
    ASSERT_TRUE(r::PowerGram(a,count,old,error));
    ASSERT_TRUE(r::BuildPowerGramBlock(a,count,many,error)); EXPECT_EQ(many.gram,old);
  }
  auto alias=one;
  ASSERT_TRUE(r::BuildPowerGramBlock(alias.power,1,alias,error)); SameBlock(alias,one);
}
TEST(QephWallPowerGram, NoncommutingChronologicalBlocksMatchEveryDirectStateOnce) {
  Eigen::MatrixXd a(2,2),b(2,2),c(2,2);
  a<<.75,.125,0,1; b<<1,0,-.25,.875; c<<.5,-.125,.25,.75;
  std::string error; r::PowerGramBlock aa,bb,cc,ab,abc;
  ASSERT_TRUE(r::BuildPowerGramBlock(a,3,aa,error));
  ASSERT_TRUE(r::BuildPowerGramBlock(b,4,bb,error));
  ASSERT_TRUE(r::BuildPowerGramBlock(c,2,cc,error));
  ASSERT_TRUE(r::ComposePowerGramBlocks(aa,bb,ab,error));
  ASSERT_TRUE(r::ComposePowerGramBlocks(ab,cc,abc,error));
  std::vector<Eigen::MatrixXd> sequence(3,a); sequence.insert(sequence.end(),4,b); sequence.insert(sequence.end(),2,c);
  const auto direct=Direct(sequence);
  EXPECT_EQ(abc.count,9u); NearMatrix(abc.power,direct.power); NearMatrix(abc.gram,direct.gram);
  Eigen::MatrixXd inclusive;
  ASSERT_TRUE(r::EndpointInclusiveGram(abc,inclusive,error));
  const Eigen::MatrixXd expected=direct.gram+direct.power.transpose()*direct.power;
  NearMatrix(inclusive,expected);
  const Eigen::MatrixXd wrong_order=aa.power*bb.power*cc.power;
  EXPECT_GT((wrong_order-direct.power).cwiseAbs().maxCoeff(),1e-3);
  const Eigen::MatrixXd duplicate_junction=expected+aa.power.transpose()*aa.power;
  EXPECT_GT((duplicate_junction-inclusive).cwiseAbs().maxCoeff(),1e-3);
  // Both input/output alias directions must stage the complete result.
  auto alias_first=aa,alias_second=bb;
  ASSERT_TRUE(r::ComposePowerGramBlocks(alias_first,bb,alias_first,error)); SameBlock(alias_first,ab);
  ASSERT_TRUE(r::ComposePowerGramBlocks(aa,alias_second,alias_second,error)); SameBlock(alias_second,ab);
  ASSERT_TRUE(r::EndpointInclusiveGram(alias_first,alias_first.gram,error));
  NearMatrix(alias_first.gram,ab.gram+ab.power.transpose()*ab.power);
}
TEST(QephWallPowerGram, NormalKickDriftConservesModifiedFormAndRejectsSignScaleControls) {
  // Independent physical x/v -> z=omega*x/v transformation. These binary
  // operands distinguish a missing physical dictionary scale by a factor 16.
  constexpr double h=.125,omega=2,length=.5,speed=8,rh=h*omega;
  constexpr double beta=h*speed/length,gamma=h*omega*omega*length/speed;
  Eigen::MatrixXd physical(2,2),d=Eigen::MatrixXd::Identity(2,2),inverse=d;
  physical<<1-beta*gamma,beta,-gamma,1;
  d(0,0)=omega*length/speed; inverse(0,0)=1/d(0,0);
  Eigen::MatrixXd oscillator(2,2); oscillator<<1-rh*rh,rh,-rh,1;
  NearMatrix(d*physical*inverse,oscillator);
  Eigen::MatrixXd invariant(2,2); invariant<<1,-rh/2,-rh/2,1;
  EXPECT_GT(1-rh/2,0); NearMatrix(oscillator.transpose()*invariant*oscillator,invariant);
  std::string error; r::PowerGramBlock block;
  ASSERT_TRUE(r::BuildPowerGramBlock(oscillator,257,block,error));
  NearMatrix(block.power.transpose()*invariant*block.power,invariant);
  const double missing_scale=h*omega*omega;
  Eigen::MatrixXd wrong_scale(2,2); wrong_scale<<1-beta*missing_scale,beta,-missing_scale,1;
  const Eigen::MatrixXd bad=d*wrong_scale*inverse;
  EXPECT_GT((bad.transpose()*invariant*bad-invariant).cwiseAbs().maxCoeff(),1e-3);
  Eigen::MatrixXd wrong_sign(2,2); wrong_sign<<1+rh*rh,rh,rh,1;
  EXPECT_GT((wrong_sign.transpose()*invariant*wrong_sign-invariant).cwiseAbs().maxCoeff(),1e-3);
  ASSERT_TRUE(r::BuildPowerGramBlock(wrong_sign,257,block,error));
  EXPECT_GT(std::sqrt(block.gram(0,0)/block.count),r::MaximumGramGain);
}
TEST(QephWallPowerGram, FullMetricKeepsPassiveIdentityAndBoundsTangentialDrift) {
  constexpr unsigned n=4096;
  const double c=std::sqrt(r::Young/r::Density),eta=n*r::H0*c/r::Length,weight=1/(1+eta);
  Eigen::MatrixXd a=Eigen::MatrixXd::Identity(4,4);
  a(0,1)=r::H0*c/r::Length*weight; // Remaining coordinates are passive identities.
  std::string error; r::PowerGramBlock b;
  ASSERT_TRUE(r::BuildPowerGramBlock(a,n,b,error));
  Eigen::MatrixXd expected=Eigen::MatrixXd::Identity(4,4); expected(0,1)=eta/(1+eta);
  NearMatrix(b.power,expected); EXPECT_EQ(b.gram(2,2),n); EXPECT_EQ(b.gram(3,3),n);
  // For [[1,s],[0,1]], the exact larger singular value is
  // (sqrt(s*s+4)+abs(s))/2. It approaches, but does not reach, phi for s<1.
  const double s=expected(0,1),phi=(1+std::sqrt(5.))/2;
  EXPECT_LT(s,1); EXPECT_LT((std::sqrt(s*s+4)+s)/2,phi);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(b.power.transpose()*b.power,Eigen::EigenvaluesOnly);
  ASSERT_EQ(spectrum.info(),Eigen::Success); EXPECT_LT(std::sqrt(spectrum.eigenvalues().maxCoeff()),phi);
  Eigen::MatrixXd inclusive;
  ASSERT_TRUE(r::EndpointInclusiveGram(b,inclusive,error));
  EXPECT_EQ(inclusive(2,2),n+1); EXPECT_EQ(inclusive(3,3),n+1);
  EXPECT_EQ(inclusive(2,0),0); EXPECT_EQ(inclusive(3,1),0);
}
TEST(QephWallPowerGram, StableBranchesDoNotAdmitTheirRepeatedAlternatingProduct) {
  Eigen::MatrixXd a(2,2),b(2,2); a<<.5,2,0,.5; b<<.5,0,2,.5;
  // Triangular branches both have exactly .5 eigenvalues. Their chronological
  // product b*a has trace4.5 and determinant1/16, hence an eigenvalue>1.
  EXPECT_EQ(a(0,0),.5); EXPECT_EQ(a(1,1),.5);
  EXPECT_EQ(b(0,0),.5); EXPECT_EQ(b(1,1),.5);
  std::string error; r::PowerGramBlock aa,bb,pair,repeated;
  ASSERT_TRUE(r::BuildPowerGramBlock(a,1,aa,error));
  ASSERT_TRUE(r::BuildPowerGramBlock(b,1,bb,error));
  ASSERT_TRUE(r::ComposePowerGramBlocks(aa,bb,pair,error)); NearMatrix(pair.power,b*a);
  const double trace=4.5,determinant=.0625;
  EXPECT_GT((trace+std::sqrt(trace*trace-4*determinant))/2,1);
  ASSERT_TRUE(r::BuildPowerGramBlock(pair.power,17,repeated,error));
  EXPECT_GT(std::sqrt(repeated.gram(0,0)/repeated.count),r::MaximumGramGain);
  // Direct alternating accumulation includes the odd-step states too.
  std::vector<Eigen::MatrixXd> sequence;
  for(unsigned i=0;i<17;++i) { sequence.push_back(a); sequence.push_back(b); }
  const auto direct=Direct(sequence);
  r::PowerGramBlock actual=pair;
  for(unsigned i=1;i<17;++i) {
    ASSERT_TRUE(r::ComposePowerGramBlocks(actual,pair,actual,error));
  }
  EXPECT_EQ(actual.count,34u); NearMatrix(actual.power,direct.power); NearMatrix(actual.gram,direct.gram);
}
TEST(QephWallPowerGram, MalformedAndLateOverflowLeaveEveryOutputIntactThenRetry) {
  const Eigen::MatrixXd identity=Eigen::MatrixXd::Identity(2,2);
  std::string error; r::PowerGramBlock held;
  ASSERT_TRUE(r::BuildPowerGramBlock(identity,3,held,error));
  auto output=held;
  Eigen::MatrixXd huge=Eigen::MatrixXd::Constant(1,1,1e200);
  EXPECT_FALSE(r::BuildPowerGramBlock(huge,2,output,error)); SameBlock(output,held);
  EXPECT_FALSE(r::BuildPowerGramBlock(identity,r::MaximumPowerGramCount+1,output,error)); SameBlock(output,held);
  r::PowerGramBlock explosive;
  ASSERT_TRUE(r::BuildPowerGramBlock(huge,1,explosive,error));
  EXPECT_FALSE(r::ComposePowerGramBlocks(explosive,explosive,output,error)); SameBlock(output,held);
  Eigen::MatrixXd gram=Eigen::MatrixXd::Constant(2,2,19),saved=gram;
  EXPECT_FALSE(r::EndpointInclusiveGram(explosive,gram,error)); EXPECT_EQ(gram,saved);
  const auto explosive_saved=explosive;
  EXPECT_FALSE(r::ComposePowerGramBlocks(explosive,explosive,explosive,error)); SameBlock(explosive,explosive_saved);
  auto malformed=held; malformed.count=0;
  EXPECT_FALSE(r::ComposePowerGramBlocks(malformed,held,output,error)); SameBlock(output,held);
  malformed=held; malformed.gram(1,1)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(r::EndpointInclusiveGram(malformed,gram,error)); EXPECT_EQ(gram,saved);
  r::PowerGramBlock capacity,one;
  ASSERT_TRUE(r::BuildPowerGramBlock(identity,r::MaximumPowerGramCount,capacity,error));
  ASSERT_TRUE(r::BuildPowerGramBlock(identity,1,one,error));
  EXPECT_FALSE(r::ComposePowerGramBlocks(capacity,one,output,error)); SameBlock(output,held);
  ASSERT_TRUE(r::ComposePowerGramBlocks(held,one,output,error));
  EXPECT_EQ(output.count,4u); EXPECT_EQ(output.power,identity); EXPECT_EQ(output.gram,4*identity);
  ASSERT_TRUE(r::EndpointInclusiveGram(output,gram,error)); EXPECT_EQ(gram,5*identity); EXPECT_TRUE(error.empty());
}
} // namespace tl::qualification::qeph::wall_recurrence
