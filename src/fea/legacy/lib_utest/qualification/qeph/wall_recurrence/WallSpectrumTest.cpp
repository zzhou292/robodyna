#include "WallRecurrenceSpectrum.h"
#include "WallRecurrenceTestFixture.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
using test::Near;
double Largest2(const Eigen::MatrixXd& g) {
  const long double a=g(0,0),b=g(0,1),c=g(1,1);
  return static_cast<double>((a+c+std::sqrt((a-c)*(a-c)+4*b*b))/2);
}
Eigen::MatrixXd DirectInclusive(const std::vector<WallOperatorRun>& runs) {
  const auto n=runs[0].matrix->rows();
  Eigen::MatrixXd p=Eigen::MatrixXd::Identity(n,n),g=Eigen::MatrixXd::Zero(n,n);
  for(const auto& run:runs) for(unsigned i=0;i<run.count;++i) {
    g+=p.transpose()*p; p=(*run.matrix*p).eval();
  }
  return g+p.transpose()*p;
}
}
TEST(QephWallSpectrum, FullJordanAndPassiveIdentitiesRemainInRawAndWeightedGrams) {
  constexpr unsigned n=4096; constexpr double beta=.125;
  Eigen::MatrixXd a=Eigen::MatrixXd::Identity(3,3); a(0,1)=beta;
  Eigen::VectorXd d=Eigen::VectorXd::Ones(3); d[0]=1/(1+n*beta);
  const auto spectrum=AnalyzeConstantWallSpectrum(a);
  ASSERT_TRUE(spectrum.complete)<<spectrum.diagnostic; EXPECT_TRUE(spectrum.passed);
  EXPECT_EQ(spectrum.eigenvalues.size(),3u); EXPECT_EQ(spectrum.near_one,3u);
  const auto sequence=AnalyzeWallSequence({{&a,n}},d);
  ASSERT_TRUE(sequence.complete)<<sequence.diagnostic; EXPECT_TRUE(sequence.passed);
  EXPECT_FALSE(sequence.raw.within_gain_budget); EXPECT_TRUE(sequence.weighted.within_gain_budget);
  EXPECT_EQ(sequence.raw.state_count,n+1); EXPECT_EQ(sequence.weighted.eigenvalues.size(),3);
  const long double count=n+1,s1=static_cast<long double>(n)*(n+1)/2,s2=static_cast<long double>(n)*(n+1)*(2*n+1)/6;
  for(unsigned weighted=0;weighted<2;++weighted) {
    const double b=beta*(weighted?d[0]:1);
    Eigen::MatrixXd g(2,2); g<<count,b*s1,b*s1,count+b*b*s2;
    const auto& actual=weighted?sequence.weighted:sequence.raw;
    Near(actual.maximum_eigenvalue,Largest2(g)); Near(actual.mean_gain,std::sqrt(Largest2(g)/static_cast<double>(count)));
    Near(actual.individual_power_bound,std::sqrt(Largest2(g)));
    // The passive identity contributes its own eigenvalue; it was not removed.
    Near(actual.eigenvalues[1],count);
  }
  EXPECT_LT(sequence.weighted.mean_gain,(1+std::sqrt(5.))/2);
}
TEST(QephWallSpectrum, NormalOscillatorChecksSignsUnitsAndModifiedQuadratic) {
  constexpr double h=.125,omega=2,length=.5,speed=8;
  constexpr double beta=h*speed/length,gamma=h*omega*omega*length/speed,r=h*omega;
  Eigen::MatrixXd physical(2,2); physical<<1-beta*gamma,beta,-gamma,1;
  Eigen::VectorXd d(2); d<<omega*length/speed,1;
  Eigen::MatrixXd weighted; std::string error;
  ASSERT_TRUE(ApplyWallStateMetric(physical,d,weighted,error));
  Eigen::MatrixXd expected(2,2); expected<<1-r*r,r,-r,1;
  EXPECT_LE((weighted-expected).cwiseAbs().maxCoeff(),2e-12);
  Eigen::MatrixXd invariant(2,2); invariant<<1,-r/2,-r/2,1;
  EXPECT_LE((weighted.transpose()*invariant*weighted-invariant).cwiseAbs().maxCoeff(),2e-12);
  EXPECT_TRUE(AnalyzeConstantWallSpectrum(physical).passed);
  EXPECT_TRUE(AnalyzeConstantWallSpectrum(weighted).passed);
  const auto sequence=AnalyzeWallSequence({{&physical,257}},d);
  ASSERT_TRUE(sequence.complete); EXPECT_TRUE(sequence.passed);
  const Eigen::MatrixXd direct=DirectInclusive({{&weighted,257}});
  Near(sequence.weighted.maximum_eigenvalue,Largest2(direct));
  Eigen::MatrixXd wrong_sign(2,2); wrong_sign<<1+r*r,r,r,1;
  const auto rejected=AnalyzeConstantWallSpectrum(wrong_sign);
  ASSERT_TRUE(rejected.complete); EXPECT_FALSE(rejected.passed); EXPECT_GT(rejected.spectral_radius,1);
  EXPECT_EQ(rejected.eigenvalues.size(),2u);
  const double omitted=h*omega*omega;
  Eigen::MatrixXd wrong_units(2,2); wrong_units<<1-beta*omitted,beta,-omitted,1;
  ASSERT_TRUE(ApplyWallStateMetric(wrong_units,d,weighted,error));
  EXPECT_GT((weighted.transpose()*invariant*weighted-invariant).cwiseAbs().maxCoeff(),1e-3);
}
TEST(QephWallSpectrum, StableBranchesFailAlternatingGainWithoutSpectralTestingFiniteProduct) {
  Eigen::MatrixXd a(2,2),b(2,2); a<<.5,2,0,.5; b<<.5,0,2,.5;
  EXPECT_TRUE(AnalyzeConstantWallSpectrum(a).passed); EXPECT_TRUE(AnalyzeConstantWallSpectrum(b).passed);
  const Eigen::VectorXd d=Eigen::VectorXd::Ones(2);
  const Eigen::MatrixXd pair=b*a;
  EXPECT_FALSE(AnalyzeConstantWallSpectrum(pair).passed);
  // One finite transfer may amplify while respecting the declared mean-gain
  // limit. It must not inherit a spectral-radius test on repeated transfer.
  EXPECT_TRUE(AnalyzeWallSequence({{&pair,1}},d).passed);
  std::vector<WallOperatorRun> runs;
  for(unsigned i=0;i<17;++i) { runs.push_back({&a,1}); runs.push_back({&b,1}); }
  const auto actual=AnalyzeWallSequence(runs,d);
  ASSERT_TRUE(actual.complete)<<actual.diagnostic; EXPECT_FALSE(actual.passed);
  EXPECT_EQ(actual.raw.ordinary_steps,34u); EXPECT_GT(actual.weighted.mean_gain,recurrence::MaximumGramGain);
  const Eigen::MatrixXd direct=DirectInclusive(runs);
  Near(actual.raw.maximum_eigenvalue,Largest2(direct));
  EXPECT_EQ(actual.raw.eigenvalues.size(),2); EXPECT_FALSE(actual.diagnostic.empty());
}
TEST(QephWallSpectrum, ChronologicalZeroOneAndJunctionsMatchIndependentSum) {
  Eigen::MatrixXd a(2,2),b(2,2); a<<.75,.125,0,1; b<<1,0,-.25,.875;
  const Eigen::VectorXd d=Eigen::VectorXd::Ones(2);
  const auto zero=AnalyzeWallSequence({{&a,0}},d);
  ASSERT_TRUE(zero.complete); EXPECT_EQ(zero.raw.state_count,1u); EXPECT_EQ(zero.raw.mean_gain,1);
  const auto one=AnalyzeWallSequence({{&a,1}},d);
  ASSERT_TRUE(one.complete); EXPECT_EQ(one.raw.state_count,2u);
  Near(one.raw.maximum_eigenvalue,Largest2(DirectInclusive({{&a,1}})));
  const auto sequence=AnalyzeWallSequence({{&a,3},{&b,4},{&a,2}},d);
  ASSERT_TRUE(sequence.complete); EXPECT_EQ(sequence.raw.state_count,10u);
  const auto expected=DirectInclusive({{&a,3},{&b,4},{&a,2}});
  Near(sequence.raw.maximum_eigenvalue,Largest2(expected));
  EXPECT_GT(std::abs(Largest2(DirectInclusive({{&a,2},{&b,4},{&a,3}}))-Largest2(expected)),1e-3);
  EXPECT_GT(std::abs(Largest2(expected+Eigen::MatrixXd::Identity(2,2))-sequence.raw.maximum_eigenvalue),.5);
}
TEST(QephWallSpectrum, MalformedNonfiniteOverflowAndAsymmetricEvidenceRejectWithoutClamping) {
  Eigen::MatrixXd a=Eigen::MatrixXd::Identity(2,2); const Eigen::VectorXd d=Eigen::VectorXd::Ones(2);
  EXPECT_FALSE(AnalyzeWallSequence({},d).complete);
  EXPECT_FALSE(AnalyzeWallSequence({{nullptr,1}},d).complete);
  EXPECT_FALSE(AnalyzeWallSequence({{&a,32769}},d).complete);
  Eigen::MatrixXd bad=a; bad(0,0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(AnalyzeConstantWallSpectrum(bad).complete);
  EXPECT_FALSE(AnalyzeWallSequence({{&bad,1}},d).complete);
  bad(0,0)=1e200; const auto overflow=AnalyzeWallSequence({{&bad,2}},d);
  EXPECT_FALSE(overflow.complete); EXPECT_FALSE(overflow.diagnostic.empty());
  recurrence::PowerGramBlock block; std::string error;
  ASSERT_TRUE(recurrence::BuildPowerGramBlock(a,2,block,error)); block.gram(0,1)=1;
  const auto asymmetric=AnalyzeWallGram(block);
  EXPECT_FALSE(asymmetric.complete); EXPECT_GT(asymmetric.antisymmetry,recurrence::DecompositionTolerance);
  EXPECT_EQ(asymmetric.eigenvalues.size(),0); // Failed before symmetrizing/eigensolving.
  block.gram=Eigen::MatrixXd::Identity(2,2); block.gram(0,0)=-10;
  const auto indefinite=AnalyzeWallGram(block);
  EXPECT_FALSE(indefinite.complete); EXPECT_LT(indefinite.minimum_eigenvalue,0);
  EXPECT_EQ(indefinite.eigenvalues.size(),2); // Measured failure is retained.
  EXPECT_TRUE(AnalyzeWallSequence({{&a,2}},d).passed);
}
} // namespace tl::qualification::qeph::wall_recurrence
