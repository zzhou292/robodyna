#include "RecurrenceIdentity.h"
#include <gtest/gtest.h>
#include <cmath>
#include <set>

namespace tl::qualification::qeph::recurrence {
namespace {
unsigned Index(const Model& m,Group g,unsigned entity,unsigned component) {
  for(unsigned i=0;i<m.dictionary.size();++i) { const auto& c=m.dictionary[i];
    if(c.group==g&&c.entity==entity&&c.component==component) return i; }
  ADD_FAILURE()<<"Missing dictionary field"; return 0;
}
void Near(double actual,double expected,double dimension=1) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(actual-expected),2e-12*(std::abs(expected)+dimension));
}
}
TEST(QephRecurrenceAudit, CompleteDictionaryUsesActualNativeSharedMassAndHistoryDimensions) {
  for(unsigned cells:{1u,2u}) {
    Model m; std::string error; ASSERT_TRUE(BuildModel(cells,m,error))<<error;
    EXPECT_EQ(m.dictionary.size(),12*m.nodes+61*cells); EXPECT_EQ(FeedbackIndices(m).size(),6*m.nodes+44*cells);
    std::set<std::string> names;
    for(const auto& c:m.dictionary) { EXPECT_TRUE(names.insert(c.name).second); EXPECT_GT(c.scale,0); EXPECT_FALSE(c.unit.empty()); }
    const long double cell_mass=Density*static_cast<long double>(Thickness)*Length*Length;
    long double mass=0,inertia=0;
    for(unsigned n=0;n<m.nodes;++n) { mass+=m.mass[n]; inertia+=m.inertia[n]; }
    Near(static_cast<double>(mass),static_cast<double>(cells*cell_mass),static_cast<double>(mass));
    Near(static_cast<double>(inertia),static_cast<double>(cells*cell_mass*(Length*Length+Thickness*Thickness)/12),static_cast<double>(inertia));
    for(unsigned e=0;e<cells;++e) {
      Near(m.dictionary[Index(m,Group::History,e,10)].scale,Young*Thickness/Length);
      Near(m.dictionary[Index(m,Group::History,e,15)].scale,Young/Length);
      EXPECT_FALSE(m.dictionary[Index(m,Group::History,e,0)].feedback);
      EXPECT_TRUE(m.dictionary[Index(m,Group::History,e,5)].feedback);
      EXPECT_FALSE(m.dictionary[Index(m,Group::History,e,33)].feedback);
    }
    Eigen::VectorXd zero=Eigen::VectorXd::Zero(m.dictionary.size()),out;
    ASSERT_TRUE(NativeMap(m,H0,zero,out,error))<<error; EXPECT_EQ(out.cwiseAbs().maxCoeff(),0);
  }
}
TEST(QephRecurrenceAudit, NativeMapDistinguishesOmittedCacheAndFrozenMaterialHistory) {
  Model m; std::string error; ASSERT_TRUE(BuildModel(2,m,error));
  Eigen::VectorXd state=Eigen::VectorXd::Zero(m.dictionary.size()),out,zero_result;
  ASSERT_TRUE(NativeMap(m,H0,state,zero_result,error));
  const auto cache=Index(m,Group::ForceCache,0,2),velocity=Index(m,Group::Velocity,0,2),position=Index(m,Group::Position,0,2);
  state[cache]=Amplitudes[0]; ASSERT_TRUE(NativeMap(m,H0,state,out,error))<<error;
  const double force=state[cache]*m.dictionary[cache].scale;
  Near(out[velocity],-H0*force/m.mass[0]/m.dictionary[velocity].scale);
  Near(out[position],-H0*H0*force/m.mass[0]/Length);
  EXPECT_GT(std::abs(out[velocity]-zero_result[velocity])/Amplitudes[0],32*MatrixTolerance);
  // A nonzero material FOR_G with zero cache/velocity must persist and produce
  // the independent uniform membrane traction; resetting H loses this column.
  state.setZero(); const auto stress=Index(m,Group::History,0,5); state[stress]=Amplitudes[0];
  ASSERT_TRUE(NativeMap(m,H0,state,out,error)); Near(out[stress],state[stress]);
  const double traction=Young*state[stress]*Thickness*Length*.5;
  for(unsigned i=0;i<4;++i) {
    const auto field=Index(m,Group::ForceCache,0,3*i);
    Near(out[field]*m.dictionary[field].scale,(i==0||i==3?-1.:1.)*traction,traction);
    EXPECT_GT(std::abs(out[field]-zero_result[field])/Amplitudes[0],32*MatrixTolerance);
  }
}
TEST(QephRecurrenceAudit, ActualCenteredMapRetainsCacheHistoryAndKnownNeutralColumns) {
  Model m; std::string error; ASSERT_TRUE(BuildModel(1,m,error));
  const auto p=Differentiate(m,H0,Amplitudes.back()); ASSERT_TRUE(p.complete)<<p.diagnostic;
  EXPECT_EQ(p.completed_columns,m.dictionary.size()); ASSERT_TRUE(p.full.allFinite());
  const auto cache=Index(m,Group::ForceCache,0,2),velocity=Index(m,Group::Velocity,0,2);
  const double coefficient=-H0*m.dictionary[cache].scale/(m.mass[0]*m.dictionary[velocity].scale);
  Near(p.full(velocity,cache),coefficient);
  const auto stress=Index(m,Group::History,0,5),force=Index(m,Group::ForceCache,0,0);
  Near(p.full(force,stress),-.5);
  Eigen::MatrixXd omitted_cache=p.full,frozen_history=p.full;
  for(unsigned i=0;i<m.dictionary.size();++i) {
    if(m.dictionary[i].group==Group::ForceCache) omitted_cache.col(i).setZero();
    if(m.dictionary[i].group==Group::History&&m.dictionary[i].feedback) frozen_history.col(i).setZero();
  }
  EXPECT_GT((omitted_cache-p.full).cwiseAbs().maxCoeff(),32*MatrixTolerance);
  EXPECT_GT((frozen_history-p.full).cwiseAbs().maxCoeff(),32*MatrixTolerance);
  MapAnalysis identity; ASSERT_TRUE(CheckStructuralIdentities(m,H0,p.full,identity))<<identity.diagnostic;
  EXPECT_LE(identity.zero_feedback_error,MatrixTolerance); EXPECT_LE(identity.observer_error,MatrixTolerance);
  EXPECT_LE(identity.rigid_error,MatrixTolerance); EXPECT_LE(identity.rigid_basis_condition,1e8);
  EXPECT_LE(identity.rigid_projection_residual,DecompositionTolerance);
}
TEST(QephRecurrenceAudit, GramBinaryCompositionMatchesIndependentIdentityJordanAndDirectSums) {
  std::string error; Eigen::MatrixXd g;
  const Eigen::MatrixXd identity=Eigen::MatrixXd::Identity(3,3);
  ASSERT_TRUE(PowerGram(identity,32769,g,error)); EXPECT_TRUE(g.isApprox(32769*identity,1e-14));
  Eigen::MatrixXd jordan=Eigen::MatrixXd::Identity(2,2); jordan(0,1)=.125;
  constexpr unsigned n=17; ASSERT_TRUE(PowerGram(jordan,n,g,error));
  Near(g(0,0),n); Near(g(0,1),.125*n*(n-1)/2); Near(g(1,0),g(0,1));
  Near(g(1,1),n+.125*.125*n*(n-1)*(2*n-1)/6);
  Eigen::MatrixXd a(2,2); a<<.7,.3,-.2,.8;
  Eigen::MatrixXd power=Eigen::MatrixXd::Identity(2,2),direct=Eigen::MatrixXd::Zero(2,2);
  for(unsigned i=0;i<13;++i) { direct+=power.transpose()*power; power=(a*power).eval(); }
  ASSERT_TRUE(PowerGram(a,13,g,error)); EXPECT_TRUE(g.isApprox(direct,2e-12));
}
TEST(QephRecurrenceAudit, GramExposesUnstableAndNonnormalMapsDespiteBenignEigenvalueInTheLatter) {
  std::string error; Eigen::MatrixXd g,a(1,1); a(0,0)=1.01;
  ASSERT_TRUE(PowerGram(a,2049,g,error)); EXPECT_GT(std::sqrt(g(0,0)/2049),MaximumGramGain);
  Eigen::MatrixXd transient(2,2); transient<<.99,100,0,.99;
  ASSERT_TRUE(PowerGram(transient,257,g,error)); EXPECT_GT(std::sqrt(g(1,1)/257),MaximumGramGain);
  // The matrix is triangular, so both eigenvalues are independently .99.
  EXPECT_LT(transient(0,0),1); EXPECT_EQ(transient(0,0),transient(1,1));
}
TEST(QephRecurrenceAudit, FailedNativeAndLateGramProbesPreserveCallerOutputAndRetry) {
  Model m; std::string error; ASSERT_TRUE(BuildModel(1,m,error));
  Eigen::VectorXd input=Eigen::VectorXd::Zero(m.dictionary.size()),out=Eigen::VectorXd::Constant(3,19);
  const auto saved=out; input[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(NativeMap(m,H0,input,out,error)); EXPECT_EQ(out,saved);
  input.setZero(); auto bad=m; bad.connectivity[0][3]=MaxNodes;
  EXPECT_FALSE(NativeMap(bad,H0,input,out,error)); EXPECT_EQ(out,saved);
  EXPECT_FALSE(NativeMap(m,0,input,out,error)); EXPECT_EQ(out,saved);
  ASSERT_TRUE(NativeMap(m,H0,input,out,error)); EXPECT_EQ(out.cwiseAbs().maxCoeff(),0);
  Eigen::MatrixXd g=Eigen::MatrixXd::Constant(2,2,17),overflow=Eigen::MatrixXd::Constant(1,1,1e308);
  const auto held=g;
  EXPECT_FALSE(PowerGram(overflow,8,g,error)); EXPECT_EQ(g,held);
  EXPECT_FALSE(PowerGram(Eigen::MatrixXd::Identity(2,2),0,g,error)); EXPECT_EQ(g,held);
  ASSERT_TRUE(PowerGram(Eigen::MatrixXd::Identity(2,2),9,g,error)); EXPECT_EQ(g,9*Eigen::MatrixXd::Identity(2,2));
  const auto incomplete=Differentiate(m,H0,0); EXPECT_FALSE(incomplete.complete); EXPECT_EQ(incomplete.completed_columns,0u);
  EXPECT_TRUE(incomplete.full.allFinite()); EXPECT_FALSE(incomplete.diagnostic.empty());
}
} // namespace tl::qualification::qeph::recurrence
