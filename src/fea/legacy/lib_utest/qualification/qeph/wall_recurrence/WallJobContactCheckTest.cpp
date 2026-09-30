#include "WallJobAnalysisTestFixture.h"
#include "ContactDerivativeChecks.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace jt=job_test;
TEST(QephWallJobContact, RecomputesFrozenDirectionsAndQuotientsWithoutSavedPassFlags) {
  for(unsigned cells:{1u,2u}) for(auto branch:{ContactBranch::Inactive,ContactBranch::Active}) {
    auto job=jt::Job(cells,8); const auto dimension=job.model.native().dictionary.size();
    const Eigen::MatrixXd shell=Eigen::MatrixXd::Identity(dimension,dimension);
    const auto probe=jt::Contact(job,0,branch,shell);
    ASSERT_FALSE(probe.complete); ASSERT_FALSE(probe.passed);
    const auto result=RecheckWallContact(job.model,job.steps[0].h,8,branch,shell,probe);
    ASSERT_TRUE(result.complete)<<result.diagnostic; EXPECT_TRUE(result.passed)<<result.diagnostic;
    EXPECT_EQ(result.completed_directions,job.model.native().nodes+7);
    for(unsigned i=0;i<result.directions.size();++i) {
      EXPECT_TRUE(result.directions[i].samples_valid);
      for(unsigned level=0;level<2;++level) {
        const Eigen::VectorXd expected=probe.full*probe.directions[i].direction.value;
        EXPECT_LE((result.directions[i].quotients[level]-expected).cwiseAbs().maxCoeff(),2e-12);
        EXPECT_LE(result.directions[i].residual_upper[level],result.directions[i].budget_lower[level]);
      }
    }
  }
}
TEST(QephWallJobContact, TamperedSamplesCannotHideBehindSavedPassingQuotients) {
  const auto job=jt::Job(); const auto n=job.model.native().dictionary.size();
  const Eigen::MatrixXd shell=Eigen::MatrixXd::Identity(n,n);
  auto probe=jt::Contact(job,0,ContactBranch::Active,shell);
  probe.complete=true; probe.passed=true;
  for(auto& direction:probe.directions) {
    direction.complete=true; direction.passed=true;
    for(auto& quotient:direction.quotients) quotient=probe.full*direction.direction.value;
    direction.residual_upper={0,0}; direction.budget={1e100,1e100};
  }
  const unsigned last=probe.directions.size()-1;
  probe.directions[last].samples[2].state[n-1]+=1e-3*recurrence::Amplitudes[2];
  const auto result=RecheckWallContact(job.model,job.steps[0].h,0,ContactBranch::Active,shell,probe);
  ASSERT_TRUE(result.complete); EXPECT_FALSE(result.passed);
  EXPECT_TRUE(result.directions.front().passed); EXPECT_FALSE(result.directions.back().passed);
  EXPECT_GT(result.directions.back().residual_upper[1],result.directions.back().budget_lower[1]);
  EXPECT_EQ(probe.directions[last].residual_upper[1],0); EXPECT_TRUE(probe.passed);
}
TEST(QephWallJobContact, WrongMaskIdentityDirectionOperatorAndNonfiniteRecordsReject) {
  const auto job=jt::Job(); const auto n=job.model.native().dictionary.size();
  const Eigen::MatrixXd shell=Eigen::MatrixXd::Identity(n,n);
  const auto clean=jt::Contact(job,0,ContactBranch::Active,shell);
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault); auto p=clean;
    if(fault==0) p.directions.back().samples[2].nodes.back().touching_or_penetrating=false;
    if(fault==1) ++p.directions.back().samples[2].attempt;
    if(fault==2) p.directions.back().direction.value[n-1]=2;
    if(fault==3) p.full(0,0)+=.01;
    if(fault==4) p.directions.back().samples[2].state[0]=std::numeric_limits<double>::infinity();
    if(fault==5) p.directions.back().completed_samples=2;
    if(fault==6) p.baseline.state[0]+=.001;
    const auto result=RecheckWallContact(job.model,job.steps[0].h,0,ContactBranch::Active,shell,p);
    EXPECT_FALSE(result.passed); EXPECT_FALSE(result.diagnostic.empty());
    if(fault==3) { EXPECT_TRUE(result.complete); EXPECT_FALSE(result.operator_difference.passed); }
    else if(fault==6) { EXPECT_TRUE(result.complete); EXPECT_FALSE(result.baseline.passed); }
    else EXPECT_FALSE(result.complete);
  }
  EXPECT_TRUE(RecheckWallContact(job.model,job.steps[0].h,0,ContactBranch::Active,shell,clean).passed);
}
TEST(QephWallJobContact, SharedArithmeticRetainsCancellationAndNonfiniteEvidence) {
  Eigen::VectorXd base(2),coarse(2),fine(2); base<<8,0; coarse<<8.25,0; fine<<8.125,0;
  const auto q=derivative_detail::OneSidedQuotient(base,coarse,fine,.25);
  EXPECT_EQ(q[0],1); EXPECT_EQ(q[1],0);
  const Eigen::MatrixXd identity=Eigen::MatrixXd::Identity(2,2); double residual=0,budget=0;
  ASSERT_TRUE(derivative_detail::CompareDirection(identity,q,q,residual,budget)); EXPECT_LE(residual,budget);
  coarse[1]=std::numeric_limits<double>::max(); fine[1]=-std::numeric_limits<double>::max();
  const auto overflow=derivative_detail::OneSidedQuotient(base,coarse,fine,.25);
  EXPECT_FALSE(overflow.allFinite()); EXPECT_EQ(overflow[0],1);
}
} // namespace tl::qualification::qeph::wall_recurrence
