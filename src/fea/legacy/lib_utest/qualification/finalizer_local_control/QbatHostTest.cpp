// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatSupport.h"
namespace final_control_test::qbat {
TEST(QbatLocalControlHost, CompleteControlRetainsSeedsAndEveryFailurePhase) {
  f::Fixture fixture(129);ASSERT_FALSE(HasFailure());
  for(unsigned fault=0;fault<=10;++fault)for(bool valid:{false,true})for(bool coupled:{false,true}) {
    SCOPED_TRACE(::testing::Message()<<fault<<":"<<valid<<":"<<coupled);
    fixture.Reset();fixture.host->model.config.usage=coupled?q::BatchUsage::CoupledForces:q::BatchUsage::PrescribedFields;
    f::Fault(fixture,fault);fixture.Stage();auto identity=Seed(valid,0x1p54);
    identity.usage=fixture.host->model.config.usage;
    Compare(fixture,identity);
    if(fault==4||fault==5) {
      EXPECT_EQ(fixture.host->control.status,q::BatchStatus::ElementFailure);
      EXPECT_EQ(fixture.host->control.diagnostics.valid,valid);
      EXPECT_EQ(fixture.host->control.diagnostics.element_count,91u);
    }
    fixture.Reset();fixture.Stage();Compare(fixture,Seed(valid,-0.));
    EXPECT_EQ(fixture.host->control.status,q::BatchStatus::Success);
  }
}
TEST(QbatLocalControlHost, FullStatusPrescanDoesNotReadUnavailableMeasurementAndRepairsSameState) {
  f::Fixture fixture(129);ASSERT_FALSE(HasFailure());fixture.Stage();
  const auto measurement=fixture.host->assembly.measurement;const auto maximum=fixture.host->assembly.maximum;
  fixture.host->candidate_status[128]=q::Status::kInvalidInput;
  fixture.host->candidate_status[17]=q::Status::kInvalidReference;
  fixture.host->assembly.measurement=nullptr;fixture.host->assembly.maximum=nullptr;
  Compare(fixture,Seed(true,-0.));
  EXPECT_EQ(fixture.host->control.element,17u);EXPECT_TRUE(fixture.host->control.diagnostics.valid);
  fixture.host->assembly.measurement=measurement;fixture.host->assembly.maximum=maximum;
  fixture.host->candidate_status[17]=fixture.host->candidate_status[128]=q::Status::kSuccess;
  Compare(fixture,Seed(false,.25));EXPECT_EQ(fixture.host->control.status,q::BatchStatus::Success);
}
TEST(QbatLocalControlHost, StagedCancellationAndDisplacementFallbackKeepExactPartialValues) {
  f::Fixture fixture;ASSERT_FALSE(HasFailure());
  for(double incoming:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}) {
    fixture.Reset();fixture.Stage();
    const double terms[]{0x1p54,1,-0x1p54,.25};
    for(unsigned i=0;i<4;++i) {
      fixture.host->assembly.measurement[i].internal_work[0]=terms[i];
      fixture.host->assembly.measurement[i].kick_operand[0]=terms[i];
    }
    Compare(fixture,Seed(true,incoming));
    fixture.host->assembly.maximum[0].valid=false; // qualified serial displacement fallback
    fixture.input.endpoint[0]=std::numeric_limits<double>::quiet_NaN();
    Compare(fixture,Seed(true,incoming));
    EXPECT_EQ(fixture.host->control.status,q::BatchStatus::NonfiniteResult);
  }
}
TEST(QbatLocalControlHost, AdmittedArenaSeparatesControlFromEveryConsumedRange) {
  f::Fixture fixture(129);ASSERT_FALSE(HasFailure());const auto& s=*fixture.host;
  const auto separate=[&](const void* p,std::size_t bytes) {
    EXPECT_TRUE(tl::fea::trial_identity::Disjoint(&s.control,sizeof(s.control),p,bytes));
  };
  separate(s.model.element,fixture.count*sizeof(*s.model.element));
  separate(s.candidate_status,fixture.count*sizeof(*s.candidate_status));
  separate(s.assembly.measurement,fixture.count*sizeof(*s.assembly.measurement));
  separate(s.assembly.maximum,m::MaximumBlocks(f::g::Nodes)*sizeof(*s.assembly.maximum));
  for(unsigned i=0;i<2;++i)separate(s.slab[i].element,fixture.count*sizeof(*s.slab[i].element));
}
} // namespace final_control_test::qbat
