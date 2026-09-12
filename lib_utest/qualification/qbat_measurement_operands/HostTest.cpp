// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace qbat_measurement_test {
TEST(QbatMeasurementHost,AllRemovalMasksVirginCarriedAndPrescribedKeepExactDiagnostics) {
  Fixture f;
  ASSERT_FALSE(HasFailure());
  for (unsigned epoch=0; epoch<4; ++epoch) {
    for (unsigned mask=0; mask<16; ++mask) {
      for (bool coupled : {false,true}) {
        SCOPED_TRACE(::testing::Message()<<epoch<<":"<<mask<<":"<<coupled);
        f.Reset(epoch);
        f.host->model.config.usage=coupled?q::BatchUsage::CoupledForces:q::BatchUsage::PrescribedFields;
        for (unsigned parent=0; parent<4; ++parent) {
          if (mask&(1u<<parent)) g::Remove(f.host->slab[1].element[parent]);
        }
        std::vector<q::BatchResult> accepted(f.host->slab[0].element,f.host->slab[0].element+4);
        std::vector<q::BatchResult> trial(f.host->slab[1].element,f.host->slab[1].element+4);
        f.Compare(epoch);
        EXPECT_EQ(f.host->control.status,q::BatchStatus::Success);
        EXPECT_EQ(std::memcmp(accepted.data(),f.host->slab[0].element,4*sizeof(q::BatchResult)),0);
        EXPECT_EQ(std::memcmp(trial.data(),f.host->slab[1].element,4*sizeof(q::BatchResult)),0);
      }
    }
  }
}
TEST(QbatMeasurementHost,EveryFailurePhaseAndDerivedOverflowRetainPrefixAndFreshRetry) {
  Fixture f;
  ASSERT_FALSE(HasFailure());
  for (unsigned fault=0; fault<=10; ++fault) {
    SCOPED_TRACE(fault);
    f.Reset();
    std::fill_n(f.host->assembly.measurement,f.count,m::MeasurementParent{});
    Fault(f,fault);
    f.Compare();
    if (fault) EXPECT_NE(f.host->control.status,q::BatchStatus::Success);
    if (fault==4 || fault==5) EXPECT_EQ(f.host->control.status,q::BatchStatus::ElementFailure);
    f.Reset();
    f.Compare();
    EXPECT_EQ(f.host->control.status,q::BatchStatus::Success);
  }
}
TEST(QbatMeasurementHost,IndividualSubtractionOperandsRetainIncomingAccumulatorAndSignedZeros) {
  Fixture f;
  ASSERT_FALSE(HasFailure());
  for (double incoming : {0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}) {
    f.Reset();
    for (unsigned parent=0; parent<4; ++parent) {
      auto& row=f.host->slab[1].element[parent];
      row.history.internal_work_j[0]=parent==0?0x1p54:parent==1?1.:parent==2?-0x1p54:.25;
      row.history.internal_work_j[1]=-0.;
    }
    f.Stage();
    auto actual=g::Identity(2);
    actual.internal_kick_work=incoming;
    actual.internal_drift_work=incoming;
    actual.maximum_absolute_strain=-0.;
    auto expected=actual;
    const auto view=f.input.Prepared(2);
    ASSERT_TRUE(g::serial::Measure(f.host->model,f.host->slab[0],f.host->slab[1],view,expected));
    ASSERT_TRUE(m::MeasureStaged(*f.host,view,actual,1));
    EXPECT_TRUE(b::SameDiagnostics(actual,expected));
    EXPECT_NE(f.host->assembly.measurement[0].kick_operand[0],0.);
  }
}
TEST(QbatMeasurementHost,FailedCandidateNeverReadsStaleTrialAndAlwaysOverwritesFlag) {
  Fixture f;
  ASSERT_FALSE(HasFailure());
  f.Stage();
  ASSERT_EQ(f.host->assembly.measurement[0].valid,1);
  auto accepted=f.host->slab[0], trial=f.host->slab[1];
  accepted.element=nullptr;
  trial.element=nullptr;
  f.host->candidate_status[0]=q::Status::kInvalidInput;
  const auto next=m::PrepareMeasurementParent(*f.host,accepted,trial,f.input.Prepared(2),g::Identity(2),0);
  EXPECT_EQ(next.valid,0);
  for (double value : next.kick_operand) EXPECT_EQ(Bits(value),Bits(0.));
  f.Compare();
  EXPECT_EQ(f.host->assembly.measurement[0].valid,0);
  f.Reset();
  f.Compare();
  EXPECT_EQ(f.host->assembly.measurement[0].valid,1);
}
TEST(QbatMeasurementHost,UnequalParentAndNodeCountsKeepCompleteSourceOrder) {
  for (std::size_t parents : {1u,127u,128u,129u,257u,513u}) {
    SCOPED_TRACE(parents);
    Fixture f(parents);
    ASSERT_FALSE(HasFailure());
    f.Compare();
    EXPECT_EQ(f.host->control.status,q::BatchStatus::Success);
    f.host->slab[1].element[parents-1].point[3].force_volume_m3=std::numeric_limits<double>::quiet_NaN();
    f.Compare();
    EXPECT_EQ(f.host->control.status,q::BatchStatus::NonfiniteResult);
  }
}
TEST(QbatMeasurementHost,TabulatedRateAndEveryPartialSurfaceMaskKeepCompleteValidation) {
  Fixture f;
  ASSERT_FALSE(HasFailure());
  const double strain[]{0,.2,1,3}, stress[]{10e6,11e6,13e6,17e6};
  for (bool tabulated : {false,true}) {
    for (unsigned mask=0; mask<16; ++mask) {
      SCOPED_TRACE(::testing::Message()<<tabulated<<":"<<mask);
      f.Reset(0);
      for (unsigned parent=0; parent<4; ++parent) {
        auto& element=f.host->model.element[parent];
        if (tabulated) {
          ASSERT_EQ(tl::material::PrepareTabulatedShellPlasticity(250e6,.35,1000,
              {strain,stress,4},{true,8000,8,10000},element.material),
              tl::material::TabulatedShellPlasticityStatus::Ok);
        }
        auto& accepted=f.host->slab[0].element[parent];
        ASSERT_EQ(b::InitializeResult(element,accepted),q::Status::kSuccess);
        for (unsigned point=0; point<4; ++point) {
          if (!(mask&(1u<<point))) continue;
          auto& history=accepted.history.point[point];
          history.surface_active=history.failure.point_active=false;
          history.failure.damage=1;
        }
        accepted.history.element_active=mask!=15;
        qbat_force_test::Fixture path(false);
        path.reference=element.reference;
        path.input=element.reference.input();
        ASSERT_EQ(b::Advance(element,accepted,qbat_force_test::Path(path,0),
            f.host->slab[1].element[parent]),q::Status::kSuccess);
      }
      f.Compare(0);
      EXPECT_EQ(f.host->control.status,q::BatchStatus::Success);
    }
  }
}
} // namespace qbat_measurement_test
