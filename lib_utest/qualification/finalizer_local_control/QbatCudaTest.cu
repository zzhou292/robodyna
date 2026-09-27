// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatSupport.h"
#include "lib_utest/qualification/qbat_measurement_operands/DeviceFixture.cuh"
namespace final_control_test::qbat {
__global__ void FrozenLeaf(b::Storage* state,tl::fea::NodalPreparedView view,q::BatchDiagnostics identity) {
  m::control2733::FinalizeMeasurement(*state,view,identity,1);
}
__global__ void CurrentLeaf(b::Storage* state,tl::fea::NodalPreparedView view,q::BatchDiagnostics identity) {
  m::FinalizeMeasurement(*state,view,identity,1);
}
inline void Drain() {ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);}
TEST(QbatLocalControlCuda, ActualMappedCallerRetainsSeededControlsAndFullSerialReference) {
  f::DeviceFixture actual(129),expected(129);ASSERT_FALSE(HasFailure());
  for(unsigned fault=0;fault<=10;++fault)for(bool valid:{false,true}) {
    SCOPED_TRACE(::testing::Message()<<fault<<":"<<valid);
    for(auto* p:{&actual,&expected}) {
      p->source.Reset();f::Fault(p->source,fault);p->Restore();p->storage->control=Poison();
    }
    const auto identity=Seed(valid,0x1p54);
    b::LaunchMappedMeasurements(actual.storage,&actual.storage->slab[0],&actual.storage->slab[1],actual.input->Prepared(2),identity,f::g::Nodes);
    f::g::serial_candidate::FinalizeCandidate<<<1,1>>>(expected.storage,&expected.storage->slab[0],&expected.storage->slab[1],expected.input->Prepared(2),identity);
    Drain();ASSERT_FALSE(HasFailure());f::Same(actual,expected);
  }
}
TEST(QbatLocalControlCuda, Literal2733LeafPreservesEveryTerminalAndSameAllocationRepair) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());
  for(unsigned fault=0;fault<=10;++fault)for(bool valid:{false,true}) {
    rig.source.Reset();f::Fault(rig.source,fault);rig.source.Stage();rig.Restore();
    const auto identity=Seed(valid,-0.);const auto view=rig.input->Prepared(2);
    const auto bytes=rig.source.count*sizeof(q::BatchResult);
    std::vector<unsigned char> slabs[2];
    for(unsigned s=0;s<2;++s){slabs[s].resize(bytes);std::memcpy(slabs[s].data(),rig.storage->slab[s].element,bytes);}
    rig.storage->control=Poison();FrozenLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());
    const auto expected=rig.storage->control;rig.storage->control=Poison();
    CurrentLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());Same(rig.storage->control,expected);
    for(unsigned s=0;s<2;++s)EXPECT_EQ(std::memcmp(slabs[s].data(),rig.storage->slab[s].element,bytes),0);
  }
  rig.source.Reset();rig.source.Stage();rig.Restore();const auto view=rig.input->Prepared(2);const auto identity=Seed(true,.25);
  rig.storage->candidate_status[128]=q::Status::kInvalidInput;
  CurrentLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());
  ASSERT_EQ(rig.storage->control.status,q::BatchStatus::ElementFailure);
  // Repair only the bad source status in the same device allocation; retain
  // the failed Control so a reset/publication regression cannot hide in upload.
  rig.storage->candidate_status[128]=q::Status::kSuccess;
  CurrentLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());
  const auto repaired=rig.storage->control;ASSERT_EQ(repaired.status,q::BatchStatus::Success);
  FrozenLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());Same(repaired,rig.storage->control);
}
TEST(QbatLocalControlCuda, PrescanAvoidsPoisonedUnconsumedRowsAndFinalNaNRetainsIdentity) {
  f::DeviceFixture rig(129);ASSERT_FALSE(HasFailure());rig.source.Reset();rig.source.Stage();rig.Restore();
  auto identity=Seed(true,std::numeric_limits<double>::quiet_NaN());const auto view=rig.input->Prepared(2);
  CurrentLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());
  const auto nonfinite=rig.storage->control;EXPECT_EQ(nonfinite.status,q::BatchStatus::NonfiniteResult);
  FrozenLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());Same(nonfinite,rig.storage->control);
  rig.storage->candidate_status[128]=q::Status::kInvalidInput;
  const auto measurement=rig.storage->assembly.measurement;const auto maximum=rig.storage->assembly.maximum;
  rig.storage->assembly.measurement=nullptr;rig.storage->assembly.maximum=nullptr;
  CurrentLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());
  const auto failed=rig.storage->control;EXPECT_EQ(failed.status,q::BatchStatus::ElementFailure);
  FrozenLeaf<<<1,1>>>(rig.storage,view,identity);Drain();ASSERT_FALSE(HasFailure());Same(failed,rig.storage->control);
  rig.storage->assembly.measurement=measurement;rig.storage->assembly.maximum=maximum;
}
} // namespace final_control_test::qbat
