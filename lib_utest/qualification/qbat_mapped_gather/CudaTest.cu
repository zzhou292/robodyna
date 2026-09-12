// SPDX-License-Identifier: MIT
#include "DeviceFixture.cuh"
#include "../qbat_resident/ResultValues.h"
namespace qbat_gather_test {
TEST(QbatMappedGatherCuda,VirginCarriedAndRemovedPacketsMatchFrozenSerialEightChannelBits) {
  DeviceFixture gather,reference;
  for(unsigned epoch=0;epoch<4;++epoch) for(unsigned mask=0;mask<(epoch?16u:1u);++mask) {
    SCOPED_TRACE(::testing::Message()<<epoch<<":"<<mask);
    gather.Upload(epoch);reference.Upload(epoch);
    for(auto* value:{&gather,&reference}) for(unsigned parent=0;parent<Parents;++parent) if(mask&(1u<<parent)) {
      Remove(value->storage->slab[0].element[parent]);
      ASSERT_TRUE(b::ValidResult(value->storage->slab[0].element[parent],value->storage->model.element[parent].material,epoch*Dt,epoch));
    }
    std::array<q::BatchResult,Parents> saved;
    std::memcpy(saved.data(),gather.storage->slab[0].element,sizeof(saved));
    gather.Assemble(epoch,false);reference.Assemble(epoch,true);
    ASSERT_EQ(gather.storage->control.status,q::BatchStatus::Success);
    SameControl(gather,reference);Compare(*gather.input,*reference.input);
    EXPECT_EQ(std::memcmp(saved.data(),gather.storage->slab[0].element,sizeof(saved)),0);
  }
}
TEST(QbatMappedGatherCuda,EarliestFailurePrecedenceWholeRollbackAndFreshInputRetry) {
  DeviceFixture gather,reference;
  for(unsigned fault=0;fault<7;++fault) {
    gather.Upload(1);reference.Upload(1);
    for(auto* value:{&gather,&reference}) {
      auto& input=*value->input;auto* rows=value->storage->slab[0].element;
      if(fault==0) rows[3].point[3].force_volume_m3=std::numeric_limits<double>::quiet_NaN();
      if(fault==1) input.orientation[4*5]=0;
      if(fault==2 || fault==3) {
        rows[2].internal_force_n[2].x=std::numeric_limits<double>::quiet_NaN();
        rows[0].internal_force_n[2].x=-std::numeric_limits<double>::max();input.values[0][0]=std::numeric_limits<double>::max();
        if(fault==3) input.orientation[4*2]=0;
      }
      if(fault==4) { rows[0].diagnostics.translation_stiffness_n_m=std::numeric_limits<double>::max();input.values[6][0]=std::numeric_limits<double>::max(); }
      if(fault==5) input.values[7][5]=std::numeric_limits<double>::quiet_NaN();
      if(fault==6) value->storage->model.element[3].nodes[3]=value->storage->model.element[3].nodes[2];
    }
    const auto before=*gather.input;
    gather.Assemble(1,false);reference.Assemble(1,true);
    EXPECT_NE(gather.storage->control.status,q::BatchStatus::Success);
    SameControl(gather,reference);Compare(*gather.input,before);
    gather.Upload(1);reference.Upload(1);
    gather.input->values[2][2]=reference.input->values[2][2]=.375;
    gather.Assemble(1,false);reference.Assemble(1,true);
    ASSERT_EQ(gather.storage->control.status,q::BatchStatus::Success);
    SameControl(gather,reference);Compare(*gather.input,*reference.input);
  }
}
TEST(QbatMappedGatherCuda,IndependentValidityAndWholeOwnerMaximumPreserveSerialDiagnosticBits) {
  DeviceFixture actual,reference;
  for(unsigned fault=0;fault<7;++fault) {
    actual.Upload(2,false);reference.Upload(2,false);
    for(auto* value:{&actual,&reference}) {
      auto* rows=value->storage->slab[1].element;
      if(fault==1) rows[3].point[3].force_volume_m3=std::numeric_limits<double>::quiet_NaN();
      if(fault==2) value->input->endpoint[3*(Nodes-1)]=std::numeric_limits<double>::quiet_NaN();
      if(fault==3) {value->input->endpoint[0]=std::numeric_limits<double>::quiet_NaN();value->input->endpoint[3*(Nodes-1)]=1e3;}
      if(fault==4) value->storage->candidate_status[2]=q::Status::kInvalidInput;
      if(fault==5) {
        rows[0].history.internal_work_j[0]=0x1p54;rows[1].history.internal_work_j[0]=1;
        rows[2].history.internal_work_j[0]=-0x1p54;rows[3].history.internal_work_j[0]=.25;
      }
      if(fault==6) {
        rows[0].history.internal_work_j[0]=std::numeric_limits<double>::max();
        rows[1].history.internal_work_j[0]=std::numeric_limits<double>::max();
      }
    }
    actual.Measure(2,false);reference.Measure(2,true);
    SameControl(actual,reference);
    EXPECT_TRUE(b::SameDiagnostics(actual.storage->control.diagnostics,reference.storage->control.diagnostics));
    actual.Upload(2,false);reference.Upload(2,false);
    actual.Measure(2,false);reference.Measure(2,true);
    ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
    EXPECT_TRUE(b::SameDiagnostics(actual.storage->control.diagnostics,reference.storage->control.diagnostics));
  }
}
TEST(QbatMappedGatherCuda,CompleteCandidateKernelKeepsCarriedCacheAndDiagnostics) {
  DeviceFixture actual,reference;
  for(unsigned epoch=0;epoch<8;++epoch) {
    actual.Upload(epoch,false);reference.Upload(epoch,false);
    std::array<q::BatchResult,Parents> saved;
    std::memcpy(saved.data(),actual.storage->slab[0].element,sizeof(saved));
    actual.Candidate(epoch,false);reference.Candidate(epoch,true);
    ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
    SameControl(actual,reference);
    EXPECT_TRUE(b::SameDiagnostics(actual.storage->control.diagnostics,reference.storage->control.diagnostics));
    for(unsigned parent=0;parent<Parents;++parent) {
      EXPECT_EQ(std::memcmp(&actual.storage->slab[0].element[parent],&saved[parent],sizeof(q::BatchResult)),0);
      EXPECT_EQ(qbat_resident_test::ResultValues(actual.storage->slab[1].element[parent]),
          qbat_resident_test::ResultValues(reference.storage->slab[1].element[parent]));
    }
  }
}
} // namespace qbat_gather_test
