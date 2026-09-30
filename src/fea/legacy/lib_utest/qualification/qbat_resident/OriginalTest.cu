// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"
#include "../qbat_catalog/OriginalCatalog.h"

namespace qbat_resident_test {
TEST(QbatResidentCuda, Original4250QuadsAndSingleT3CommitOnceAndRetainNativePointHistory) {
  qbat_catalog_test::SourceCatalog source;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(source.geometry.Input(),fe::ShellHostBindingLimits::Vehicle()).status,
      fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeFormulationCatalog(binding,source.Input(),fe::ShellPlasticityCatalogLimits::Vehicle()).status,
      fe::ShellPlasticityBindingStatus::Success);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,source.failures.data(),source.failures.size(),
      fe::ShellBatchFailureLimits::Vehicle()).status,fe::ShellPlasticityBindingStatus::Success);
  const fe::ShellFormulationScope scope{&binding,&catalog,&failure};
  Rig rig;
  ASSERT_TRUE(rig.Initialize(scope,true,true,0x1p-24));
  EXPECT_EQ(binding.qeph_count(),0u);
  EXPECT_EQ(binding.t3_source_id(0),2357656u);
  EXPECT_EQ(binding.qbat_count(),4250u);
  // A real source-size buffer can fit a complete diagnostic packet. Read-only
  // source views are not writable output storage, even through an explicit cast.
  ASSERT_GE(binding.node_count()*sizeof(fe::ShellBindingNode),sizeof(fe::ShellBatchDiagnostics));
  auto* alias=reinterpret_cast<fe::ShellBatchDiagnostics*>(
      const_cast<fe::ShellBindingNode*>(binding.nodes().data()));
  std::array<unsigned char,sizeof(fe::ShellBatchDiagnostics)> source_bytes;
  std::memcpy(source_bytes.data(),binding.nodes().data(),source_bytes.size());
  EXPECT_EQ(rig.publication.CopyAcceptedDiagnostics(rig.owner.accepted(),alias).status,
      fe::ShellPublicationStatus::InvalidInput);
  EXPECT_EQ(std::memcmp(source_bytes.data(),binding.nodes().data(),source_bytes.size()),0);
  const auto allocation=rig.qbat.allocations();
  EXPECT_EQ(allocation.device_allocations,1u);
  EXPECT_LT(allocation.device_bytes,33u*1024*1024);
  std::vector<qbat_force_test::NativeState> native;
  native.reserve(4250);
  for(std::size_t parent=0;parent<4250;++parent) {
    qbat_force_test::Fixture fixture;
    fixture.reference=binding.qbat_reference(parent);
    fixture.input=fixture.reference.input();
    ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qbat,parent,&fixture.material));
    fixture.failure=failure.parent(fe::ShellBindingFamily::Qbat,parent)->constant;
    native.emplace_back(fixture.Virgin().data());
  }
  for(unsigned step=0;step<3;++step) {
    Prepared prepared;
    ASSERT_TRUE(rig.Prepare(prepared));
    ASSERT_EQ(prepared.qbat.size(),4250u);
    for(std::size_t parent=0;parent<4250;++parent) {
      qbat_force_test::Fixture fixture;
      fixture.reference=binding.qbat_reference(parent);
      fixture.input=fixture.reference.input();
      ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qbat,parent,&fixture.material));
      fixture.failure=failure.parent(fe::ShellBindingFamily::Qbat,parent)->constant;
      native[parent].Step(fixture,Interval(rig,prepared,parent));
      ASSERT_FALSE(HasFailure())<<step<<' '<<parent;
      qbat_force_test::CompareNative(Restore(Element(fixture),prepared.qbat[parent]),native[parent]);
      ASSERT_FALSE(HasFailure())<<step<<' '<<parent;
    }
    if(step==0) {
      const fe::ShellFormulationCandidates candidates{nullptr,&prepared.diagnostics.t3,
          &prepared.diagnostics.qbat,nullptr};
      EXPECT_EQ(rig.publication.PrepareFormulations(rig.owner,prepared.token,candidates,alias).status,
          fe::ShellPublicationStatus::InvalidInput);
      EXPECT_EQ(std::memcmp(source_bytes.data(),binding.nodes().data(),source_bytes.size()),0);
      Prepared retry;
      ASSERT_TRUE(rig.Prepare(retry));
      Exact(prepared.qbat,retry.qbat);
      prepared=std::move(retry);
    }
    if(step==1) {
      Arm(ReadFault::LateNonfinite,4250);
      std::vector<qb::BatchResult> output=prepared.qbat;
      const auto before=Bytes(output.back());
      const auto rejected=rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,output.data(),output.size());
      EXPECT_EQ(rejected.status,qb::BatchStatus::NonfiniteResult);
      EXPECT_EQ(rejected.element,4249u);
      EXPECT_EQ(Bytes(output.back()),before);
      rig.Discard();
      Prepared retry;
      ASSERT_TRUE(rig.Prepare(retry));
      Exact(prepared.qbat,retry.qbat);
      prepared=std::move(retry);
    }
    ASSERT_TRUE(rig.Commit(prepared));
    EXPECT_EQ(rig.qbat.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(rig.qbat.allocations().device_allocations,allocation.device_allocations);
  }
  Fields accepted;
  std::vector<qb::BatchResult> rows;
  fe::ShellBatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Accepted(accepted,rows,diagnostics));
  EXPECT_EQ(accepted.stamp.epoch,3u);
  EXPECT_EQ(rows.back().stamp.sample_index,3u);
  EXPECT_EQ(diagnostics.qbat.active_count,4250u);
  fe::ShellBatchLayeredSection triangle;
  fe::t3::BatchDiagnostics triangle_diagnostics;
  ASSERT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(accepted.stamp,&triangle,1,&triangle_diagnostics).status,
      fe::t3::BatchStatus::Success);
  EXPECT_EQ(triangle.law(),fe::ShellSectionLaw::Law44Nip1);
  EXPECT_NE(triangle.one_point(),nullptr);
}
} // namespace qbat_resident_test
