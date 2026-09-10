#include "WallDerivedReadTestFixture.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace drt=derived_test;
namespace io=crash::output;
TEST(QephWallDerivedRead, ReconstructsSummaryWithoutUsingAggregateOrRawGainVerdicts) {
  drt::Fixture fixture; WallDerivedReadResult result; std::string error;
  ASSERT_TRUE(ReadWallJobSummary(fixture.derived,fixture.raw,fixture.binding,result,error))<<error;
  ASSERT_TRUE(result.summary_available); EXPECT_TRUE(result.receipt.analysis_complete);
  EXPECT_FALSE(result.receipt.analysis_passed); EXPECT_EQ(result.receipt.files.size(),87u);
  EXPECT_EQ(result.receipt.files.back().sha256,fixture.binding.analysis_index_sha256);
  EXPECT_EQ(result.summary.cells,1u); EXPECT_EQ(result.summary.dimension,109u);
  std::array<bool,6> passing{};
  for(unsigned s=0;s<6;++s) {
    const auto& step=result.summary.steps[s]; EXPECT_TRUE(step.complete); EXPECT_EQ(step.passed,s!=5);
    EXPECT_EQ(step.h,recurrence::Steps[s]); EXPECT_EQ(step.total_steps,static_cast<unsigned>(ScreenHorizon/step.h)); passing[s]=step.passed;
    for(const auto& a:step.amplitudes) for(const auto& g:a.gains) {
      EXPECT_TRUE(g.complete); EXPECT_EQ(g.raw,80); EXPECT_EQ(g.weighted,s==5?65:1);
    }
  }
  EXPECT_EQ(SelectWallScreenStep(passing),recurrence::H0);
  EXPECT_TRUE(ValidateWallJobSummary(result.summary,error))<<error;
}
TEST(QephWallDerivedRead, ClosedPartialAndFailedNonfiniteEvidenceStayUnavailable) {
  drt::Fixture fixture(true,true); WallDerivedReadResult result; std::string error;
  ASSERT_TRUE(ReadWallJobSummary(fixture.derived,fixture.raw,fixture.binding,result,error))<<error;
  EXPECT_FALSE(result.summary_available); EXPECT_TRUE(result.receipt.final_index_present);
  EXPECT_FALSE(result.receipt.analysis_complete); EXPECT_EQ(result.summary.dimension,0u);
  const auto saved=drt::rt::Read(fixture.derived/"amplitude-h0-a0.json");
  EXPECT_EQ(std::string(saved["analysis"]["branches"]["constant_branches"][0]["raw_spectrum"]["schur_residual"]["nonfinite"].GetString()),"nan");
  const auto held=drt::Snapshot(result); const auto corrupt=fixture.directory.path/"nonfinite-complete";
  const auto binding=drt::Clone(fixture.derived,corrupt,[](const std::string& name,io::Document& d) {
    if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][0]["raw_spectrum"]["complete"].SetBool(true);
  });
  EXPECT_FALSE(ReadWallJobSummary(corrupt,fixture.raw,binding,result,error)); EXPECT_EQ(drt::Snapshot(result),held);
}
TEST(QephWallDerivedRead, ExternalBindingsDirectoryKindsAndByteCapsPreserveOutput) {
  drt::Fixture fixture(true); WallDerivedReadResult result; std::string error;
  ASSERT_TRUE(ReadWallJobSummary(fixture.derived,fixture.raw,fixture.binding,result,error))<<error;
  const auto held=drt::Snapshot(result);
  auto reject=[&](const WallDerivedReadBinding& binding) {
    EXPECT_FALSE(ReadWallJobSummary(fixture.derived,fixture.raw,binding,result,error));
    EXPECT_FALSE(error.empty()); EXPECT_EQ(drt::Snapshot(result),held);
  };
  auto wrong=fixture.binding; wrong.analysis_index_sha256=std::string(64,'0'); reject(wrong);
  wrong=fixture.binding; wrong.analysis_provenance_sha256=std::string(64,'0'); reject(wrong);
  wrong=fixture.binding; wrong.remaining_set_bytes=result.receipt.total_bytes-1; reject(wrong);
  wrong=fixture.binding; wrong.remaining_set_bytes=ScreenSetByteCap+1; reject(wrong);
  io::WriteBytes(fixture.derived/"extra.json","{}"); reject(fixture.binding); std::filesystem::remove(fixture.derived/"extra.json");
  const auto payload=fixture.derived/"amplitude-h0-a0.json",held_file=fixture.directory.path/"held-payload.json";
  std::filesystem::rename(payload,held_file); reject(fixture.binding);
  std::filesystem::create_symlink(held_file,payload); reject(fixture.binding);
  std::filesystem::remove(payload); std::filesystem::rename(held_file,payload);
  std::filesystem::rename(fixture.derived/"index.json",fixture.directory.path/"held-index.json"); reject(fixture.binding);
  std::filesystem::rename(fixture.directory.path/"held-index.json",fixture.derived/"index.json");
  ASSERT_TRUE(ReadWallJobSummary(fixture.derived,fixture.raw,fixture.binding,result,error))<<error;
  EXPECT_EQ(drt::Snapshot(result),held);
}
TEST(QephWallDerivedRead, RehashedModelContextOperatorsAndPrefixMutationsAreRejected) {
  drt::Fixture fixture(true); WallDerivedReadResult result; std::string error;
  ASSERT_TRUE(ReadWallJobSummary(fixture.derived,fixture.raw,fixture.binding,result,error))<<error;
  const auto held=drt::Snapshot(result);
  const std::array<drt::Mutation,6> mutations{{
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["raw_model"]["sha256"].SetString(std::string(64,'0').c_str(),d.GetAllocator()); },
    [](const std::string& name,io::Document& d) { if(name=="context-h0.json") d["context"]["metric"]["normal_weight"].SetDouble(1); },
    [](const std::string& name,io::Document& d) { if(name=="context-h0.json") d["context"]["schedule"]["windows"][0]["active"].SetUint(1); },
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][1]["weighted_operator"]["sha256"].SetString(std::string(64,'0').c_str(),d.GetAllocator()); },
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["raw_native_matrix"]["sha256"].SetString(std::string(64,'0').c_str(),d.GetAllocator()); },
    [](const std::string& name,io::Document& d) { if(name=="progress-002.json") d["completed_amplitudes"].SetUint(0); }
  }};
  for(unsigned i=0;i<mutations.size();++i) {
    SCOPED_TRACE(i);
    const auto target=fixture.directory.path/("corrupt-"+std::to_string(i));
    const auto binding=drt::Clone(fixture.derived,target,mutations[i]);
    EXPECT_FALSE(ReadWallJobSummary(target,fixture.raw,binding,result,error));
    EXPECT_EQ(drt::Snapshot(result),held);
  }
}
TEST(QephWallDerivedRead, RechecksSpectrumGramContactAndComparisonMeasurements) {
  drt::Fixture fixture; WallDerivedReadResult result; std::string error;
  ASSERT_TRUE(ReadWallJobSummary(fixture.derived,fixture.raw,fixture.binding,result,error))<<error;
  const auto held=drt::Snapshot(result);
  const std::array<drt::Mutation,7> mutations{{
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][0]["raw_spectrum"]["spectral_radius"].SetDouble(.5); },
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][0]["raw_spectrum"]["near_one"].SetUint(0); },
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][0]["continuous_gram"]["weighted"]["mean_gain"].SetDouble(2); },
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][0]["continuous_gram"]["raw"]["passed"].SetBool(true); },
    [](const std::string& name,io::Document& d) { if(name=="amplitude-h0-a0.json") d["analysis"]["branches"]["constant_branches"][0]["continuous_gram"]["weighted"]["controlling_coordinate"].SetUint(0); },
    [](const std::string& name,io::Document& d) { if(name=="contact-h0-b1.json") d["analysis"]["directions"][0]["quotients"][0][0].SetDouble(123); },
    [](const std::string& name,io::Document& d) { if(name=="step-h0.json") d["analysis"]["derived_amplitude_comparisons"][0]["weighted_gains_gate"][0]["budget_lower"].SetDouble(123); }
  }};
  for(unsigned i=0;i<mutations.size();++i) {
    SCOPED_TRACE(i);
    const auto target=fixture.directory.path/("numeric-"+std::to_string(i));
    const auto binding=drt::Clone(fixture.derived,target,mutations[i]);
    EXPECT_FALSE(ReadWallJobSummary(target,fixture.raw,binding,result,error)); EXPECT_EQ(drt::Snapshot(result),held);
  }
}
} // namespace tl::qualification::qeph::wall_recurrence
