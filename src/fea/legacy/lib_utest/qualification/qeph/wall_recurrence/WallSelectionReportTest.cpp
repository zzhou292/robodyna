#include "WallSelectionReportTestFixture.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace st=selection_test;
namespace io=crash::output;
TEST(QephWallSelectionReport, CompleteSetRetainsFourFullComparisonsAndExactInventory) {
  st::Fixture f; const auto path=f.directory.path/"selection";
  WallSelectionReportWriter writer(path,f.inputs,raw_test::Provenance(),"synthetic-selection.json",ScreenSetByteCap);
  f.Record(writer); writer.Finish(f.Decision()); const auto& receipt=writer.receipt();
  EXPECT_EQ(receipt.files.size(),18u); EXPECT_TRUE(receipt.final_index_present); EXPECT_TRUE(receipt.decision_complete);
  EXPECT_TRUE(receipt.passed); EXPECT_EQ(receipt.selected_h,recurrence::H0); st::Files(path,receipt);
  const auto boost=raw_test::Read(path/"boost-c2-plus8.json");
  EXPECT_EQ(boost["zero_input"]["raw_index_sha256"].GetString(),f.inputs[3].raw_index_sha256);
  EXPECT_EQ(boost["boost_input"]["derived_index_sha256"].GetString(),f.inputs[5].derived_index_sha256);
  const auto& step=boost["comparison"]["steps"][5]; const auto& amplitude=step["amplitudes"][2];
  EXPECT_EQ(step["boost_context"]["sha256"].GetString(),st::Fixture::File(11,"context-h5.json").sha256);
  EXPECT_EQ(amplitude["boost_native"]["sha256"].GetString(),f.raw[5].files.back().sha256);
  EXPECT_EQ(amplitude["boost_derived"]["sha256"].GetString(),f.derived[5].files.back().sha256);
  const auto& source=f.set.boosts[3].steps[5].amplitudes[2];
  for(unsigned i=0;i<194;++i) {
    EXPECT_EQ(amplitude["zero_baseline"]["expected"][i].GetDouble(),source.baselines[0].expected[i]);
    EXPECT_EQ(amplitude["zero_baseline"]["residual"][i].GetDouble(),source.baselines[0].residual[i]);
    EXPECT_EQ(amplitude["boost_baseline"]["expected"][i].GetDouble(),source.baselines[1].expected[i]);
    EXPECT_EQ(amplitude["boost_baseline"]["residual"][i].GetDouble(),source.baselines[1].residual[i]);
  }
  EXPECT_EQ(amplitude["branch_matrices_inactive_active"].Size(),2u);
  EXPECT_EQ(amplitude["raw_gains_diagnostic"].Size(),11u); EXPECT_EQ(amplitude["weighted_gains_gate"].Size(),11u);
  for(unsigned i=0;i<11;++i) {
    EXPECT_EQ(amplitude["raw_gains_diagnostic"][i]["difference_upper"].GetDouble(),source.raw_gains[i].difference);
    EXPECT_EQ(amplitude["weighted_gains_gate"][i]["budget_lower"].GetDouble(),source.weighted_gains[i].budget);
  }
  const auto index=raw_test::Read(path/"index.json"); EXPECT_EQ(index["files"].Size(),17u);
  EXPECT_EQ(index["source_input_bytes"].GetUint64(),ValidateWallSelectionInputs(f.inputs));
  EXPECT_EQ(index["provenance_sha256"].GetString(),io::Sha256(raw_test::Provenance()));
  EXPECT_FALSE(index["trajectory_admitted"].GetBool());
}
TEST(QephWallSelectionReport, ScientificRejectionAndDiagnosticCoarseFailureAreCompleteDecisions) {
  st::Fixture f;
  for(unsigned rejected=0;rejected<2;++rejected) {
    for(auto& job:f.set.jobs) job.steps[5].passed=false;
    if(rejected) f.set.jobs[5].steps[0].passed=false;
    const auto path=f.directory.path/("selection-"+std::to_string(rejected));
    WallSelectionReportWriter writer(path,f.inputs,raw_test::Provenance(),"synthetic-selection.json",ScreenSetByteCap);
    f.Record(writer); writer.Finish(f.Decision());
    EXPECT_TRUE(writer.receipt().decision_complete); EXPECT_EQ(writer.receipt().passed,!rejected);
    EXPECT_EQ(writer.receipt().selected_h,rejected?0:recurrence::H0); st::Files(path,writer.receipt());
    const auto d=raw_test::Read(path/"selection.json");
    EXPECT_FALSE(d["decision"]["steps"][5]["passed"].GetBool());
    EXPECT_TRUE(d["decision"]["steps"][5]["diagnostic_only"].GetBool());
  }
}
TEST(QephWallSelectionReport, UnavailableSummaryRetainsPartialComparisonsAndNoSelectedStep) {
  st::Fixture f; f.available[0]=false;
  for(unsigned b=0;b<2;++b) {
    auto& comparison=f.set.boosts[b]; comparison={}; comparison.cells=1; comparison.dimension=109;
    comparison.normal_velocity=b?8:-8; comparison.diagnostic="Synthetic closed partial input";
    for(unsigned h=0;h<6;++h) comparison.steps[h].h=recurrence::Steps[h];
    comparison.steps[5].amplitudes[2].lift_residual_max=std::numeric_limits<double>::infinity();
  }
  const auto path=f.directory.path/"partial";
  WallSelectionReportWriter writer(path,f.inputs,raw_test::Provenance(),"synthetic-selection.json",ScreenSetByteCap);
  f.Record(writer); writer.Finish(f.Decision());
  EXPECT_TRUE(writer.receipt().final_index_present); EXPECT_FALSE(writer.receipt().decision_complete);
  EXPECT_FALSE(writer.receipt().passed); EXPECT_EQ(writer.receipt().selected_h,0); st::Files(path,writer.receipt());
  const auto boost=raw_test::Read(path/"boost-c1-minus8.json");
  EXPECT_EQ(boost["comparison"]["steps"][5]["amplitudes"][2]["lift_residual_max"]["nonfinite"].GetString(),std::string("positive-infinity"));
  const auto decision=raw_test::Read(path/"selection.json");
  EXPECT_FALSE(decision["decision"]["steps"][0]["jobs"][0].GetBool());
  EXPECT_TRUE(decision["decision"]["steps"][0]["jobs"][5].GetBool());
}
TEST(QephWallSelectionReport, BadBindingsLateChildFailuresAndCreateOnlyBudgetFailuresPreserveEvidence) {
  st::Fixture f; auto bad=f.inputs; bad[5]=bad[4]; const auto invalid=f.directory.path/"invalid";
  EXPECT_THROW(WallSelectionReportWriter(invalid,bad,raw_test::Provenance(),"synthetic.json",ScreenSetByteCap),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(invalid));
  const auto path=f.directory.path/"selection";
  WallSelectionReportWriter writer(path,f.inputs,raw_test::Provenance(),"synthetic.json",ScreenSetByteCap);
  const auto initial=writer.receipt().total_bytes; RawReadResult r; r.receipt=f.raw[0]; r.receipt.files[1].sha256=std::string(64,'0');
  WallDerivedReadResult d; d.receipt=f.derived[0]; d.summary=f.set.jobs[0]; d.summary_available=true;
  EXPECT_THROW(writer.RecordJob(0,r,d),std::runtime_error); EXPECT_EQ(writer.receipt().total_bytes,initial);
  f.Job(writer,0); f.Job(writer,1); const auto held=writer.receipt().total_bytes;
  auto comparison=f.set.boosts[0]; auto& baseline=comparison.steps[5].amplitudes[2].baselines[0];
  baseline.complete=false; baseline.passed=false; // Parent still claims complete/pass.
  EXPECT_THROW(writer.RecordBoost(comparison),std::runtime_error);
  EXPECT_EQ(writer.receipt().total_bytes,held); EXPECT_FALSE(std::filesystem::exists(path/"boost-c1-minus8.json"));
  writer.RecordBoost(f.set.boosts[0]); // Clean retry after a pre-write semantic failure.
  f.Job(writer,2); const auto before_conflict=writer.receipt().total_bytes;
  io::WriteBytes(path/"boost-c1-plus8.json","retained conflict");
  EXPECT_THROW(writer.RecordBoost(f.set.boosts[1]),std::runtime_error);
  EXPECT_EQ(writer.receipt().total_bytes,before_conflict);
  EXPECT_EQ(io::ReadBounded(path/"boost-c1-plus8.json",100),"retained conflict");
  EXPECT_FALSE(std::filesystem::exists(path/"index.json")); st::Files(path,writer.receipt());
  const auto bounded=f.directory.path/"bounded";
  const auto cap=raw_test::Provenance().size()+1;
  EXPECT_THROW(WallSelectionReportWriter(bounded,f.inputs,raw_test::Provenance(),"synthetic.json",cap),std::runtime_error);
  EXPECT_TRUE(std::filesystem::exists(bounded/"provenance.json")); EXPECT_FALSE(std::filesystem::exists(bounded/"index.json"));
  EXPECT_LE(std::filesystem::file_size(bounded/"provenance.json"),cap);
}
} // namespace tl::qualification::qeph::wall_recurrence
