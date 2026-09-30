#include "ResponseTestFixture.h"
#include "ResponseProtocol.h"
#include "../free_response/RecurrenceReport.h"
#include <cstdlib>

namespace response_test {
namespace io=crash::output;
namespace a=tl::qualification::qeph::recurrence;
namespace {
r::Binding Binding() { return {std::string(64,'a'),std::string(64,'b'),std::string(64,'c')}; }
io::Document Provenance() { io::Document d; d.SetObject(); io::String(d,"test_scope","Synthetic observer/report fixture, not a scientific trajectory"); return d; }
struct Temp {
  std::filesystem::path path;
  Temp() { char name[]="/tmp/qeph-response-report-XXXXXX"; const auto* p=mkdtemp(name); if(!p) throw std::runtime_error("mkdtemp failed"); path=p; }
  ~Temp() { std::error_code error; std::filesystem::remove_all(path,error); }
};
struct AuditFiles { std::string raw,decision; io::Document provenance; };
AuditFiles SyntheticAdmission() {
  // Artifact boundary fixture only. The root-supplied fingerprint is authority;
  // this test neither executes nor claims a native matrix qualification.
  a::Audit audit;
  for(unsigned c=0;c<2;++c) {
    a::Model model; std::string error; EXPECT_TRUE(a::BuildModel(c+1,model,error));
    auto& fixture=audit.cases[c]; fixture.elements=c+1; fixture.nodes=model.nodes;
    fixture.dictionary=model.dictionary; fixture.complete=true;
    for(unsigned s=0;s<a::Steps.size();++s) {
      auto& step=fixture.steps[s]; step.h=a::Steps[s];
      for(unsigned k=0;k<a::Amplitudes.size();++k) {
        auto& p=step.probes[k]; p.amplitude=a::Amplitudes[k]; p.complete=true; p.completed_columns=model.dictionary.size();
        p.full=Eigen::MatrixXd::Identity(model.dictionary.size(),model.dictionary.size());
      }
    }
  }
  auto provenance=Provenance(); AuditFiles files;
  files.raw=a::EncodeReport(a::DescribeAudit(audit,provenance,true));
  audit.audit_passed=true; audit.selected_h=a::H0;
  for(auto& fixture:audit.cases) for(auto& step:fixture.steps) {
    step.passed=true;
    for(auto& analysis:step.analysis) {
      analysis.complete=analysis.passed=true; analysis.spectral_radius=1; analysis.gram_gain=1;
      analysis.rigid_basis_condition=1;
      analysis.eigenvalues.assign(6*fixture.nodes+44*fixture.elements,{1.,0.});
    }
  }
  files.decision=a::EncodeReport(a::DescribeAudit(audit,provenance,false,io::Sha256(files.raw),files.raw.size()));
  files.provenance.SetObject(); io::String(files.provenance,"matrix_decision_sha256",io::Sha256(files.decision)); return files;
}
}
TEST(QephResponseReport, FullPrecisionRoundtripPreservesPhasesFieldsAndDistinctCompletionStage) {
  // This decimal lies on a difficult conversion path in default RapidJSON.
  const auto decimal=r::protocol::Parse("{\"value\":1.0000000000000002}");
  EXPECT_EQ(io::Bits(decimal["value"].GetDouble()),io::Bits(std::nextafter(1.,2.)));
  io::Document hard; hard.SetObject();
  const double adjacent=std::nextafter(0x1.a2b3c4d5e6f71p-127,INFINITY);
  io::Number(hard,"adjacent",adjacent);
  EXPECT_EQ(io::Bits(r::protocol::Parse(r::Encode(hard))["adjacent"].GetDouble()),io::Bits(adjacent));
  const auto input=Synthetic(2); auto provenance=Provenance();
  const auto bytes=r::Encode(r::DescribeRun(input,Binding(),provenance));
  r::Binding binding; const auto restored=r::ReadRun(bytes,binding);
  EXPECT_EQ(binding.runtime_sha256,Binding().runtime_sha256); EXPECT_EQ(restored.samples.size(),257u);
  EXPECT_EQ(restored.samples[71].values,input.samples[71].values);
  EXPECT_EQ(restored.samples[71].carried_kinetic,input.samples[71].carried_kinetic);
  EXPECT_EQ(restored.samples[71].synchronous_kinetic,input.samples[71].synchronous_kinetic);
  EXPECT_EQ(restored.samples[71].source_work,input.samples[71].source_work);
  EXPECT_EQ(restored.samples[71].carried_velocity_time,input.samples[71].carried_velocity_time);
  EXPECT_TRUE(restored.completed);
  const auto document=r::protocol::Parse(bytes);
  EXPECT_FALSE(document["refinement_admitted"].GetBool()); EXPECT_FALSE(document["simulation_ready"].GetBool());
  const std::array<r::Run,3> runs{{Synthetic(1),Synthetic(2),Synthetic(4)}};
  const auto comparison=r::Compare(runs); ASSERT_TRUE(comparison.passed);
  const auto encoded=r::Encode(r::DescribeComparison(comparison,runs,{std::string(64,'1'),std::string(64,'2'),std::string(64,'3')},Binding()));
  const auto summary=r::protocol::Parse(encoded); EXPECT_TRUE(summary["free_response_refinement_passed"].GetBool());
  EXPECT_FALSE(summary["simulation_ready"].GetBool());
}
TEST(QephResponseReport, MalformedPhaseMassSummaryKineticAndInitialPreloadRejectWithoutBindingPublication) {
  auto provenance=Provenance(); const auto run=Synthetic();
  const auto original=r::Encode(r::DescribeRun(run,Binding(),provenance));
  auto binding=Binding(); const auto unchanged=binding;
  for(unsigned fault=0;fault<9;++fault) {
    SCOPED_TRACE(fault);
    auto d=r::protocol::Parse(original);
    if(fault==0) d["qualification_id"].SetUint64(17);
    if(fault==1) d["common_endpoint_samples"][3]["carried_velocity_time_s"].SetDouble(0);
    if(fault==2) d["nodal_mass_kg"][0].SetDouble(1);
    if(fault==3) d["all_endpoint_small_response"]["displacement_over_side"].SetDouble(0);
    if(fault==4) {
      auto& sample=d["common_endpoint_samples"][4]; sample["synchronous_kinetic_J"][0].SetDouble(1);
      sample["energy_residual_J"].SetDouble(1+sample["energy_residual_J"].GetDouble()); d["maximum_abs_residual_J"].SetDouble(2);
    }
    if(fault==5) d["last_accepted_endpoint"]["external_work_J"].SetDouble(0);
    if(fault==6) d["common_endpoint_samples"][0]["values"][Field(run.fields,"element.material_stress[0][0]")].SetDouble(1);
    if(fault==7) d["fields"][0]["scale"].SetDouble(1);
    if(fault==8) d.AddMember("completed",true,d.GetAllocator());
    EXPECT_THROW(r::ReadRun(r::Encode(d),binding),std::exception);
    EXPECT_EQ(binding.decision_sha256,unchanged.decision_sha256); EXPECT_EQ(binding.runtime_sha256,unchanged.runtime_sha256);
  }
  EXPECT_THROW(r::ReadRun("{bad",binding),std::exception);
  auto incomplete=run; incomplete.completed=false; incomplete.failure="Intentional observer stop";
  const auto partial=r::ReadRun(r::Encode(r::DescribeRun(incomplete,Binding(),provenance)),binding);
  EXPECT_FALSE(partial.completed);
  EXPECT_FALSE(r::Compare({partial,Synthetic(2),Synthetic(4)}).passed);
}
TEST(QephResponseReport, MatrixAdmissionBindsFingerprintRawBytesDictionaryAndRequiredNumericalThresholds) {
  const auto source=SyntheticAdmission();
  const auto good=r::CheckAdmission(source.decision,source.raw,source.provenance);
  EXPECT_EQ(good.decision_sha256,io::Sha256(source.decision)); EXPECT_EQ(good.raw_sha256,io::Sha256(source.raw));
  EXPECT_THROW(r::CheckAdmission(source.decision,source.raw+" ",source.provenance),std::exception);
  auto wrong=r::protocol::Parse(source.decision); wrong["selected_candidate_dt_s"].SetDouble(r::H0/2);
  auto changed=r::Encode(wrong); io::Document pin; pin.SetObject(); io::String(pin,"matrix_decision_sha256",io::Sha256(changed));
  EXPECT_THROW(r::CheckAdmission(changed,source.raw,pin),std::exception);
  wrong=r::protocol::Parse(source.decision); wrong["cases"][0]["steps"][3]["analyses"][0]["gram_mean_gain"].SetDouble(65);
  changed=r::Encode(wrong); pin["matrix_decision_sha256"].SetString(io::Sha256(changed).c_str(),pin.GetAllocator());
  EXPECT_THROW(r::CheckAdmission(changed,source.raw,pin),std::exception);
  wrong=r::protocol::Parse(source.decision); wrong["cases"][1]["dictionary"][2]["scale"].SetDouble(2);
  changed=r::Encode(wrong); pin["matrix_decision_sha256"].SetString(io::Sha256(changed).c_str(),pin.GetAllocator());
  EXPECT_THROW(r::CheckAdmission(changed,source.raw,pin),std::exception);
}
TEST(QephResponseReport, ExistingArtifactDestinationRemainsUnchangedOnOutputConflict) {
  Temp directory; const auto path=directory.path/"run.json"; const auto run=Synthetic(); auto provenance=Provenance();
  const auto bytes=r::Encode(r::DescribeRun(run,Binding(),provenance)); io::WriteBytes(path,bytes);
  EXPECT_THROW(io::WriteBytes(path,"replacement"),std::exception);
  EXPECT_EQ(io::ReadBounded(path,r::FileCap),bytes);
  EXPECT_EQ(io::Sha256(io::ReadBounded(path,r::FileCap)),io::Sha256(bytes));
}
} // namespace response_test
