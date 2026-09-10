#include "WallResponseJson.h"
#include "WallResponseCommand.h"
#include "WallResponseComparisonTestFixture.h"
#include <gtest/gtest.h>
#include <cstdlib>

namespace tl::qualification::qeph::wall_response::report_test {
namespace p=json::p;
const std::array<Run,3>& SyntheticRuns() {
  // Generated once; these are explicitly synthetic observation records, not
  // CUDA/native trajectories. Report tests reuse the owning comparison fixture.
  static const auto runs=comparison_test::Sequence(); return runs;
}
std::string Provenance() { return "{\n  \"scope\": \"synthetic wall-response report test only\"\n}\n"; }
ReportBinding Binding() {
  return {std::string(64,'a'),std::string(64,'b'),io::Sha256(Provenance())};
}
std::string Record(const Run& r) { return EncodeReport(DescribeRun(r,Binding(),Provenance())); }
struct Directory {
  std::filesystem::path path;
  Directory() {
    char name[]="/tmp/robo-dyna-wall-report-XXXXXX";
    const auto result=::mkdtemp(name); if(!result) throw std::runtime_error("Cannot create report test directory"); path=result;
  }
  ~Directory() { std::error_code error; std::filesystem::remove_all(path,error); }
};
TEST(QephWallResponseReport, ExactRunRoundTripKeepsPhasesCertificatesAndComparison) {
  const auto& source=SyntheticRuns(); std::array<::tl::qualification::qeph::wall_response::Run,3> reread;
  std::array<std::string,3> hashes;
  for(unsigned i=0;i<3;++i) {
    const auto bytes=Record(source[i]); hashes[i]=io::Sha256(bytes); ReportBinding binding;
    reread[i]=ReadRun(bytes,hashes[i],binding); EXPECT_TRUE(SameBinding(binding,Binding()));
    EXPECT_EQ(Record(reread[i]),bytes); EXPECT_TRUE(reread[i].completed);
    EXPECT_EQ(reread[i].samples.size(),SampleCount);
    EXPECT_EQ(reread[i].samples[1].carried_velocity_time,reread[i].samples[1].time-Step(reread[i].config)/2);
  }
  const auto comparison=DescribeComparison(reread,hashes,Binding());
  EXPECT_TRUE(comparison["broadside_refinement_passed"].GetBool());
  EXPECT_FALSE(comparison["simulation_ready"].GetBool());
}
TEST(QephWallResponseReport, RehashedCorruptionRejectsBeforePublishingBinding) {
  const auto original=Record(SyntheticRuns()[0]);
  for(unsigned fault=0;fault<10;++fault) {
    SCOPED_TRACE(fault); auto d=p::Parse(original);
    if(fault==0) d["qualification_id"].SetUint64(17);
    if(fault==1) d["model"]["nodal_mass_kg"][0].SetDouble(1);
    if(fault==2) d["model"]["fields"][0]["scale"].SetDouble(1);
    if(fault==3) d["common_endpoint_samples"][1]["carried_velocity_time_s"].SetDouble(0);
    if(fault==4) d["common_endpoint_samples"][1]["kinetic_error_J"][0].SetDouble(-1);
    if(fault==5) d["common_endpoint_samples"][1]["contact"]["candidate"].SetBool(false);
    if(fault==6) d["all_endpoint_summary"]["peak"]["observed"].SetBool(false);
    if(fault==7) d["provenance_exact_bytes"].SetString("{}",d.GetAllocator());
    if(fault==8) d["last_accepted_endpoint"]["endpoint_native_force_error_N"][0].SetDouble(1);
    if(fault==9) d.AddMember("completed",true,d.GetAllocator());
    const auto changed=EncodeReport(d); ReportBinding binding{std::string(64,'1'),std::string(64,'2'),std::string(64,'3')};
    const auto before=binding;
    EXPECT_THROW(ReadRun(changed,io::Sha256(changed),binding),std::exception);
    EXPECT_TRUE(SameBinding(binding,before));
  }
  ReportBinding b;
  EXPECT_THROW(ReadRun(original+" ",io::Sha256(original),b),std::exception);
  EXPECT_THROW(ReadRun("{bad",io::Sha256("{bad"),b),std::exception);
}
TEST(QephWallResponseReport, InitializationFailureAndRetainedFailedRunRemainUnadmitted) {
  ::tl::qualification::qeph::wall_response::Run empty;
  empty.config={1,1,H0,std::string(64,'a')}; std::string error;
  ASSERT_TRUE(BuildModel(1,empty.model,error))<<error; empty.failure="Synthetic initialization failure";
  const auto bytes=Record(empty); ReportBinding binding;
  const auto partial=ReadRun(bytes,io::Sha256(bytes),binding);
  EXPECT_TRUE(partial.samples.empty()); EXPECT_FALSE(partial.completed);
  const auto prefix=comparison_test::Synthetic(1,1,.008,200);
  const auto prefix_bytes=Record(prefix); const auto copied=ReadRun(prefix_bytes,io::Sha256(prefix_bytes),binding);
  EXPECT_EQ(copied.accepted_steps,200u); EXPECT_EQ(copied.last_accepted.epoch,200u); EXPECT_FALSE(copied.completed);
  auto failed=SyntheticRuns(); failed[0].completed=false; failed[0].failure="Synthetic final admission stop";
  const auto retained=Record(failed[0]); failed[0]=ReadRun(retained,io::Sha256(retained),binding);
  EXPECT_EQ(failed[0].accepted_steps,Steps(failed[0].config)); EXPECT_FALSE(Compare(failed).passed);
}
TEST(QephWallResponseReport, ExistingDestinationAndWrongProvenancePreserveEvidence) {
  Directory directory; const auto& run=SyntheticRuns()[0]; const auto bytes=Record(run);
  io::WriteBytes(directory.path/"run.json",bytes);
  command::Launch launch; launch.binding=Binding(); launch.provenance_bytes=Provenance(); launch.selected_h=H0;
  EXPECT_THROW(command::WriteRunRecord(directory.path,run,launch),std::exception);
  EXPECT_EQ(io::ReadBounded(directory.path/"run.json",ReportByteCap),bytes);
  EXPECT_THROW(DescribeRun(run,Binding(),Provenance()+" "),std::exception);
  auto wrong=Binding(); wrong.screen_index_sha256=std::string(64,'c');
  EXPECT_THROW(DescribeRun(run,wrong,Provenance()),std::exception);
}
} // namespace tl::qualification::qeph::wall_response::report_test
