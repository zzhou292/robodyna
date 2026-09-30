#include "RecurrenceReport.h"
#include <gtest/gtest.h>
#include <unistd.h>

namespace tl::qualification::qeph::recurrence {
namespace io=crash::output;
namespace {
Audit UnresolvedFixture() {
  Audit result;
  for(unsigned c=0;c<2;++c) {
    Model m; std::string error;
    if(!BuildModel(c+1,m,error)) throw std::runtime_error(error);
    auto& fixture=result.cases[c]; fixture.elements=m.elements; fixture.nodes=m.nodes;
    fixture.dictionary=m.dictionary; fixture.complete=true;
    for(unsigned s=0;s<Steps.size();++s) {
      auto& step=fixture.steps[s]; step.h=Steps[s];
      for(unsigned a=0;a<Amplitudes.size();++a) {
        step.probes[a].amplitude=Amplitudes[a]; step.probes[a].full=Eigen::MatrixXd::Zero(m.dictionary.size(),m.dictionary.size());
      }
    }
  }
  auto& p=result.cases[0].steps[0].probes[0]; p.completed_columns=1; p.full(0,0)=-17;
  p.diagnostic="Deliberate late probe failure; remaining columns unevaluated";
  return result;
}
io::Document Provenance() { io::Document d; d.SetObject(); io::String(d,"test_inventory","synthetic; not original source authentication"); return d; }
struct Directory {
  std::filesystem::path path;
  Directory() { char pattern[]="/tmp/qeph-audit-report-XXXXXX"; const auto p=::mkdtemp(pattern);
    if(!p) throw std::runtime_error("Cannot create isolated test directory"); path=p; }
  ~Directory() { std::error_code ignored; std::filesystem::remove_all(path,ignored); }
};
}
TEST(QephRecurrenceReport, RejectedRawMatricesDictionaryAndProvenanceSurviveRoundtrip) {
  const auto result=UnresolvedFixture(); auto provenance=Provenance();
  const auto bytes=EncodeReport(DescribeAudit(result,provenance,true));
  io::Document parsed; parsed.Parse(bytes.c_str(),bytes.size()); ASSERT_FALSE(parsed.HasParseError());
  EXPECT_FALSE(parsed["audit_passed"].GetBool()); EXPECT_FALSE(parsed["simulation_ready"].GetBool());
  EXPECT_FALSE(parsed["trajectory_execution_qualified"].GetBool());
  const auto& first=parsed["cases"][0]["steps"][0]["probes"][0];
  EXPECT_FALSE(first["complete"].GetBool()); EXPECT_EQ(first["completed_columns"].GetUint64(),1u);
  EXPECT_EQ(first["full_matrix_rows"][0][0].GetDouble(),-17);
  EXPECT_EQ(parsed["cases"][0]["dictionary"].Size(),109u); EXPECT_EQ(parsed["cases"][1]["dictionary"].Size(),194u);
  const auto decision=DescribeAudit(result,provenance,false,io::Sha256(bytes),bytes.size());
  EXPECT_EQ(decision["raw_report_sha256"].GetString(),io::Sha256(bytes));
  EXPECT_EQ(decision["raw_report_bytes"].GetUint64(),bytes.size());
  EXPECT_FALSE(decision["cases"][0]["steps"][0]["probes"][0].HasMember("full_matrix_rows"));
}
TEST(QephRecurrenceReport, NonfiniteChangedGridAndUnqualifiedSelectionCannotPublishClaims) {
  auto result=UnresolvedFixture(); const auto provenance=Provenance();
  result.cases[0].steps[0].probes[0].full(0,0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(DescribeAudit(result,provenance,true),std::runtime_error);
  result=UnresolvedFixture(); result.cases[1].steps[2].h*=.75;
  EXPECT_THROW(DescribeAudit(result,provenance,true),std::runtime_error);
  result=UnresolvedFixture(); result.selected_h=H0; result.audit_passed=true;
  EXPECT_THROW(DescribeAudit(result,provenance,false,std::string(64,'a'),1),std::runtime_error);
  result=UnresolvedFixture();
  EXPECT_THROW(DescribeAudit(result,provenance,false,"",0),std::runtime_error);
}
TEST(QephRecurrenceReport, SharedCreateOnlyWriterPreservesExistingRawEvidence) {
  Directory directory; const auto path=directory.path/"raw-matrices.json";
  const auto bytes=EncodeReport(DescribeAudit(UnresolvedFixture(),Provenance(),true));
  io::WriteBytes(path,bytes);
  EXPECT_THROW(io::WriteBytes(path,"replacement"),std::runtime_error);
  EXPECT_EQ(io::ReadBounded(path,32u*1024*1024),bytes);
  EXPECT_EQ(io::Sha256(io::ReadBounded(path,32u*1024*1024)),io::Sha256(bytes));
}
} // namespace tl::qualification::qeph::recurrence
