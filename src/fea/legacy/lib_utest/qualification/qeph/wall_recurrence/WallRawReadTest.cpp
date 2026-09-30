#include "WallRawReadTestFixture.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace test=read_test;
namespace io=crash::output;
TEST(QephWallRawRead, CompleteSyntheticCollectionRetainsFailedChecksWithoutAdmission) {
  test::rt::Directory directory; const auto path=directory.path/"raw"; const auto receipt=test::Write(path,true);
  ASSERT_EQ(receipt.files.size(),65u); RawReadResult result; std::string error;
  ASSERT_TRUE(ReadRawJob(path,test::Binding(path),result,error))<<error;
  EXPECT_TRUE(result.job.collection_complete); EXPECT_TRUE(result.receipt.final_index_present);
  EXPECT_EQ(result.receipt.total_bytes,receipt.total_bytes); EXPECT_EQ(result.receipt.files.back().sha256,receipt.files.back().sha256);
  for(unsigned s=0;s<6;++s) {
    EXPECT_TRUE(CompleteRawStep(result.job.steps[s],1));
    for(const auto& p:result.job.steps[s].contact) {
      EXPECT_FALSE(p.passed); EXPECT_FALSE(p.complete); EXPECT_EQ(p.directions.size(),11u);
      EXPECT_TRUE(std::isinf(p.directions[0].quotients[0][0]));
      EXPECT_EQ(p.directions[0].completed_samples,3u);
    }
  }
  EXPECT_EQ(io::Bits(result.job.model.law().stiffness_per_area),io::Bits(test::rt::Fixture().model.law().stiffness_per_area));
  test::rt::CheckFiles(path,result.receipt);
}
TEST(QephWallRawRead, ClosedPartialNonfiniteColumnsRoundTripAndFailedReadsPreserveAllFields) {
  test::rt::Directory directory; const auto path=directory.path/"raw"; test::Write(path,false);
  RawReadResult out; std::string error; const auto binding=test::Binding(path);
  ASSERT_TRUE(ReadRawJob(path,binding,out,error))<<error;
  EXPECT_FALSE(out.job.collection_complete); EXPECT_TRUE(out.receipt.final_index_present);
  EXPECT_EQ(out.job.steps[0].native[0].derivative.completed_columns,1u);
  const auto nan=out.job.steps[0].native[0].derivative.full(0,1);
  EXPECT_TRUE(std::isnan(nan)); const auto saved=test::Snapshot(out);
  auto wrong=binding; wrong.provenance_sha256=std::string(64,'0');
  EXPECT_FALSE(ReadRawJob(path,wrong,out,error)); EXPECT_EQ(test::Snapshot(out),saved);
  wrong=binding; wrong.normal_velocity=8;
  EXPECT_FALSE(ReadRawJob(path,wrong,out,error)); EXPECT_EQ(test::Snapshot(out),saved);
  wrong=binding; wrong.remaining_screen_byte_budget=out.receipt.total_bytes-1;
  EXPECT_FALSE(ReadRawJob(path,wrong,out,error)); EXPECT_EQ(test::Snapshot(out),saved);
  io::WriteBytes(path/"uninventoried.json","{}");
  EXPECT_FALSE(ReadRawJob(path,binding,out,error)); EXPECT_EQ(test::Snapshot(out),saved);
}
TEST(QephWallRawRead, MissingIndexAndChangedBytesCannotBecomeClosedEvidence) {
  test::rt::Directory directory; const auto path=directory.path/"raw"; test::Write(path,false);
  RawReadResult out; std::string error; const auto binding=test::Binding(path);
  ASSERT_TRUE(ReadRawJob(path,binding,out,error)); const auto saved=test::Snapshot(out);
  const auto changed=directory.path/"changed";
  const auto changed_binding=test::Clone(path,changed,[](const std::string& name,io::Document& d) {
    if(name=="native-h0-a0.json") d["probe"]["full_matrix"]["values_rows"][0][0].SetDouble(17);
  });
  EXPECT_FALSE(ReadRawJob(changed,binding,out,error)); EXPECT_EQ(test::Snapshot(out),saved); // Old external index hash.
  ASSERT_TRUE(ReadRawJob(changed,changed_binding,out,error))<<error; // Rebound synthetic finite column is retained, not admitted.
  EXPECT_EQ(out.job.steps[0].native[0].derivative.full(0,0),17);
  const auto held=test::Snapshot(out); std::filesystem::remove(changed/"index.json");
  EXPECT_FALSE(ReadRawJob(changed,changed_binding,out,error)); EXPECT_EQ(test::Snapshot(out),held);
  const auto linked=directory.path/"linked";
  const auto linked_binding=test::Clone(path,linked,[](const std::string&,io::Document&){});
  std::filesystem::remove(linked/"provenance.json");
  std::filesystem::create_symlink(path/"provenance.json",linked/"provenance.json");
  EXPECT_FALSE(ReadRawJob(linked,linked_binding,out,error)); EXPECT_EQ(test::Snapshot(out),held);
}
TEST(QephWallRawRead, RehashedModelHeaderGridAndInventoriesStillNeedExactContract) {
  test::rt::Directory directory; const auto source=directory.path/"raw"; test::Write(source,false);
  RawReadResult out; std::string error; ASSERT_TRUE(ReadRawJob(source,test::Binding(source),out,error));
  const auto saved=test::Snapshot(out);
  for(unsigned mutation=0;mutation<7;++mutation) {
    SCOPED_TRACE(mutation);
    const auto path=directory.path/("mutation-"+std::to_string(mutation));
    const auto binding=test::Clone(source,path,[&](const std::string& name,io::Document& d) {
      if(mutation==0&&name=="model.json") {
        auto& m=d["model"]["assembled_native_mass_kg"][0]; m.SetDouble(std::nextafter(m.GetDouble(),INFINITY));
      }
      if(mutation==1&&name=="model.json") d["model"]["kappa_binary64_bits"].SetUint64(1);
      if(mutation==2&&name=="native-h0-a0.json") d["probe"]["fixed_dt_s"].SetDouble(recurrence::Steps[1]);
      if(mutation==3&&name=="native-h0-a0.json") d.AddMember("unknown",true,d.GetAllocator());
      if(mutation==4&&name=="progress-002.json") d["bytes_before_this_index"].SetUint64(1);
      if(mutation==5&&name=="index.json") d["collection_complete"].SetBool(true);
      if(mutation==6&&name=="native-h0-a0.json") {
        auto& row=d["probe"]["full_matrix"]["values_rows"][0]; row[0].CopyFrom(row[1],d.GetAllocator());
      }
    });
    EXPECT_FALSE(ReadRawJob(path,binding,out,error)); EXPECT_EQ(test::Snapshot(out),saved);
  }
}
TEST(QephWallRawRead, RehashedContactRowsDirectionsAndOperatorsCannotBypassBinding) {
  test::rt::Directory directory; const auto source=directory.path/"raw"; test::Write(source,false,true);
  RawReadResult out; std::string error; ASSERT_TRUE(ReadRawJob(source,test::Binding(source),out,error))<<error;
  EXPECT_FALSE(out.job.collection_complete); const auto saved=test::Snapshot(out);
  for(unsigned mutation=0;mutation<6;++mutation) {
    SCOPED_TRACE(mutation);
    const auto path=directory.path/("mutation-"+std::to_string(mutation));
    const auto binding=test::Clone(source,path,[&](const std::string& name,io::Document& d) {
      if(name!="contact-h0-b1.json") return; auto& p=d["probe"];
      if(mutation==0) p["directions"][0]["samples"][0]["nodes"][0]["source_row_diagnostic"]["attempt"].SetUint64(2);
      if(mutation==1) p["directions"][0]["direction"][0].SetDouble(.25);
      if(mutation==2) p["full_operator"]["values_rows"][0][0].SetDouble(7);
      if(mutation==3) p["directions"][0]["samples"][0]["nodes"][0]["force_n"]["error"].SetDouble(-1);
      if(mutation==4) p["directions"][0]["one_sided_quotients"][0][0]["binary64_bits"].SetUint64(io::Bits(1.));
      if(mutation==5) p["directions"][0]["samples"][0]["nodes"][0]["touching_or_penetrating"].SetBool(false);
    });
    EXPECT_FALSE(ReadRawJob(path,binding,out,error)); EXPECT_EQ(test::Snapshot(out),saved);
  }
}
} // namespace tl::qualification::qeph::wall_recurrence
