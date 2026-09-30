#include "WallRawReportTestFixture.h"
#include <cmath>
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace rt=raw_test;
namespace io=crash::output;
TEST(QephWallRawReport, PartialColumnsAndNonfiniteBitsRoundTripWithExactInventoryBinding) {
  rt::Directory directory; const auto path=directory.path/"raw"; auto job=rt::Fixture();
  RawJobWriter writer(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  writer(job,{RawProgressKind::Model}); rt::Native(job,0,0,false);
  auto& p=job.steps[0].native[0].derivative;
  p.full(0,0)=-17; p.full(0,1)=std::numeric_limits<double>::quiet_NaN();
  writer(job,{RawProgressKind::NativeMatrix,0,0});
  const auto matrix=rt::Read(path/"native-h0-a0.json"); const auto& data=matrix["probe"]["full_matrix"];
  EXPECT_FALSE(matrix["screen_decision_included"].GetBool()); EXPECT_FALSE(matrix["simulation_ready"].GetBool());
  EXPECT_EQ(data["values_rows"][0][0].GetDouble(),-17);
  EXPECT_EQ(data["values_rows"][0][1]["nonfinite"].GetString(),std::string("nan"));
  EXPECT_EQ(data["values_rows"][0][1]["binary64_bits"].GetUint64(),io::Bits(p.full(0,1)));
  EXPECT_EQ(matrix["provenance_sha256"].GetString(),io::Sha256(rt::Provenance()));
  EXPECT_EQ(matrix["model_sha256"].GetString(),io::Sha256(io::ReadBounded(path/"model.json",RawFileByteCap)));
  const auto held=writer.receipt().total_bytes;
  EXPECT_THROW(writer(job,{RawProgressKind::NativeMatrix,0,0}),std::runtime_error);
  EXPECT_EQ(writer.receipt().total_bytes,held);
  p.complete=true; // A mutable in-memory flag cannot upgrade the saved partial matrix.
  EXPECT_THROW(writer(job,{RawProgressKind::Finished}),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path/"index.json")); p.complete=false;
  writer(job,{RawProgressKind::Finished});
  const auto final=rt::Read(path/"index.json"); EXPECT_FALSE(final["collection_complete"].GetBool());
  EXPECT_TRUE(final["final_index"].GetBool()); EXPECT_FALSE(final["screen_decision_included"].GetBool());
  rt::CheckFiles(path,writer.receipt());
}
TEST(QephWallRawReport, CompletedNonfiniteColumnsAndForeignIdentityRejectBeforePayload) {
  rt::Directory directory; const auto path=directory.path/"raw"; auto job=rt::Fixture();
  RawJobWriter writer(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  writer(job,{RawProgressKind::Model}); const auto held=writer.receipt().total_bytes;
  rt::Native(job,0,0,false); job.steps[0].native[0].derivative.full(0,0)=std::numeric_limits<double>::infinity();
  EXPECT_THROW(writer(job,{RawProgressKind::NativeMatrix,0,0}),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path/"native-h0-a0.json")); EXPECT_EQ(writer.receipt().total_bytes,held);
  rt::Native(job,0,0,true); job.steps[0].native[0].baseline[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(writer(job,{RawProgressKind::NativeMatrix,0,0}),std::runtime_error);
  EXPECT_EQ(writer.receipt().total_bytes,held);
  rt::Native(job,0,0,true); job.normal_velocity=8;
  EXPECT_THROW(writer(job,{RawProgressKind::NativeMatrix,0,0}),std::runtime_error);
  job.normal_velocity=0; job.steps[0].h*=.75;
  EXPECT_THROW(writer(job,{RawProgressKind::NativeMatrix,0,0}),std::runtime_error);
  job.steps[0].h=recurrence::Steps[0]; writer(job,{RawProgressKind::NativeMatrix,0,0});
  rt::CheckFiles(path,writer.receipt());
}
TEST(QephWallRawReport, ProvenancePrecisionCreateOnlyAndBudgetFailuresPreserveEarlierFiles) {
  const auto parsed=ParseRawProvenance(rt::Provenance());
  EXPECT_EQ(io::Bits(parsed["hard_double"].GetDouble()),io::Bits(std::nextafter(1.,0.)));
  EXPECT_EQ(parsed["large_id"].GetUint64(),18014398509481991ull);
  EXPECT_THROW(ParseRawProvenance("{\"a\":1,\"a\":2}"),std::runtime_error);
  EXPECT_THROW(ParseRawProvenance(std::string(ProvenanceByteCap+1,' ')),std::runtime_error);
  rt::Directory directory; auto job=rt::Fixture(); const auto path=directory.path/"full";
  RawJobWriter full(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap); full(job,{RawProgressKind::Model});
  const auto model_bytes=io::ReadBounded(path/"model.json",RawFileByteCap);
  EXPECT_THROW(RawJobWriter(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap),std::runtime_error);
  EXPECT_EQ(io::ReadBounded(path/"model.json",RawFileByteCap),model_bytes);
  const auto invalid=directory.path/"invalid";
  EXPECT_THROW(RawJobWriter(invalid,1,0,"[]","synthetic.json",ScreenSetByteCap),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(invalid));
  const auto bounded=directory.path/"bounded";
  RawJobWriter limited(bounded,1,0,rt::Provenance(),"synthetic.json",full.receipt().total_bytes+512);
  limited(job,{RawProgressKind::Model}); const auto held=limited.receipt().total_bytes;
  rt::Native(job,0,0,true);
  EXPECT_THROW(limited(job,{RawProgressKind::NativeMatrix,0,0}),std::runtime_error);
  EXPECT_EQ(limited.receipt().total_bytes,held); EXPECT_FALSE(std::filesystem::exists(bounded/"native-h0-a0.json"));
  EXPECT_FALSE(std::filesystem::exists(bounded/"index.json")); rt::CheckFiles(bounded,limited.receipt());
}
TEST(QephWallRawReport, SixJobBudgetIncludesProgressAndDerivedReportsWithoutAdmissionClaim) {
  std::array<RawJobReceipt,6> receipts;
  unsigned i=0;
  for(unsigned cells:{1u,2u}) for(double velocity:VelocityBaselines) {
    auto& r=receipts[i++]; r.cells=cells; r.normal_velocity=velocity;
    r.files={{"provenance.json",std::string(64,'a'),100},{"progress-000.json",std::string(64,'b'),200}};
    r.total_bytes=300; // Synthetic receipts test accounting only, not file authentication.
  }
  EXPECT_EQ(RawSetBytes(receipts),1800u);
  EXPECT_EQ(RawSetBytes(receipts,ScreenSetByteCap-1800),ScreenSetByteCap);
  EXPECT_THROW(RawSetBytes(receipts,ScreenSetByteCap-1799),std::runtime_error);
  auto malformed=receipts; malformed[1]=malformed[0];
  EXPECT_THROW(RawSetBytes(malformed),std::runtime_error);
  malformed=receipts; ++malformed[0].total_bytes;
  EXPECT_THROW(RawSetBytes(malformed),std::runtime_error);
  malformed=receipts; malformed[0].collection_complete=true;
  EXPECT_THROW(RawSetBytes(malformed),std::runtime_error);
}
} // namespace tl::qualification::qeph::wall_recurrence
