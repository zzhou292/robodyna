#pragma once
#include "WallSelectionReport.h"
#include "WallBoostTestFixture.h"
#include "WallRawReportTestFixture.h"

namespace tl::qualification::qeph::wall_recurrence::selection_test {
namespace io=crash::output;
// Protocol-only inputs: explicit synthetic receipts, identity-map comparison
// fixtures and scalar summaries. These are not authenticated numerical runs.
struct Fixture {
  raw_test::Directory directory;
  boost_test::Set set=boost_test::CompleteSet();
  std::array<WallSelectionInput,6> inputs;
  std::array<RawJobReceipt,6> raw;
  std::array<WallDerivedReceipt,6> derived;
  std::array<bool,6> available{{true,true,true,true,true,true}};
  static RawFileReceipt File(unsigned slot,const std::string& name) {
    const auto bytes="synthetic input "+std::to_string(slot)+" "+name;
    return {name,io::Sha256(bytes),bytes.size()};
  }
  Fixture() {
    for(unsigned i=0;i<6;++i) {
      auto& r=raw[i]; auto& d=derived[i]; auto& p=inputs[i];
      p.cells=set.jobs[i].cells; p.normal_velocity=set.jobs[i].normal_velocity;
      p.raw_directory=directory.path/("raw-"+std::to_string(i)); p.derived_directory=directory.path/("derived-"+std::to_string(i));
      r.cells=d.cells=p.cells; r.normal_velocity=d.normal_velocity=p.normal_velocity;
      r.files={File(i,"provenance.json"),File(i,"index.json")};
      d.files={File(i+6,"provenance.json"),File(i+6,"index.json")};
      for(unsigned h=0;h<6;++h) {
        d.files.push_back(File(i+6,"context-h"+std::to_string(h)+".json"));
        for(unsigned a=0;a<3;++a) {
          r.files.push_back(File(i,"native-h"+std::to_string(h)+"-a"+std::to_string(a)+".json"));
          d.files.push_back(File(i+6,"amplitude-h"+std::to_string(h)+"-a"+std::to_string(a)+".json"));
        }
      }
      for(const auto& f:r.files) r.total_bytes+=f.bytes;
      for(const auto& f:d.files) d.total_bytes+=f.bytes;
      p.raw_bytes=r.total_bytes; p.derived_bytes=d.total_bytes;
      p.raw_provenance_sha256=d.raw_provenance_sha256=r.files[0].sha256;
      p.raw_index_sha256=d.raw_index_sha256=r.files[1].sha256;
      p.derived_provenance_sha256=d.analysis_provenance_sha256=d.files[0].sha256;
      p.derived_index_sha256=d.files[1].sha256;
      r.final_index_present=r.collection_complete=d.final_index_present=d.analysis_complete=d.analysis_passed=true;
    }
  }
  void Job(WallSelectionReportWriter& writer,unsigned i) const {
    RawReadResult r; r.receipt=raw[i];
    WallDerivedReadResult d; d.receipt=derived[i]; d.summary=set.jobs[i]; d.summary_available=available[i];
    writer.RecordJob(i,r,d);
  }
  void Record(WallSelectionReportWriter& writer) const {
    for(unsigned i=0;i<6;++i) {
      Job(writer,i);
      if(i%3) writer.RecordBoost(set.boosts[2*(i/3)+i%3-1]);
    }
  }
  WallScreenSelection Decision() const {
    auto jobs=set.jobs;
    for(unsigned i=0;i<6;++i) if(!available[i]) jobs[i]={};
    auto out=SelectWallScreen(jobs,set.boosts);
    if(!out.input_valid) for(unsigned h=0;h<6;++h) {
      for(unsigned i=0;i<6;++i) out.steps[h].jobs[i]=available[i]&&set.jobs[i].steps[h].complete&&set.jobs[i].steps[h].passed;
      for(unsigned b=0;b<4;++b) out.steps[h].boosts[b]=set.boosts[b].input_valid&&set.boosts[b].steps[h].complete&&set.boosts[b].steps[h].passed;
    }
    return out;
  }
};
inline void Files(const std::filesystem::path& path,const WallSelectionReceipt& receipt) {
  std::size_t bytes=0;
  for(const auto& f:receipt.files) {
    const auto data=io::ReadBounded(path/f.name,RawFileByteCap);
    EXPECT_EQ(data.size(),f.bytes); EXPECT_EQ(io::Sha256(data),f.sha256); bytes+=data.size();
  }
  EXPECT_EQ(bytes,receipt.total_bytes); EXPECT_LE(bytes,WallSelectionReportByteCap);
}
} // namespace tl::qualification::qeph::wall_recurrence::selection_test
