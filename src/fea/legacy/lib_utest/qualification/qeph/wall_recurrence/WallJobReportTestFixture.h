#pragma once
#include "WallJobReport.h"
#include "WallRawReportTestFixture.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence::report_test {
namespace rt=raw_test;
namespace io=crash::output;
struct Fixture {
  rt::Directory directory;
  RawReadResult raw;
  RawReadBinding binding;
  Fixture() {
    const auto path=directory.path/"raw"; auto job=rt::Fixture();
    RawJobWriter writer(path,1,0,rt::Provenance(),"synthetic-raw.json",ScreenSetByteCap);
    writer(job,{RawProgressKind::Model});
    for(unsigned a:{1u,2u}) { rt::Native(job,0,a,true); writer(job,{RawProgressKind::NativeMatrix,0,a}); }
    writer(job,{RawProgressKind::Finished});
    binding={1,0,io::Sha256(rt::Provenance()),writer.receipt().files.back().sha256,ScreenSetByteCap};
    std::string error; if(!ReadRawJob(path,binding,raw,error)) throw std::runtime_error(error);
  }
  WallJobAnalysis Job() const {
    WallJobAnalysis job; job.cells=1; job.dimension=109; job.input_valid=true;
    for(unsigned s=0;s<6;++s) {
      job.steps[s].h=recurrence::Steps[s]; job.steps[s].contact[1].branch=ContactBranch::Active;
      for(unsigned a=0;a<3;++a) {
        job.steps[s].amplitudes[a].amplitude=recurrence::Amplitudes[a];
        job.steps[s].amplitudes[a].attempted=raw.job.steps[s].native_attempted[a];
      }
    }
    job.diagnostic="Synthetic report evidence; no eigensolve or native recurrence admission"; return job;
  }
  void Prepare(WallJobAnalysis& job,unsigned amplitude) const {
    auto& a=job.steps[0].amplitudes[amplitude]; const auto& native=raw.job.steps[0].native[amplitude];
    a.input_complete=true; a.baseline=CheckWallMovingBaseline(raw.job.model,job.steps[0].h,native);
    auto& b=a.branches; b.h=job.steps[0].h; std::string error;
    if(!BuildWallStateMetric(raw.job.model,b.metric,error)) throw std::runtime_error(error);
    b.schedule=AnalyzeWallSwitchingSchedule(raw.job.model,b.h);
    for(unsigned i=0;i<2;++i) {
      auto& c=b.branches[i]; c.branch=i?ContactBranch::Active:ContactBranch::Inactive;
      if(!BuildContactBranch(raw.job.model,b.h,c.branch,native.derivative.full,c.full,error)||
         !ApplyWallStateMetric(c.full,b.metric.diagonal,c.weighted,error)) throw std::runtime_error(error);
    }
    // Retain a failed decomposition after actual operator construction. No
    // synthetic spectrum is presented as an admitted scientific result.
    b.branches[0].spectrum.matrix_norm=std::numeric_limits<double>::infinity();
    b.branches[0].spectrum.diagnostic="Deliberate failed partial scalar";
    a.diagnostic="Synthetic incomplete spectral evidence";
  }
};
inline void CheckFiles(const std::filesystem::path& path,const WallDerivedReceipt& receipt) {
  std::size_t total=0;
  for(const auto& f:receipt.files) {
    const auto bytes=io::ReadBounded(path/f.name,RawFileByteCap);
    EXPECT_EQ(bytes.size(),f.bytes); EXPECT_EQ(io::Sha256(bytes),f.sha256); total+=bytes.size();
  }
  EXPECT_EQ(receipt.total_bytes,total);
}
} // namespace tl::qualification::qeph::wall_recurrence::report_test
