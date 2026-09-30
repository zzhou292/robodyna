#pragma once
#include "WallRawReport.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>

namespace tl::qualification::qeph::wall_recurrence::raw_test {
namespace io=crash::output;
inline std::string Provenance() {
  return "{\n  \"scope\":\"synthetic report fixtures; not qualified matrices\",\n"
         "  \"hard_double\":0.9999999999999999,\"large_id\":18014398509481991\n}\n";
}
struct Directory {
  std::filesystem::path path;
  Directory() { char pattern[]="/tmp/qeph-wall-raw-XXXXXX"; const auto p=::mkdtemp(pattern);
    if(!p) throw std::runtime_error("Cannot create isolated raw test directory"); path=p; }
  ~Directory() { std::error_code ignored; std::filesystem::remove_all(path,ignored); }
};
inline RawJob Fixture(unsigned cells=1,double velocity=0) {
  RawJob job; job.cells=cells; job.normal_velocity=velocity; std::string error;
  if(!BuildWallRecurrenceModel(cells,job.model,error)) throw std::runtime_error(error);
  for(unsigned s=0;s<6;++s) job.steps[s].h=recurrence::Steps[s]; return job;
}
inline void Native(RawJob& job,unsigned step,unsigned amplitude,bool complete) {
  const auto dimension=job.model.native().dictionary.size();
  auto& p=job.steps[step].native[amplitude]; job.steps[step].native_attempted[amplitude]=true;
  p.velocity={job.normal_velocity,0,0}; p.baseline_complete=true; p.baseline=Eigen::VectorXd::Zero(dimension);
  p.derivative.amplitude=recurrence::Amplitudes[amplitude]; p.derivative.full=Eigen::MatrixXd::Identity(dimension,dimension);
  p.derivative.complete=complete; p.derivative.completed_columns=complete?dimension:1;
  if(!complete) p.derivative.diagnostic="Deliberate partial synthetic matrix";
}
inline io::Document Read(const std::filesystem::path& path) {
  const auto bytes=io::ReadBounded(path,RawFileByteCap); io::Document d;
  d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
  if(d.HasParseError()) throw std::runtime_error("Test artifact is invalid JSON"); return d;
}
inline void CheckFiles(const std::filesystem::path& path,const RawJobReceipt& receipt) {
  std::size_t total=0;
  for(const auto& file:receipt.files) {
    const auto bytes=io::ReadBounded(path/file.name,RawFileByteCap);
    EXPECT_EQ(bytes.size(),file.bytes); EXPECT_EQ(io::Sha256(bytes),file.sha256); total+=bytes.size();
  }
  EXPECT_EQ(total,receipt.total_bytes);
}
} // namespace tl::qualification::qeph::wall_recurrence::raw_test
