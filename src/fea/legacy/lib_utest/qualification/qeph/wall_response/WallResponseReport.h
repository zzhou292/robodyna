#pragma once
#include "WallResponseComparison.h"
#include "output/ArtifactIO.h"

namespace tl::qualification::qeph::wall_response {
namespace io=crash::output;
constexpr std::size_t ReportByteCap=16u*1024*1024;
struct ReportBinding {
  std::string screen_index_sha256,runtime_sha256,provenance_sha256;
};
// The external launcher authenticates reviewed source/runtime provenance.
// A reader verifies the externally supplied exact report hash and rechecks
// retained numeric evidence; it does not rerun discarded intermediate states.
io::Document DescribeRun(const Run&,const ReportBinding&,const std::string& exact_provenance);
Run ReadRun(const std::string& bytes,const std::string& expected_sha256,ReportBinding&);
io::Document DescribeComparison(const std::array<Run,3>&,
                               const std::array<std::string,3>& hashes,const ReportBinding&);
std::string EncodeReport(const io::Document&);
bool SameBinding(const ReportBinding&,const ReportBinding&) noexcept;
} // namespace tl::qualification::qeph::wall_response
