#pragma once
#include "ResponseData.h"
#include "output/ArtifactIO.h"

namespace tl::qualification::qeph::response {
namespace io=crash::output;
struct Binding { std::string decision_sha256,raw_sha256,runtime_sha256; };
// Required root-supplied decision fingerprint plus exact raw binding and frozen
// model/dictionary/grid/threshold checks. This is not issuer authentication.
Binding CheckAdmission(const std::string& decision,const std::string& raw,const io::Document& provenance);
io::Document DescribeRun(const Run&,const Binding&,const io::Document& provenance);
Run ReadRun(const std::string& bytes,Binding&);
io::Document DescribeComparison(const Comparison&,const std::array<Run,3>&,
                               const std::array<std::string,3>& report_hashes,const Binding&);
std::string Encode(const io::Document&);
} // namespace tl::qualification::qeph::response
