#pragma once

#include "GuidedPlateStudy.h"
#include <filesystem>

namespace crash::case_data {
inline constexpr std::size_t kGuidedStudyByteCap=1024*1024;
// Completed numerical evidence, including explicitly failed physical outcomes.
// Create-only writes and bounded full-precision reads; malformed data throws.
// This report is an observer result, never a restart or accepted mesh archive.
// Required fields reject duplicate keys. Unknown extension fields are ignored.
void WriteGuidedPlateStudy(const std::filesystem::path&,const GuidedStudyData&);
GuidedStudyData ReadGuidedPlateStudy(const std::filesystem::path&);
GuidedStudyData ParseGuidedPlateStudy(const std::string& bounded_bytes);
void WriteGuidedPlateComparison(const std::filesystem::path&,const GuidedStudyComparison&,
                               const std::string& coarse_sha256,const std::string& fine_sha256);
} // namespace crash::case_data
