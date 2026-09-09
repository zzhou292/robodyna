#pragma once
#include "GuidedPlateStudy.h"
#include <filesystem>

namespace crash::case_data {
struct GuidedPlateWallStudyPaths {
    std::filesystem::path wall,derived_study,derived_provenance,canonical_study,canonical_provenance,report;
};
// Bounded exact-byte I/O + source/transform authentication. Both wall sidecars
// must match the same authenticated original and their respective Study bytes.
// The first role must be flip/subdivision; the second must be Original.
// Publishes a separate same-h comparison schema. Valid numerical failures
// write passed=false and return it; malformed inputs/outputs throw before any
// report write. No mechanics execution/restart or independent history proof.
GuidedStudyComparison CompareAndWriteGuidedPlateWallStudies(const GuidedPlateWallStudyPaths&);
} // namespace crash::case_data
