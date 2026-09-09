#pragma once

#include "ArtifactIO.h"
#include <cstdint>
#include <string_view>

namespace crash::output::guided_experiment_metadata {
// Named archive contract only. The reference model owns experiment creation;
// this solver-independent protocol neither selects nor executes mechanics.
inline constexpr const char* Field = "guided_experiment";
inline constexpr const char* Original = "original";
inline constexpr const char* PenaltyMargin = "penalty-margin-v1";
inline constexpr std::uint64_t OriginalQualification = 0x4432475549444531ULL;
inline constexpr std::uint64_t PenaltyMarginQualification = 0x4432475549444532ULL;
inline constexpr double MaximumPenetration = .0005;
inline constexpr double TargetPenetration = .000375;
inline constexpr double ForceError = 5e-7;
inline constexpr double PotentialError = 1.2500000000000005e-12;
inline constexpr unsigned MaxLeaves = 4096, MaxVisits = 16384, MaxDepth = 16;

inline bool Known(std::string_view name) {
    return name == Original || name == PenaltyMargin;
}
inline std::uint64_t Qualification(std::string_view name) {
    Require(Known(name), "Unknown guided experiment");
    return name == Original ? OriginalQualification : PenaltyMarginQualification;
}
inline double Penalty(std::string_view name) {
    Require(Known(name), "Unknown guided experiment");
    return name == Original ? 1e5 : 4e5;
}
inline std::string Name(const Value& object, bool required = false) {
    Require(object.IsObject(), "Guided experiment metadata must be an object");
    const Value* found = nullptr;
    for (const auto& member : object.GetObject()) {
        if (std::string_view(member.name.GetString(), member.name.GetStringLength()) != Field) continue;
        Require(!found, "Repeated guided experiment field");
        found = &member.value;
    }
    if (!found) {
        Require(!required, "Explicit guided experiment is missing");
        return Original;  // Original guided v1 records predate named variants.
    }
    Require(found->IsString(), "Guided experiment must be a string");
    std::string name(found->GetString(), found->GetStringLength());
    Require(Known(name), "Unknown guided experiment");
    return name;
}
} // namespace crash::output::guided_experiment_metadata
