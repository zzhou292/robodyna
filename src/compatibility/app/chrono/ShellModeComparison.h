#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace crash::reference {

inline constexpr std::size_t kShellModeCapacity = 24;
inline constexpr std::size_t kShellTranslationCapacity = 12;
inline constexpr std::size_t kNoShellMode = kShellModeCapacity;
inline constexpr double kShellClusterRelativeGap = .01;
inline constexpr double kShellModeMinimumSquaredCosine = .99;
inline constexpr double kShellModeMaximumRelativeFrequencyChange = .05;
// Each column is normalized before the rank test. A smaller singular-value
// ratio does not admit a reliable translational subspace for this screen.
inline constexpr double kShellModeMinimumRankRatio = 1e-8;

enum class ShellModeComparisonStatus {
    kSuccess, kInvalidInput, kDimensionMismatch, kRankDeficient,
    kUnmatched, kAmbiguous, kFrequencyMismatch, kNumericalFailure
};

// Already selected physical displacement projections, not eigenvectors in two
// different mass-normalized full-coordinate systems. Frequencies use any one
// common unit. Rows name modes, columns the SAME translational coordinates in
// both sets. Cluster labels may be permuted, but must describe the connected
// groups of sorted adjacent frequencies with (next-previous)/previous <= 1%.
// This rule is frozen before the screen; no branch selection happens here.
struct ShellModeSet {
    std::size_t coordinate_count = 0;
    std::size_t mode_count = 0;
    std::array<double,kShellModeCapacity> frequency{};
    std::array<std::array<double,kShellTranslationCapacity>,kShellModeCapacity> translation{};
    std::array<std::uint32_t,kShellModeCapacity> cluster{};
};
using ShellTranslationMass = std::array<double,kShellTranslationCapacity>;
using ShellModeClusters = std::array<std::uint32_t,kShellModeCapacity>;

struct ShellModeClusterMatch {
    std::uint32_t reference_cluster = 0, candidate_cluster = 0;
    std::size_t mode_count = 0;
    // Members retain their input order. A multi-mode subspace has no unique
    // vector pairing; its frequency comparison uses sorted multisets only.
    std::array<std::size_t,kShellModeCapacity> reference_modes{},candidate_modes{};
    double minimum_squared_cosine = 0;
    double maximum_relative_frequency_change = 0;
};
struct ShellModeComparison {
    std::size_t mode_count = 0, cluster_count = 0;
    std::array<ShellModeClusterMatch,kShellModeCapacity> clusters{};
    // Only singleton matches are populated. Clustered modes hold kNoShellMode.
    std::array<std::size_t,kShellModeCapacity> singleton_candidate{};
    double minimum_squared_cosine = 0;
    double maximum_relative_frequency_change = 0;
};

// Positive finite frequencies only. Output labels are contiguous in increasing
// frequency order, independent of input order. Unused entries become zero.
// All operations stage their output; failure changes only diagnostic.
ShellModeComparisonStatus ClusterShellFrequencies(
    const std::array<double,kShellModeCapacity>& frequencies, std::size_t count,
    ShellModeClusters& output, std::string& diagnostic);

// One positive finite diagonal translational mass metric is shared by BOTH
// sets. Arbitrary per-mode scaling and sign do not change the comparison.
// Equal-dimensional clusters match by squared MAC (one mode) or the minimum
// squared principal-angle cosine (several modes), at least .99. More than one
// geometric match is diagnosed as ambiguous even if frequency could break the
// tie. After geometric identity, each matched cluster's sorted frequency
// multiset must change by at most 5%, relative to the reference frequencies.
// The candidate pool may contain extra eligible clusters. Every selected
// reference cluster must match uniquely and injectively; unused candidate
// clusters are allowed. Eligibility belongs to the calling formulation screen.
// Near-rank-deficient translational projections are rejected, not truncated.
// This is a bounded host numerical screen, not a shell formulation policy or
// a rigorous interval certificate of eigensolver error.
ShellModeComparisonStatus CompareShellModes(
    const ShellModeSet& reference, const ShellModeSet& candidate,
    const ShellTranslationMass& common_translational_mass,
    ShellModeComparison& output, std::string& diagnostic);

}  // namespace crash::reference
