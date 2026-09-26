#pragma once
#include "../PostGapmMainSource.h"
#include "../main_coefficients/SupportQuery.h"
#include "../main_coefficients/SolidSupportQuery.h"
#include "../main_coefficients/Compose.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
#include "lib_src/collision/RadiossType25MainGeometry.h"
#include "lib_src/collision/radioss_type25/NativeConstants.h"
#include "lib_src/math/ScalarBits.h"
#include "lib_utils/BoundedArena.h"
#include <stdexcept>
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
namespace old = main_coefficients::detail;
using output::Require;
struct Failure : std::runtime_error {
    Report report;
    explicit Failure(Report value) : std::runtime_error(value.reason), report(std::move(value)) {}
};
[[noreturn]] inline void Reject(Status status, const char* reason, std::size_t primary = SIZE_MAX,
    std::uint64_t first = 0, std::uint64_t second = 0) {
    throw Failure({status, reason, primary, first, second});
}
struct Values {
    std::vector<std::uint64_t> node_ids;
    std::vector<double> positions, coefficients;
    std::vector<s::PrimaryCornerPermutation> corners;
    std::vector<s::PreShellSolidSupport> before_shell;
    std::vector<s::PostGapmMainSupport> final_support;
    std::vector<PhysicalOwner> owners;
    std::vector<Geometry> geometry;
    std::vector<std::uint32_t> secondary_nodes, main_nodes;
    std::vector<double> secondary_gaps, main_node_gaps;
    std::vector<n::source_gaps::MainGapFields> main_gaps;
    n::source_gaps::Profile gap_profile;
    n::source_gaps::Report gap_report;
    Counts counts;
    s::SolidErosion incoming_erosion = s::SolidErosion::Unspecified;
    s::SolidErosion final_erosion = s::SolidErosion::Unspecified;
};
bool OrdinaryNativeProperty(const modelio::assembly::Material&, const modelio::assembly::Section&) noexcept;
void Check(const Mixed&, const GapOperands&, Limits);
Forecast Budget(const Mixed&, const GapOperands&, Limits);
void SourceNodeRosters(const coated::Inputs&, const std::vector<initial_surfaces::Face>&,
    std::vector<std::uint32_t>& secondary, std::vector<std::uint32_t>& main);
void Maps(const Mixed&, Values&);
void Resolve(const Mixed&, const old::Packed&, bool grouping, Values&);
std::vector<s::Main> OrientedMains(const s::MixedSidesSnapshot&, const std::vector<s::PrimaryCornerPermutation>&);
n::source_gaps::Profile SourceGapProfile() noexcept;
void Gaps(const Mixed&, const GapOperands&, Values&, Limits);
std::string Digest(const Mixed&, const GapOperands&, const Values&, std::size_t cap);
std::size_t RetainedValues(const Values&, std::size_t cap);
}
