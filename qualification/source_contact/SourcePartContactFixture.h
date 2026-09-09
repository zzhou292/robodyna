#pragma once

#include "collision/Q4SurfaceMapping.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace crash::qualification::source_contact {
inline constexpr std::size_t NodeCount=117, ParentCount=94, Q4Count=88, T3Count=6;
inline constexpr std::uint64_t PartId=2000157;
inline constexpr const char* ReadinessSha256=
    "74e733b76a5c530c94ba39876ce3707df2ca8c602348204632eb902a48402d89";
inline constexpr std::size_t ReadinessBytes=671971;

struct SourceNodeBinding {
    std::uint64_t source_id=0;
    std::uint32_t canonical_index=0,source_line=0;
    std::array<std::int32_t,2> codes{};
    std::uint16_t blank_mask=0;
};
struct SourceParentBinding {
    std::uint64_t source_id=0;
    std::uint32_t canonical_index=0,source_line=0;
    std::array<std::uint64_t,6> raw_record{};
    std::array<std::uint32_t,4> canonical_node_indices{},local_node_indices{};
    std::uint16_t blank_mask=0;
    std::uint8_t arity=0;
};
enum class FixtureStatus { Ok,InvalidArgument,ReadFailure,HashMismatch,InvalidFixture };
struct FixtureReport { FixtureStatus status; std::string diagnostic; };
class SourcePartContactFixture;
FixtureReport LoadPinnedSourcePartContact(const std::filesystem::path&,SourcePartContactFixture*);

// Qualification input only: the exact immutable E2a report produced by the
// existing modelio source-card/array verifier. This is not a general model
// loader, source-formulation adapter, contact law or dynamics admission.
// All source order, raw fourth T3 slots, indices and node coordinate bits are
// retained. Loading/reloading publishes only after the complete pinned input
// has passed; every failure preserves the existing fixture.
class SourcePartContactFixture {
  public:
    bool prepared() const noexcept { return prepared_; }
    const auto& nodes() const noexcept { return nodes_; }
    const auto& parents() const noexcept { return parents_; }
    const auto& coordinates() const noexcept { return coordinates_; }
    tlfea::contact::VectorView positions() const noexcept {
        return prepared_?tlfea::contact::VectorView{coordinates_.data(),NodeCount,3,1}:
                         tlfea::contact::VectorView{};
    }
    // Typed parent construction preserves cyclic order. Source EID is also
    // the stable feature ID within this single-member qualification fixture;
    // this does not define the full vehicle's instance/feature namespace.
    // Wrong arity/index/unprepared input preserves the caller's parent.
    bool q4_parent(std::size_t index,tlfea::contact::SurfaceQ4&) const noexcept;
    bool t3_parent(std::size_t index,tlfea::contact::SurfaceTriangle&) const noexcept;
  private:
    std::array<double,3*NodeCount> coordinates_{};
    std::array<SourceNodeBinding,NodeCount> nodes_{};
    std::array<SourceParentBinding,ParentCount> parents_{};
    bool prepared_=false;
    friend FixtureReport LoadPinnedSourcePartContact(const std::filesystem::path&,SourcePartContactFixture*);
};
static_assert(sizeof(SourcePartContactFixture)<32*1024,"Revisit bounded qualification storage");
} // namespace crash::qualification::source_contact
