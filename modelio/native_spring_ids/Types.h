#pragma once
#include "modelio/tied_shell/TiedShellDeclaration.h"
#include <array>
#include <string_view>
namespace crash::modelio::native_spring_ids {
namespace source = output::full_shell::source;
enum class Profile { Unknown, DirectKeywordR14FreshRadiossPoSortById };
enum class Readiness { Ready, UnsupportedProfile, MissingSource, InvalidSource, UnsupportedSource, ResourceLimit, IdentityMismatch, Overflow };
enum class SourceKind { Type13, DiscreteNamespaceOnly, DefaultSpotweld, RegularJoint };
struct Diagnostic {
    Readiness status = Readiness::MissingSource;
    std::string reason, file;
    std::size_t line = 0;
    std::uint64_t source_id = 0;
};
struct Member {
    std::string filename;
    std::string_view bytes; // Borrowed only during context preparation.
};
struct ImportMembers {
    Profile profile = Profile::Unknown;
    std::string entry_member;
    std::vector<Member> members; // Complete authenticated canonical source_files closure.
};
struct Location {
    std::string file;
    std::size_t line = 0;
};
struct SourceRow {
    SourceKind kind = SourceKind::Type13;
    std::uint64_t original_id = 0;
    std::array<std::uint64_t, 2> endpoints{};
    // Context source rows remain unbound (SIZE_MAX). Successful Resolution rows:
    // Type13 -> SourceType13.data().beams; DefaultSpotweld -> TYPE25 model.connections();
    // RegularJoint -> VehicleType45Source.data().rows. Discrete rows stay unbound.
    std::size_t source_index = SIZE_MAX;
    std::size_t canonical_index = SIZE_MAX; // Separate original canonical provenance, when present.
    Location location;
    bool physical_participant = false;
};
struct Row : SourceRow {
    std::uint64_t native_id = 0;
};
struct Counts {
    std::size_t type13 = 0, discrete_namespace_only = 0, non_spring_beams = 0;
    std::size_t default_welds = 0, regular_joints = 0, physical = 0;
};
struct Limits {
    // Complete inherited source reservations may overlap; actual integer mapping
    // work has its separate small bound and never allocates a physical model.
    std::size_t host_bytes = std::size_t{8} << 30, member_bytes = 64u << 20;
    std::size_t resolve_bytes = 64u << 20;
    std::size_t metadata_bytes = 1u << 20, members = 8, blocks = 8192, rows = 16384;
};
struct Forecast {
    std::size_t retained_source = 0, member_reservation = 0, decode_scratch = 0;
    std::size_t result_reservation = 0, total_bytes = 0;
};
struct Resolution {
    Diagnostic diagnostic;
    Counts counts;
    std::uint64_t existing_maximum = 0, final_maximum = 0;
    std::vector<Row> rows; // Complete namespace in ascending native ID order.
    std::vector<std::uint32_t> physical_order; // Indices into rows; never includes omitted discrete mechanics.
    std::vector<std::uint32_t> source_order; // SourceKind/original-ID lookup order over rows.
    std::string source_digest, mapping_digest;
    const Row* Find(SourceKind, std::uint64_t original_id) const noexcept;
};
} // namespace crash::modelio::native_spring_ids
