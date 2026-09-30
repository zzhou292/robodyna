#pragma once
#include "modelio/source_assembly/SourceAssemblyData.h"
#include "output/full_shell/static_bundle/Types.h"
#include <memory>

namespace crash::modelio::tied_shell {
namespace source = output::full_shell::source;
using SourceId = assembly::SourceId;

struct Limits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t member_bytes = 64 * 1024 * 1024;
    std::size_t metadata_bytes = 8 * 1024 * 1024;
    std::size_t masters = 524288, slave_elements = 65536, slave_nodes = 65536;
    std::size_t parts = 1024, groups = 1024, group_members = 32768, blocks = 16384;
};
enum class ElementFamily { Shell, Beam, Solid };
enum class NativeReadiness { Unresolved };
struct SourceEvidence {
    assembly::SourceBlock block;
    // Exact noncomment data cards, including blank cards and source lines.
    std::vector<std::pair<std::size_t, std::string>> cards;
};
struct Part {
    SourceId id = 0, material_id = 0, section_id = 0;
    std::size_t part_source = 0, section_source = 0, material_source = 0;
    bool master = false, slave = false;
    std::size_t shells = 0, beams = 0, solids = 0;
};
struct Element {
    SourceId id = 0;
    std::uint32_t canonical_index = 0, part_index = 0;
    ElementFamily family = ElementFamily::Shell;
    unsigned arity = 0;
};
struct Incidence {
    std::uint32_t canonical_node = 0, element = 0;
    unsigned local_slot = 0;
};
struct Node {
    SourceId id = 0;
    std::uint32_t canonical_index = 0;
    bool master = false;
    std::array<std::int32_t, 2> source_codes{};
};
struct GroupEvidence {
    SourceId id = 0, node_set_id = 0;
    std::size_t source = 0, node_set_source = 0;
    std::vector<SourceId> source_nodes, slave_nodes, master_nodes;
};
struct UnresolvedBlock {
    std::string filename, keyword, sha256;
    std::size_t first_line = 0, last_line = 0;
    // Present only for selected original-member constraint evidence.
    std::size_t retained_source = SIZE_MAX;
};
struct Counts {
    std::size_t masters = 0, q4 = 0, t3 = 0, master_nodes = 0;
    std::size_t slave_shells = 0, slave_beams = 0, slave_solids = 0;
    std::size_t slave_nodes = 0, incidences = 0, shared_nodes = 0;
    std::size_t groups = 0, groups_touching_slaves = 0, groups_touching_masters = 0;
};
struct Data {
    SourceId slave_set_id = 0, master_set_id = 0;
    std::size_t contact_source = 0, slave_set_source = 0, master_set_source = 0;
    std::vector<SourceId> slave_part_ids, master_part_ids; // Original card order.
    std::vector<SourceEvidence> sources;                 // Original line order.
    std::vector<Part> parts;                             // Ascending original PID.
    std::vector<Element> masters;                        // Canonical shell order.
    std::vector<std::uint32_t> master_nodes;              // Ascending NID; canonical indices only.
    std::vector<Element> slaves;                         // Shell, beam, solid; source order within each.
    std::vector<Incidence> incidences;                    // Element then original local-slot order.
    std::vector<Node> slave_nodes;                       // Ascending original NID, not native NSV.
    std::vector<GroupEvidence> groups;                   // Original main-member declaration order.
    std::vector<UnresolvedBlock> unresolved_constraints; // Includes auxiliary-file identities.
    NativeReadiness ordering = NativeReadiness::Unresolved;
    NativeReadiness classification = NativeReadiness::Unresolved;
    NativeReadiness search = NativeReadiness::Unresolved;
    Counts counts;
    std::size_t startup_budget_bytes = 0, owned_payload_bytes = 0;
};

// Source-only declaration. No native NSV permutation, pair, constraint mask,
// mass/inertia, force, runtime admission, or solver clock is constructed.
class TiedShellDeclaration {
  public:
    static TiedShellDeclaration Prepare(const source::CanonicalSource&,
                                       const std::string& original_member, Limits = {});
    static std::size_t Forecast(const source::CanonicalSource&, Limits = {});
    TiedShellDeclaration(const TiedShellDeclaration&) noexcept = default;
    TiedShellDeclaration(TiedShellDeclaration&& other) noexcept : storage_(other.storage_) {}
    TiedShellDeclaration& operator=(const TiedShellDeclaration&) = delete;
    TiedShellDeclaration& operator=(TiedShellDeclaration&&) = delete;
    const source::CanonicalSource& canonical() const noexcept;
    const Data& data() const noexcept;
  private:
    struct Storage;
    explicit TiedShellDeclaration(std::shared_ptr<const Storage> storage)
        : storage_(std::move(storage)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::tied_shell
