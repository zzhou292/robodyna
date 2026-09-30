#pragma once

#include "modelio/tied_shell/TiedShellDeclaration.h"
#include <memory>
#include <optional>

namespace crash::modelio::self_contact {

namespace source = output::full_shell::source;
using SourceId = assembly::SourceId;

enum class OriginalSelectionProfile {
    AutomaticSingleSurfacePartSetV1,
};

struct Limits {
    std::size_t host_bytes = 256u << 20;
    std::size_t auxiliary_member_bytes = 1u << 20;
    std::size_t combine_member_bytes = 64u << 10;
    std::size_t metadata_bytes = 1u << 20;
    std::size_t blocks = 128;
    std::size_t parts = 1024;
};

struct OriginalContactFields {
    SourceId slave_set_id = 0;
    SourceId master_set_id = 0;
    unsigned slave_set_type = 0;
    std::optional<double> static_friction;
    std::optional<double> dynamic_friction;
    std::optional<double> decay_coefficient;
    std::optional<double> soft;
    std::optional<double> ignore_initial_penetration;
};

struct PartDisposition {
    SourceId part_id = 0;
    SourceId material_id = 0;
    SourceId section_id = 0;
    bool shell_section = false;
    bool retained_shell_part = false;
    bool excluded_shell_part = false;
    std::size_t shells = 0;
    std::size_t solids = 0;
    std::size_t beams = 0;
};

struct Counts {
    std::size_t selected_parts = 0;
    std::size_t shell_parts = 0;
    std::size_t retained_shell_parts = 0;
    std::size_t excluded_shell_parts = 0;
    std::size_t non_shell_parts = 0;
    std::size_t shells = 0;
    std::size_t retained_shells = 0;
    std::size_t excluded_shells = 0;
    std::size_t solids = 0;
    std::size_t beams = 0;
};

struct Data {
    OriginalSelectionProfile profile =
        OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1;
    std::string auxiliary_filename;
    std::string auxiliary_sha256;
    std::string combine_filename;
    std::string combine_sha256;
    OriginalContactFields source_fields;
    // Dependency order: contact card, root SET_PART_ADD, resolved part list.
    std::vector<tied_shell::SourceEvidence> sources;
    std::vector<SourceId> selected_part_ids;
    std::vector<PartDisposition> parts;
    Counts counts;
    std::size_t startup_budget_bytes = 0;
    std::size_t owned_payload_bytes = 0;
};

// Authenticated original contact-card selection only. Source friction/damping
// fields are retained as provenance and are not an executed contact law.
class OriginalSelection {
  public:
    static OriginalSelection Prepare(const source::CanonicalSource&,
        const std::string& auxiliary_member,
        const std::string& combine_member, Limits = {});
    static std::size_t Forecast(const source::CanonicalSource&,
        std::size_t auxiliary_member_bytes,
        std::size_t combine_member_bytes, Limits = {});
    OriginalSelection(const OriginalSelection&) noexcept = default;
    OriginalSelection(OriginalSelection&& other) noexcept
        : storage_(other.storage_) {}
    OriginalSelection& operator=(const OriginalSelection&) = delete;
    OriginalSelection& operator=(OriginalSelection&&) = delete;
    const source::CanonicalSource& canonical() const noexcept;
    const Data& data() const noexcept;

  private:
    struct Storage;
    explicit OriginalSelection(std::shared_ptr<const Storage> storage)
        : storage_(std::move(storage)) {}
    std::shared_ptr<const Storage> storage_;
};

}  // namespace crash::modelio::self_contact
