#pragma once
#include "Types.h"
#include <memory>
namespace crash::modelio::native_spring_ids {
struct ContextData {
    Diagnostic diagnostic;
    Profile profile = Profile::Unknown;
    std::string entry_member, source_digest;
    std::vector<SourceRow> precursors; // All original SPRING precursors, including namespace-only rows.
    std::vector<SourceRow> welds, joints; // Original source identities, no generated native IDs yet.
    std::size_t non_spring_beams = 0;
    std::vector<tied_shell::SourceEvidence> evidence;
    std::vector<output::full_shell::RecordFile> members;
    Forecast forecast;
};
// Private construction binds a supported reader policy to authenticated closed
// source members. No caller-supplied boolean can assert complete namespace/order.
class ImportContext {
 public:
    static Forecast Preflight(const source::CanonicalSource&, const ImportMembers&, Limits = {});
    static ImportContext Prepare(const source::CanonicalSource&, const ImportMembers&, Limits = {});
    const source::CanonicalSource& canonical() const noexcept;
    const ContextData& data() const noexcept;
 private:
    struct Storage;
    explicit ImportContext(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::native_spring_ids
