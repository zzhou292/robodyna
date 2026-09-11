#include "Internal.h"
namespace crash::cases::vehicle_startup::tied_assessment_detail {
std::size_t CanonicalPayload(const tied::source::CanonicalData& data, std::size_t cap) {
    std::size_t bytes = sizeof(data);
    const auto text = [&](const auto& value) { Add(bytes, value.capacity()+1, sizeof(value[0]), cap); };
    const auto record = [&](const output::full_shell::RecordFile& value) {
        text(value.file);
        text(value.sha256);
    };
    const auto& in = data.inputs;
    text(in.canonical_root.native());
    text(in.scope_root.native());
    text(in.member_root.native());
    record(in.canonical_manifest);
    record(in.scope_report);
    record(in.source_member);
    text(in.tire_policy);
    text(in.units.mass);
    text(in.units.length);
    text(in.units.time);
    text(data.canonical_bytes);
    text(data.scope_bytes);
    text(data.archive_sha256);
    text(data.retained_shell_ids_sha256);
    text(data.excluded_shell_ids_sha256);
    text(data.selected_node_ids_sha256);
    for (const auto& array : data.arrays) {
        text(array.name);
        text(array.bytes);
        text(array.descriptor.file);
        text(array.descriptor.sha256);
        Add(bytes, array.descriptor.layout.fields.capacity(), sizeof(std::string), cap);
        for (const auto& field : array.descriptor.layout.fields) text(field);
    }
    Add(bytes, data.parts.capacity(), sizeof(tied::source::PartDeclaration), cap);
    Add(bytes, data.selected_parts.capacity(), sizeof(std::uint64_t), cap);
    Add(bytes, data.excluded_parts.capacity(), sizeof(std::uint64_t), cap);
    return bytes;
}
} // namespace crash::cases::vehicle_startup::tied_assessment_detail
