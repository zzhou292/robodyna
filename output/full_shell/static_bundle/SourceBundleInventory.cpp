#include "SourceBundleInternal.h"
#include <set>

namespace crash::output::full_shell::source::detail {
std::vector<RecordFile> Inventory(const PreparedSourceMapping& mapping,
        const MappingRecordPlan& plan, const BundleDescription& d) {
    std::vector<RecordFile> files{d.canonical_manifest, d.scope_report};
    for (const auto& a : mapping.source().data().arrays)
        files.push_back({a.descriptor.file, a.descriptor.sha256, a.descriptor.bytes});
    for (const auto& c : d.chunks) files.push_back(c.record);
    for (const auto& a : plan.arrays) files.push_back({a.file, a.sha256, a.bytes});
    files.push_back(plan.description);
    std::set<std::string> names;
    for (const auto& f : files) {
        arrays::CheckRelativeName(f.file);
        arrays::CheckHash(f.sha256);
        Require(f.bytes && f.bytes <= d.file_byte_cap && names.insert(f.file).second,
            "Duplicate or oversized static source file");
    }
    return files;
}
void CheckInventory(const std::vector<RecordFile>& actual, const std::vector<RecordFile>& expected) {
    Require(actual.size() == expected.size(), "Static source inventory count differs");
    for (std::size_t i = 0; i < actual.size(); ++i)
        Require(SameFile(actual[i], expected[i]), "Static source inventory differs from reconstructed authority");
}
} // namespace crash::output::full_shell::source::detail
