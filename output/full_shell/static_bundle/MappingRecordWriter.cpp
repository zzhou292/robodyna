#include "MappingRecords.h"
#include "MappingFields.h"
#include "InputChecks.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::output::full_shell::source {
MappingRecordPlan PlanMappingRecord(const PreparedSourceMapping& mapping, const std::string& stem, std::size_t cap) {
    arrays::CheckRelativeName(stem);
    Require(stem.size() <= 128 && stem.find('/') == std::string::npos && cap && cap <= StaticReserveBytes,
        "Invalid mapping record name/byte reservation");
    MappingRecordPlan plan;
    plan.files.reserve(9);
    for (std::size_t i = 0; i < plan.arrays.size(); ++i) {
        plan.arrays[i] = mapping.arrays()[i].descriptor;
        plan.arrays[i].file = stem + "-" + mapping.arrays()[i].name + ".bin";
        arrays::CheckRelativeName(plan.arrays[i].file);
        detail::AddBytes(plan.bytes, plan.arrays[i].bytes, cap);
        plan.files.push_back({plan.arrays[i].file, plan.arrays[i].bytes});
    }
    auto doc = detail::MappingDocument(mapping.source(), mapping.digest(), plan.arrays);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    Require(doc.Accept(writer) && buffer.GetSize() <= MappingMetadataByteCap,
        "Mapping descriptor exceeds metadata capacity");
    plan.metadata_bytes.assign(buffer.GetString(), buffer.GetSize());
    plan.description = {stem + ".mapping.json", Sha256(plan.metadata_bytes), plan.metadata_bytes.size()};
    detail::AddBytes(plan.bytes, plan.description.bytes, cap);
    plan.files.push_back({plan.description.file, plan.description.bytes});
    return plan;
}
RecordFile WriteMappingRecord(const std::filesystem::path& root, const PreparedSourceMapping& mapping,
        const std::string& stem, std::size_t cap) {
    const auto plan = PlanMappingRecord(mapping, stem, cap);
    for (const auto& file : plan.files) arrays::CheckedPath(root, file.file, false);
    const arrays::Limits limits{mapping.source().data().limits.file_bytes, UINT32_MAX, 64};
    for (std::size_t i = 0; i < plan.arrays.size(); ++i) {
        const auto& a = mapping.arrays()[i];
        arrays::WriteBytes(root, plan.arrays[i].file, a.descriptor.layout, a.bytes, limits);
    }
    WriteBytes(arrays::CheckedPath(root, plan.description.file, false), plan.metadata_bytes);
    return plan.description; // Complete mapping only; never a run-completion manifest.
}
} // namespace crash::output::full_shell::source
