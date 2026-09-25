#include "MappingExecution.h"
#include "InputChecks.h"
#include <cmath>

namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
constexpr const char* Profile = "native_a62_ordinary_law1_npt0";
constexpr const char* Revision = "a62b27e6baa555d222a580d6218867d0be4d70b5";
constexpr const char* Scope = "explicit_default_shell_controls_virgin_offg1_no_element_freeze_no_implicit_thermal_xfem_restart";
void U64(std::string& bytes, std::uint64_t value) {
    for (unsigned i = 0; i < 8; ++i) bytes.push_back(char((value >> (8 * i)) & 255));
}
void TextKey(std::string& bytes, const std::string& value) {
    U64(bytes, value.size());
    bytes += value;
}
} // namespace

void CheckMappingExecution(const MappingExecution& execution) {
    Require(execution.profile == MappingExecutionProfile::NativeA62OrdinaryLaw1 &&
        std::isfinite(execution.projection_working_length_m) && execution.projection_working_length_m > 0 &&
        std::isfinite(execution.coefficient_working_length_m) && execution.coefficient_working_length_m > 0 &&
        !execution.parts.empty() && execution.parts.size() <= 1024,
        "Invalid resolved mapping execution profile");
    std::uint64_t previous = 0;
    for (const auto& part : execution.parts) {
        Require(part.part > previous && part.material && part.section &&
            part.qeph <= 1048576 && part.t3 <= 1048576 && part.qeph + part.t3 > 0,
            "Invalid mapping execution PID coverage");
        previous = part.part;
    }
}

Document MappingExecutionDocument(const MappingExecution& execution) {
    CheckMappingExecution(execution);
    Document document;
    document.SetObject();
    String(document, "profile", Profile);
    String(document, "revision", Revision);
    String(document, "required_execution_scope", Scope);
    Integer(document, "source_nip", 3);
    Integer(document, "resolved_npt", 0);
    Integer(document, "ismstr", 2);
    Integer(document, "ithick", 1);
    Integer(document, "iplas", 1);
    Integer(document, "internal_drilling", 0);
    Number(document, "projection_working_length_m", execution.projection_working_length_m);
    Number(document, "coefficient_working_length_m", execution.coefficient_working_length_m);
    Value parts(rapidjson::kArrayType);
    for (const auto& part : execution.parts) {
        Document item;
        item.SetObject();
        Integer(item, "part_id", part.part);
        Integer(item, "material_id", part.material);
        Integer(item, "section_id", part.section);
        Integer(item, "qeph_parents", part.qeph);
        Integer(item, "t3_parents", part.t3);
        parts.PushBack(Value(item, document.GetAllocator()), document.GetAllocator());
    }
    document.AddMember("parts", parts, document.GetAllocator());
    return document;
}

MappingExecution ParseMappingExecution(const Value& value) {
    Keys(value, {"profile", "revision", "required_execution_scope", "source_nip", "resolved_npt", "ismstr", "ithick",
        "iplas", "internal_drilling", "projection_working_length_m", "coefficient_working_length_m", "parts"});
    Require(Text(value["profile"]) == Profile && Text(value["revision"]) == Revision &&
        Text(value["required_execution_scope"]) == Scope && UInt(value["source_nip"]) == 3 &&
        UInt(value["resolved_npt"]) == 0 && UInt(value["ismstr"]) == 2 && UInt(value["ithick"]) == 1 &&
        UInt(value["iplas"]) == 1 && UInt(value["internal_drilling"]) == 0,
        "Unknown native mapping execution semantics");
    MappingExecution execution;
    execution.profile = MappingExecutionProfile::NativeA62OrdinaryLaw1;
    execution.projection_working_length_m = Real(value["projection_working_length_m"]);
    execution.coefficient_working_length_m = Real(value["coefficient_working_length_m"]);
    const auto& parts = value["parts"];
    Require(parts.IsArray() && !parts.Empty() && parts.Size() <= 1024, "Mapping execution table exceeds cap");
    execution.parts.reserve(parts.Size());
    for (const auto& part : parts.GetArray()) {
        Keys(part, {"part_id", "material_id", "section_id", "qeph_parents", "t3_parents"});
        execution.parts.push_back({UInt(part["part_id"]), UInt(part["material_id"]), UInt(part["section_id"]),
            UInt(part["qeph_parents"]), UInt(part["t3_parents"])});
    }
    CheckMappingExecution(execution);
    return execution;
}

std::string MappingExecutionDigest(const std::string& array_digest, const MappingExecution& execution) {
    CheckMappingExecution(execution);
    arrays::CheckHash(array_digest);
    std::string bytes;
    TextKey(bytes, "robo_dyna.full_shell_source_mapping.digest.v2");
    TextKey(bytes, array_digest);
    TextKey(bytes, Profile);
    TextKey(bytes, Revision);
    TextKey(bytes, Scope);
    // Same explicit policy fields as the metadata document, in fixed order.
    for (const auto field : {3u, 0u, 2u, 1u, 1u, 0u}) U64(bytes, field);
    U64(bytes, Bits(execution.projection_working_length_m));
    U64(bytes, Bits(execution.coefficient_working_length_m));
    U64(bytes, execution.parts.size());
    for (const auto& part : execution.parts) {
        U64(bytes, part.part);
        U64(bytes, part.material);
        U64(bytes, part.section);
        U64(bytes, part.qeph);
        U64(bytes, part.t3);
    }
    return Sha256(bytes);
}
} // namespace crash::output::full_shell::source::detail
