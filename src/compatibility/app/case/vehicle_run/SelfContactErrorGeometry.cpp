#include "SelfContactErrorGeometry.h"

#include "lib_src/collision/SelfContactTransactionTypes.h"
#include "output/BoundedArrayJson.h"

#include <algorithm>
#include <cmath>

namespace crash::cases::vehicle_run::detail {
namespace {

using namespace output;
namespace contact = tlfea::contact;

// Keep exact binary64 payloads even for nonfinite failure diagnostics. Finite
// numbers are included for human inspection; bits are the lossless authority.
Document ReportedNumbers(const double* values, std::size_t count) {
    Document result;
    result.SetObject();
    Value bits(rapidjson::kArrayType), readable(rapidjson::kArrayType);
    bool finite = true;
    for (std::size_t i = 0; i < count; ++i) {
        Value word;
        word.SetUint64(Bits(values[i]));
        bits.PushBack(word, result.GetAllocator());
        Value number;
        if (std::isfinite(values[i])) number.SetDouble(values[i]);
        else finite = false; // Null retains position without invalid JSON.
        readable.PushBack(number, result.GetAllocator());
    }
    Boolean(result, "all_finite", finite);
    result.AddMember("binary64_bits", bits, result.GetAllocator());
    result.AddMember("values", readable, result.GetAllocator());
    return result;
}

void Vector(Document& result, const char* name, contact::Vec3 value) {
    const double xyz[]{value.x, value.y, value.z};
    array_json::Child(result, name, ReportedNumbers(xyz, 3));
}

Document Facet(const contact::SelfContactTransactionReport& report,
               unsigned side) {
    const auto& motion = report.offending_motion[side];
    Document result;
    result.SetObject();
    const bool available = motion.facet.parent_eid != 0;
    Boolean(result, "available", available);
    if (!available) return result;
    Integer(result, "source_instance_id", motion.facet.source_instance_id);
    Integer(result, "parent_eid", motion.facet.parent_eid);
    Integer(result, "level", motion.facet.level);
    Integer(result, "local_facet", motion.facet.local_facet);
    if (motion.active_parent != SIZE_MAX)
        Integer(result, "active_parent_ordinal", motion.active_parent);
    Integer(result, "motion_code", static_cast<unsigned>(motion.motion));
    Integer(result, "rigid_group_count", motion.rigid_group_count);
    Boolean(result, "rigid_groups_complete", motion.rigid_group_count <= 4);
    Value groups(rapidjson::kArrayType);
    for (std::size_t i = 0; i < std::min<std::size_t>(motion.rigid_group_count, 4); ++i) {
        const auto& group = motion.rigid_groups[i];
        Document child;
        child.SetObject();
        if (group.binding_group != SIZE_MAX)
            Integer(child, "binding_group_ordinal", group.binding_group);
        Integer(child, "source_kind_code", static_cast<unsigned>(group.source_kind));
        Integer(child, "source_group_id", group.source_group_id);
        Integer(child, "source_node_set_id", group.source_node_set_id);
        Value value;
        value.CopyFrom(child, result.GetAllocator());
        groups.PushBack(value, result.GetAllocator());
    }
    result.AddMember("rigid_groups", groups, result.GetAllocator());
    array_json::Child(result, "reported_half_thickness_m",
        ReportedNumbers(&report.offending_half_thickness_m[side], 1));
    Vector(result, "reported_swept_lower_m", report.offending_swept_bounds[side].lower);
    Vector(result, "reported_swept_upper_m", report.offending_swept_bounds[side].upper);
    Value coefficients(rapidjson::kArrayType);
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
        Document child;
        child.SetObject();
        Vector(child, "lower", report.offending_quadratic_lower[side][vertex]);
        Vector(child, "upper", report.offending_quadratic_upper[side][vertex]);
        Value value;
        value.CopyFrom(child, result.GetAllocator());
        coefficients.PushBack(value, result.GetAllocator());
    }
    result.AddMember("reported_quadratic_coefficients", coefficients,
                     result.GetAllocator());
    return result;
}

}  // namespace

void AppendSelfContactErrorGeometry(
    output::Document& document, const contact::SelfContactTransactionReport& report) {
    using namespace output;
    String(document, "geometry_diagnostics_scope",
        "report fields as captured; zero values may be unpopulated; "
        "no endpoint geometry or accepted-owner ledger; not a replay fixture");
    if (report.status == contact::SelfContactTransactionStatus::UnresolvedCandidate &&
        report.pair != SIZE_MAX)
        String(document, "pair_ordinal_scope",
            "report-local index; candidate publication failures use the current filtered chunk");
    if (report.crossing_reason == contact::RepresentedIntervalReason::WorkExhausted)
        String(document, "crossing_reason_detail",
            "represented traversal work or depth exhausted; fallback diagnostics "
            "may not be populated by this report path");
    Value facets(rapidjson::kArrayType);
    for (unsigned side = 0; side < 2; ++side) {
        const auto child = Facet(report, side);
        Value value;
        value.CopyFrom(child, document.GetAllocator());
        facets.PushBack(value, document.GetAllocator());
    }
    document.AddMember("offending_facets", facets, document.GetAllocator());
    array_json::Child(document, "reported_feature_distance_m",
        ReportedNumbers(&report.offending_feature_distance_m, 1));
    array_json::Child(document, "reported_edge_parameters",
        ReportedNumbers(report.offending_edge_parameters, 2));
}

}  // namespace crash::cases::vehicle_run::detail
