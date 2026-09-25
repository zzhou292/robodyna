#include "Internal.h"
#include <algorithm>

namespace crash::modelio::vehicle::resolution {
std::size_t Preflight(const VehicleSourcePlan& plan, const assembly::ArtifactIdentity& id,
                      ResolutionLimits limits, BudgetScope scope) {
    Require(scope == BudgetScope::Artifact || scope == BudgetScope::CompleteRigidOverlay,
            "Unknown vehicle resolution budget scope");
    const auto maximum = scope == BudgetScope::CompleteRigidOverlay ?
        ResolutionLimits::CompleteRigidOverlay() : ResolutionLimits{};
    Require(limits.declaration_bytes && limits.declaration_bytes <= maximum.declaration_bytes &&
            limits.host_bytes && limits.host_bytes <= maximum.host_bytes &&
            limits.parents && limits.parents <= maximum.parents &&
            limits.parts && limits.parts <= maximum.parts &&
            limits.tables && limits.tables <= maximum.tables &&
            limits.curve_points >= 2 && limits.curve_points <= maximum.curve_points,
            "Invalid vehicle section resolution limits");
    Require(plan.counts().parents <= limits.parents && plan.parts().size() <= limits.parts &&
            id.bytes && id.bytes <= limits.declaration_bytes && id.sha256.size() == 64 &&
            std::all_of(id.sha256.begin(), id.sha256.end(), [](char c) {
                return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
            }), "Vehicle resolution count/content identity exceeds limits");
    // Include the source plan's conservative retained/startup allowance once.
    // The sidecar DOM, typed records, complete parent declarations and decode
    // scratch are additional; this is a payload bound, not allocator/RSS.
    std::size_t budget = 0;
    const auto add = [&](std::size_t count, std::size_t width) {
        Require(count <= (limits.host_bytes - budget) / width, "Vehicle resolution host cap exceeded");
        budget += count * width;
    };
    add(plan.startup_budget_bytes(), 1);
    add(id.bytes, 16);
    add(plan.counts().parents, sizeof(SectionParentResolution) + sizeof(tl::fea::ShellFailureParentInput));
    add(plan.canonical().data().canonical_shells, 6 * sizeof(std::uint64_t));
    add(plan.parts().size(), 2048 + sizeof(tl::fea::ShellPlasticityMaterialInput));
    return budget;
}

void CheckAuthority(const VehicleSourcePlan& plan, const Value& doc) {
    const auto schema = Text(doc,"schema");
    const bool glass = schema == GlassResolutionSchema;
    Require(glass || schema == ResolutionSchema, "Unsupported vehicle section resolution schema");
    Require(glass == doc.HasMember("glass_declarations"), "Glass declarations require explicit resolution V2");
    if (glass) CheckGlassPolicy(Member(doc,"glass_declarations"));
    Flag(doc, "simulation_ready", false);
    Flag(doc, "native_startup_qualified", false);
    const auto& source = Member(doc, "source");
    const auto& canonical = plan.canonical().data();
    TextIs(source, "vehicle_plan_sha256", plan.identity().sha256);
    Require(Unsigned(source, "vehicle_plan_bytes") == plan.identity().bytes, "Historical plan size changed");
    TextIs(source, "canonical_manifest_sha256", canonical.inputs.canonical_manifest.sha256);
    TextIs(source, "scope_sha256", canonical.inputs.scope_report.sha256);
    TextIs(source, "archive_sha256", canonical.archive_sha256);
    TextIs(source, "member_sha256", canonical.inputs.source_member.sha256);
    TextIs(source, "tire_policy", canonical.inputs.tire_policy);
    Require(Unsigned(source, "scope_bytes") == canonical.inputs.scope_report.bytes &&
            Unsigned(source, "member_bytes") == canonical.inputs.source_member.bytes,
            "Original resolution source size changed");
    const auto& policy = Member(doc, "policy");
    TextIs(policy, "name", "openradioss_mat024_positive_fail_constant_all_points_v1");
    TextIs(policy, "revision", "a62b27e6baa555d222a580d6218867d0be4d70b5");
    TextIs(policy, "material_source", "reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:6390-6424");
    TextIs(policy, "failure_source", "reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:1485-1494,1715-1724");
    TextIs(policy, "failure", "ConstantAllPoints");
    TextIs(policy, "D1", "FAIL");
    for (const auto* key : {"D2", "D3", "D4", "D5"}) Same(Real(policy, key), 0);
    Require(Unsigned(policy, "IFAIL_SH") == 2 && Unsigned(policy, "NIP") == 3, "Failure caller policy changed");
    Same(Real(policy, "source_time_to_s"), 1);
    Same(Real(policy, "rate_filter_hz"), 10000);
    TextIs(policy, "table_continuation", "NativeLastSegment");
    TextIs(policy, "tabulated_etan", "blank_or_zero_preserved_native_B_zero");
    Flag(policy, "point_eps_max_cleared", true);
    Flag(policy, "simulation_ready", false);
}
} // namespace crash::modelio::vehicle::resolution
