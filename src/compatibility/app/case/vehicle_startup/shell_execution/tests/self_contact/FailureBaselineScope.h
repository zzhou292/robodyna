#pragma once

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::failure_detail {

inline constexpr const char* UnreportedBaselineScope =
    "baseline nonlinear status/depth/work unreported; original typed rejection retained separately; "
    "replay uses configured standalone limits, not observed remaining production budget";
inline constexpr const char* ObservedBaselineScope =
    "observed combined nonlinear status/depth/work; work includes initial-root plus coverage work "
    "under production remaining budgets; replay uses configured standalone limits, "
    "not an exact production-budget replay";
inline constexpr const char* UnreportedBaselineDepthScope =
    "unreported; not a measured production subdivision depth";
inline constexpr const char* ObservedBaselineDepthScope =
    "observed maximum of initial-root and coverage depth; not standalone replay depth";

inline constexpr const char* LegacyFailureManifestSchema =
    "robo_dyna.self_contact_failure_fixture.v1";
inline constexpr const char* FailureManifestSchema =
    "robo_dyna.self_contact_failure_fixture.v2";

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::failure_detail
