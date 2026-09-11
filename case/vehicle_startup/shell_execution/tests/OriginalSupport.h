#pragma once
#include "../VehicleShellExecution.h"
#include "../Packing.h"
#include "case/vehicle_startup/physical_model/tests/Support.h"

namespace crash::cases::vehicle_startup::shell_execution::test {
namespace fe = tl::fea;
namespace source = modelio::vehicle;
inline const VehicleShellExecution& Actual() {
    static const auto value = VehicleShellExecution::Prepare(physical_model::test::Actual());
    return value;
}
inline void Same(double a, double b) { EXPECT_EQ(output::Bits(a),output::Bits(b)); }
inline void SameFailure(const fe::ShellFailureParentInput& a, const fe::ShellFailureParentInput& b) {
    EXPECT_TRUE(detail::Same(a.source,b.source));
    EXPECT_EQ(a.policy,b.policy);
    Same(a.constant.failure_strain,b.constant.failure_strain);
    EXPECT_EQ(a.tab1.parent_policy,b.tab1.parent_policy);
    Same(a.tab1.table.failure_strain,b.tab1.table.failure_strain);
    for (unsigned i = 0; i < 3; ++i) Same(a.tab1.table.triaxiality[i],b.tab1.table.triaxiality[i]);
}
inline const modelio::assembly::Curve* OriginalCurve(const VehicleSectionResolution& r, std::uint64_t id) {
    for (const auto* curves : {&r.source().curves(),&r.failure_curves()})
        for (const auto& curve : *curves) if (curve.id == id) return &curve;
    return nullptr;
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
