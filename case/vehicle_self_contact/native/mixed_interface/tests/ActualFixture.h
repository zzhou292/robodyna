#pragma once
#include "../Internal.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "../../coated/tests/ActualFixture.h"
namespace crash::cases::vehicle_self_contact::native::mixed_interface::test {
namespace actual_physical = vehicle_startup::physical_model::supports_test;
inline const Initial& ActualInitialSource() {
    static const auto value = [] {
        const auto& model = actual_physical::Model();
        nodal_seed::test::ActualMembers members(model.shell_source().references().source().canonical());
        const auto before = nodal_seed::PreCorrectionNodalSource::Prepare(model, actual_physical::Joints(), members.Input());
        const auto corrected = initial_surfaces::Context::Prepare(before, members.Input());
        output::Require(corrected.report.status == nodal_correction::Status::Ready && corrected.source,
            "Mixed actual fixture requires the qualified corrected source");
        const auto initial = Initial::Prepare(*corrected.source, coated::test::Selection(), actual_physical::Inputs().member);
        output::Require(initial.report.status == initial_surfaces::Status::Ready && initial.source,
            "Mixed actual fixture requires qualified complete initial surfaces");
        return *initial.source;
    }();
    return value;
}
}
