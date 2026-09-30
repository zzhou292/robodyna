#pragma once
#include "../SourceAdmission.h"
#include "case/vehicle_runtime/source/tests/ActualFixture.h"
#include "case/vehicle_self_contact/native/mixed_main/tests/ActualFixture.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include "case/vehicle_self_contact/native/nodal_seed/tests/ActualMembers.h"
namespace crash::cases::vehicle_native_contact::test {
namespace main_source = vehicle_self_contact::native::post_gapm;
namespace starter_source = vehicle_self_contact::native::mixed_starter;
namespace wall_source = vehicle_wall::native::wall_interface;
namespace controls_source = vehicle_self_contact::native::initial_controls;
inline const vehicle_wall::native::EnvelopeOwnerSource& ActualOwnerSource() {
    return vehicle_runtime::source_test::OwnerSource();
}
inline const main_source::PostGapmMainSource& ActualMainSource() {
    // Preserve the already-qualified owner-then-contact construction order.
    (void)ActualOwnerSource();
    return main_source::test::ActualPostGapmSource();
}
inline const wall_source::FiniteWallContactSource& ActualWallSource() {
    static const auto value = [] {
        const auto made = wall_source::FiniteWallContactSource::Prepare(ActualOwnerSource(), ActualMainSource(),
            {wall_source::Profile::AllRetainedVehicleNodesToFixedMeshV1, 1, 1});
        output::Require(made.report.status == wall_source::Status::Prepared && made.source,
                        made.report.reason.c_str());
        return *made.source;
    }();
    return value;
}
inline const starter_source::MixedStarterSource& ActualSelfSource() {
    static const auto value = [] {
        (void)ActualWallSource();
        const auto& embedding = ActualOwnerSource().execution_source().mechanical().embedding();
        const auto made = starter_source::MixedStarterSource::Prepare(ActualMainSource(), embedding);
        output::Require(made.report.status == starter_source::Status::Ready && made.source,
                        made.report.reason.c_str());
        return *made.source;
    }();
    return value;
}
inline const controls_source::InitializerControlsSource& ActualControlsSource() {
    static const auto value = [] {
        (void)ActualSelfSource();
        const auto& main = ActualMainSource();
        vehicle_self_contact::native::nodal_seed::test::ActualMembers members(main.mixed().initial().selection().canonical());
        const auto& wall = ActualOwnerSource().execution_source().mechanical().wall();
        const auto made = controls_source::InitializerControlsSource::Prepare(main, members.Input(), &wall);
        output::Require(made.report.status == controls_source::Status::Ready && made.source,
                        made.report.reason.c_str());
        return *made.source;
    }();
    return value;
}
inline detail::SourceInputs ActualSources() {
    const auto& controls = ActualControlsSource();
    return {ActualOwnerSource(), ActualSelfSource(), ActualWallSource(), controls};
}
} // namespace crash::cases::vehicle_native_contact::test
