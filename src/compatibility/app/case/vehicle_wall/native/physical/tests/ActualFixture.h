#pragma once
#include "../../EnvelopePhysicalSource.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_self_contact/native/nodal_seed/tests/ActualMembers.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include <cstdlib>
#include <sstream>
namespace crash::cases::vehicle_wall::native::physical_test {
namespace original = vehicle_startup::physical_model::supports_test;
inline const WallSource& Wall() {
    static const auto value=[] {
        const auto& domain=original::Domain();
        vehicle_self_contact::native::nodal_seed::test::ActualMembers members(domain.source().tied_source().canonical());
        const auto* path=std::getenv("ROBO_NATIVE_WALL_MANIFEST");
        output::Require(path&&*path,"Missing authenticated original wall manifest");
        const auto bytes=case_data::ReadPinnedWallManifest(path);
        case_data::CanonicalWall canonical;std::istringstream input(bytes);
        output::Require(canonical.Load(input).status==case_data::WallStatus::Ok,"Original wall manifest rejected");
        Declaration declaration;declaration.profile=Profile::EnvelopeFixedElasticV1;
        const auto made=WallSource::Prepare(domain,members.Input(),canonical,bytes,declaration);
        output::Require(made.report.status==Status::Ready&&bool(made.source),made.report.reason.c_str());
        return *made.source;
    }();
    return value;
}
inline const vehicle_startup::VehicleShellReferences& References() {
    static const auto value=vehicle_startup::VehicleShellReferences::Prepare(modelio::vehicle::test::RigidResolution(),
        vehicle_startup::QephMetricProfile::AuthenticatedSourceLength,vehicle_startup::ReferenceLimits::CompleteRigidOverlay());
    return value;
}
}
