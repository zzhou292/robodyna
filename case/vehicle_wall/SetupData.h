#pragma once
#include "VehicleWallSetup.h"
#include "EnvelopeWall.h"
#include <optional>
namespace crash::cases::vehicle_wall {
struct VehicleWallSetup::Data {
    Data(const vehicle_runtime::Execution& e,const vehicle_runtime::Attachments& a):execution(e),attachments(a) {}
    vehicle_runtime::Execution execution;
    vehicle_runtime::Attachments attachments;
    Settings settings;
    SetupForecast forecast;
    ShellCollectionContactGeometry geometry;
    case_data::PlacedCanonicalWall wall;
    std::optional<EnvelopeWall> generated;
    PlacementValues placement;
    tlfea::contact::PlanarWallBoxCoverage coverage;
    tlfea::contact::PlanarContactReport coverage_report;
    tlfea::contact::PlanarContactReport original_coverage_report;
    tlfea::contact::PlanarWallBoxCoverage original_coverage;
};
} // namespace crash::cases::vehicle_wall
