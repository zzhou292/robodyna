#include "SetupData.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall {
VehicleWallSetup VehicleWallSetup::Prepare(const vehicle_runtime::Execution& e,const vehicle_runtime::Attachments& a,
    const case_data::CanonicalWall& canonical,const std::string& bytes,const Settings& settings,Limits limits) {
    const auto forecast=Preflight(e,a,canonical,bytes,settings,limits);
    auto next=std::make_shared<Data>(e,a);
    next->settings=settings;
    next->forecast=forecast;
    const auto geometry=next->geometry.InitializeMapped(*e.physical().mapping(),limits.geometry);
    output::Require(bool(geometry),geometry.message);
    output::Require(next->geometry.startup_payload_bytes()==forecast.geometry.startup_bytes &&
        next->geometry.mapping()->mapping().data()==e.physical().mapping()->mapping().data(),
        "Prepared wall geometry differs from its exact shared mapping forecast");
    next->placement=Place(next->geometry.reference_bounds(),settings);
    const auto placed=next->wall.Initialize(canonical,bytes,next->placement.translation_x_m);
    output::Require(placed.status==case_data::PlacedWallStatus::Ok,placed.diagnostic.c_str());
    output::Require(next->wall.geometry()->wall_x()==next->placement.represented_wall_x_m,
        "Placed canonical wall differs from represented X operation");
    next->original_coverage_report=next->geometry.CheckWallCoverage(*next->wall.geometry(),
        next->placement.declared_world_envelope,settings.exposed_clearance_m,settings.wall_binding_id,&next->original_coverage);
    next->coverage_report=next->original_coverage_report;
    next->coverage=next->original_coverage;
    CheckOriginalCoverage(next->original_coverage_report,settings.mesh_profile);
    if (settings.mesh_profile==WallMeshProfile::EnvelopeRectangleV1) {
        next->generated=EnvelopeWall::Prepare(next->placement,settings);
        next->coverage_report=next->geometry.CheckWallCoverage(next->generated->geometry(),
            next->placement.declared_world_envelope,settings.exposed_clearance_m,
            settings.wall_binding_id,&next->coverage);
        output::Require(next->coverage_report.status==tlfea::contact::PlanarContactStatus::Ok &&
            next->coverage.covered,next->coverage_report.message);
    }
    return VehicleWallSetup(std::move(next));
}
const vehicle_runtime::Execution& VehicleWallSetup::execution() const noexcept {return data_->execution;}
const vehicle_runtime::Attachments& VehicleWallSetup::attachments() const noexcept {return data_->attachments;}
const Settings& VehicleWallSetup::settings() const noexcept {return data_->settings;}
const SetupForecast& VehicleWallSetup::forecast() const noexcept {return data_->forecast;}
const ShellCollectionContactGeometry& VehicleWallSetup::geometry() const noexcept {return data_->geometry;}
const case_data::PlacedCanonicalWall& VehicleWallSetup::wall() const noexcept {return data_->wall;}
tlfea::contact::PlanarWallView VehicleWallSetup::selected_wall_view() const noexcept {
    return data_->generated ? data_->generated->view() : data_->wall.view();
}
const tlfea::contact::PlanarWallGeometry& VehicleWallSetup::selected_wall_geometry() const noexcept {
    return data_->generated ? data_->generated->geometry() : *data_->wall.geometry();
}
const tlfea::contact::PlanarContactReport& VehicleWallSetup::original_coverage_report() const noexcept {
    return data_->original_coverage_report;
}
const PlacementValues& VehicleWallSetup::placement() const noexcept {return data_->placement;}
const tlfea::contact::PlanarWallBoxCoverage& VehicleWallSetup::coverage() const noexcept {return data_->coverage;}
const tlfea::contact::PlanarContactReport& VehicleWallSetup::coverage_report() const noexcept {return data_->coverage_report;}
} // namespace crash::cases::vehicle_wall
