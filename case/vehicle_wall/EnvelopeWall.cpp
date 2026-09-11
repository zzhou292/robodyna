#include "EnvelopeWall.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall {
namespace c = tlfea::contact;
c::PlanarWallView EnvelopeWall::view() const noexcept {
    return {vertices_.data(),4,triangles_.data(),2};
}
EnvelopeWall EnvelopeWall::Prepare(const PlacementValues& placement, const Settings& settings) {
    CheckSettings(settings);
    EnvelopeWall result;
    auto& box = result.extent_;
    box = placement.projected_wall_box;
    // A second explicit margin leaves every declared envelope point farther
    // from exposed mesh edges than the contact clearance, including roundoff.
    for (auto ends : {std::pair<double*,double*>{&box.minimum.y,&box.maximum.y},
                      std::pair<double*,double*>{&box.minimum.z,&box.maximum.z}}) {
        output::Require(c::q4_bounds::AddScalar(*ends.first,-settings.transverse_margin_m,false,ends.first) &&
                        c::q4_bounds::AddScalar(*ends.second,settings.transverse_margin_m,true,ends.second),
                        "Generated wall extent overflows");
    }
    const auto lo = box.minimum;
    const auto hi = box.maximum;
    const std::uint64_t base = 0xd700000000000000ULL;
    result.vertices_ = {{{{lo.x,lo.y,lo.z},base+1,base+1},
                         {{lo.x,lo.y,hi.z},base+2,base+2},
                         {{lo.x,hi.y,hi.z},base+3,base+3},
                         {{lo.x,hi.y,lo.z},base+4,base+4}}};
    result.triangles_ = {{{{0,1,2},base+5,base+7,base+7},
                          {{0,2,3},base+6,base+7,base+7}}};
    const auto checked = result.geometry_.Initialize(result.view());
    output::Require(checked.status == c::PlanarContactStatus::Ok,checked.message);
    c::PlanarWallBoxCoverage coverage;
    const auto report = c::CheckPlanarWallBox(result.geometry_,placement.projected_wall_box,
        settings.exposed_clearance_m,settings.wall_binding_id,c::PlanarWallBoxMode::ConservativeExpansion,&coverage);
    output::Require(report.status == c::PlanarContactStatus::Ok && coverage.covered,report.message);
    return result;
}
} // namespace crash::cases::vehicle_wall
