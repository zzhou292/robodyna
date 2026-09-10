#include "SourceAssemblyWallSetup.h"
#include "case/wall_penalty/WallPlacementBounds.h"
#include <cmath>
#include <new>

namespace crash::cases::source_assembly {
namespace sc=tlfea::contact;
using Code=SourceAssemblyWallStatus;
namespace {
bool ValidSettings(const SourceAssemblyWallSettings& s) noexcept {
    for(double value:{s.initial_velocity[0],s.leading_gap,s.motion_margin,s.exposed_clearance,
                     s.parent_force_error,s.parent_energy_error,s.maximum_step_rate})
        if(!std::isfinite(value)||value<=0)return false;
    return s.initial_velocity[1]==0&&s.initial_velocity[2]==0&&s.exposed_clearance<s.motion_margin&&
        s.maximum_step_rate<=.125&&wall_penalty::ValidPenaltyDesign(s.penalty)&&
        s.configuration_id&&s.qualification_id&&s.wall_binding_id&&
        s.boundary==SourceAssemblyWallBoundary::ReleasedExternalConnections;
}
} // namespace
struct SourceAssemblyWallSetup::Data {
    explicit Data(const SourceAssemblyBindings& source):bindings(source) {}
    SourceAssemblyBindings bindings;
    SourceAssemblyWallSettings settings;
    SourceAssemblyWallCertificate certificate;
    ShellCollectionContactGeometry geometry;
    case_data::PlacedCanonicalWall wall;
    std::size_t startup_bytes=0;
};
SourceAssemblyWallSetup::SourceAssemblyWallSetup()=default;
SourceAssemblyWallSetup::~SourceAssemblyWallSetup()=default;
SourceAssemblyWallReport SourceAssemblyWallSetup::Initialize(const SourceAssemblyBindings& source,
    const case_data::CanonicalWall& canonical,const std::string& bytes,const SourceAssemblyWallSettings& settings,
    const SourceAssemblyWallLimits& limits) {
    if(data_)return {Code::AlreadyInitialized,"Source assembly wall setup is immutable after publication"};
    if(!ValidSettings(settings)||source.source().data().boundary.policy!="released_external_connections"||
       !canonical.loaded()||bytes.empty())
        return {Code::InvalidInput,"Declared uniform motion, released component boundary and original canonical wall are required"};
    if(!limits.max_startup_bytes||limits.max_startup_bytes>64*1024*1024||
       !limits.max_wall_manifest_bytes||limits.max_wall_manifest_bytes>1024*1024||bytes.size()>limits.max_wall_manifest_bytes||
       !limits.geometry.max_startup_bytes||limits.geometry.max_startup_bytes>32*1024*1024||
       limits.geometry.weights.max_owned_bytes<sizeof(sc::NodalWallWeights)||
       limits.geometry.weights.max_owned_bytes>4*1024*1024)
        return {Code::ResourceLimit,"Assembly wall setup exceeds its explicit startup or wall-manifest limits"};
    // Bounded before allocation: the declared complete geometry budget plus
    // two canonical manifest copies and a fixed reserve for original wall
    // vertices/faces/coverage maps and at most 64 group kinetic ledgers/masks.
    // Already-owned authenticated source/binding storage remains shared input.
    const std::size_t reserve=1024*1024;
    const auto payload=sizeof(Data)+limits.geometry.max_startup_bytes+2*bytes.size()+reserve;
    if(payload>limits.max_startup_bytes)
        return {Code::ResourceLimit,"Complete additional wall setup startup payload exceeds its byte budget"};
    try {
        auto next=std::make_shared<Data>(source);next->settings=settings;next->startup_bytes=payload;
        const auto geometry=next->geometry.Initialize(source.shells(),limits.geometry);
        if(!geometry)return {geometry.status==ShellContactGeometryStatus::ResourceLimit?Code::ResourceLimit:Code::GeometryFailure,
                             geometry.message,SIZE_MAX,geometry.parent.family_index};
        const auto energy=EncloseSourceAssemblyInitialKinetic(source,settings.initial_velocity[0],&next->certificate.initial_kinetic);
        if(!energy)return {energy.status==InitialKineticStatus::ResourceLimit?Code::ResourceLimit:
                           energy.status==InitialKineticStatus::InvalidInput?Code::InvalidInput:Code::CertificateFailure,
                           energy.message,energy.node};
        auto& certificate=next->certificate;
        const auto penalty=wall_penalty::CertifyPenalty(*next->geometry.weights(),source.shells().node_count(),
            {certificate.initial_kinetic.with_aggregate_groups,wall_penalty::InitialKineticMetric::NativeNodesWithAggregateGroups},
            settings.penalty,&certificate.penalty);
        if(!penalty)return {Code::CertificateFailure,penalty.message,penalty.node};
        const auto bounds=next->geometry.reference_bounds();
        const double desired_wall_x=bounds[1].x+settings.leading_gap;
        const double translation_x=desired_wall_x-.05; // Exact canonical-loader X contract.
        if(!std::isfinite(desired_wall_x)||!std::isfinite(translation_x)||
           next->wall.Initialize(canonical,bytes,translation_x).status!=case_data::PlacedWallStatus::Ok)
            return {Code::GeometryFailure,"Authenticated complete wall cannot be placed by its declared X translation"};
        if(!wall_penalty::EncloseLeadingGap(next->wall.geometry()->wall_x(),bounds[1].x,&certificate.leading_gap))
            return {Code::CertificateFailure,"Represented mesh-wall placement does not certify a positive leading gap"};
        sc::PlanarWallBox motion;
        if(!wall_penalty::ExpandProjectedMotion(bounds,settings.motion_margin,&motion))
            return {Code::CertificateFailure,"Complete source motion envelope cannot be rounded outward"};
        // This adapter projects BOTH X endpoints onto the actual placed wall.
        // The retained geometry and all physical source coordinates stay fixed.
        const auto coverage=next->geometry.CheckWallCoverage(*next->wall.geometry(),motion,settings.exposed_clearance,
                                                            settings.wall_binding_id,&certificate.coverage);
        if(coverage.status!=sc::PlanarContactStatus::Ok)return {Code::GeometryFailure,coverage.message};
        data_=std::move(next);return {Code::Ok,"Complete source geometry, aggregate-startup penalty and finite wall coverage prepared"};
    } catch(const std::bad_alloc&) { return {Code::ResourceLimit,"Bounded source assembly wall allocation failed"}; }
}
bool SourceAssemblyWallSetup::initialized() const noexcept { return bool(data_); }
const SourceAssemblyBindings* SourceAssemblyWallSetup::bindings() const noexcept { return data_?&data_->bindings:nullptr; }
const SourceAssemblyWallSettings* SourceAssemblyWallSetup::settings() const noexcept { return data_?&data_->settings:nullptr; }
const SourceAssemblyWallCertificate* SourceAssemblyWallSetup::certificate() const noexcept { return data_?&data_->certificate:nullptr; }
const ShellCollectionContactGeometry* SourceAssemblyWallSetup::source_geometry() const noexcept { return data_?&data_->geometry:nullptr; }
const case_data::PlacedCanonicalWall* SourceAssemblyWallSetup::placed_wall() const noexcept { return data_?&data_->wall:nullptr; }
std::size_t SourceAssemblyWallSetup::startup_payload_bytes() const noexcept { return data_?data_->startup_bytes:0; }
} // namespace crash::cases::source_assembly
