#include "SourcePartWallSetup.h"
#include "SourcePartWallCertification.h"
#include "qualification/source_contact/SourceShellCollection.h"
#include "case/wall_penalty/WallPlacementBounds.h"
#include <cmath>
#include <cstring>
#include <new>

namespace crash::cases::source_part_wall {
namespace {
using Code=SourcePartWallStatus;

bool SameBits(double a,double b) noexcept {
    std::uint64_t x=0,y=0;std::memcpy(&x,&a,sizeof(x));std::memcpy(&y,&b,sizeof(y));return x==y;
}
bool InitialStamp(const tl::fea::NodalStamp& stamp) noexcept {
    return stamp.owner_id&&stamp.node_count==source::NodeCount&&stamp.has_rotations&&
        stamp.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart&&
        stamp.velocity_phase==tl::fea::NodalVelocityPhase::Collocated&&
        !stamp.epoch&&stamp.time==0&&stamp.velocity_time==0&&!stamp.reactions_valid&&
        !stamp.reaction_base_epoch&&stamp.reaction_time==0&&stamp.reaction_kick_dt==0&&
        std::isfinite(stamp.fixed_dt)&&stamp.fixed_dt>=1e-12;
}
// The case owns startup authentication. This additional local check prevents
// associating a different source/material inventory with these contact areas.
// All mass operands used below still come from the actual caller binding.
bool MatchesOriginal(const source::SourcePartContactFixture& input,const tl::fea::ShellBatchBinding& binding) {
    if(!binding.prepared()||binding.node_count()!=source::NodeCount||binding.qeph_count()!=source::Q4Count||
       binding.t3_count()!=source::T3Count)return false;
    auto collection=std::make_unique<source::SourceShellCollection>();
    auto reference=std::make_unique<tl::fea::ShellBatchBinding>();
    return collection->Initialize(input).status==source::FixtureStatus::Ok&&
        reference->Initialize(collection->input()).status==tl::fea::ShellBindingStatus::Success&&
        reference->inventory()==binding.inventory();
}
} // namespace
struct SourcePartWallSetup::Data {
    SourcePartWallSettings settings;
    SourcePartWallCertificate certificate;
    SourcePartContactGeometry source_geometry;
    case_data::PlacedCanonicalWall wall;
    tl::fea::NodalStamp stamp;
    std::array<double,3*source::NodeCount> initial_positions{};
    std::array<double,source::NodeCount> inverse_mass{};
    std::array<std::uint8_t,source::NodeCount> translation_fixed_bits{};
};
SourcePartWallSetup::SourcePartWallSetup()=default;
SourcePartWallSetup::~SourcePartWallSetup()=default;
SourcePartWallReport SourcePartWallSetup::Initialize(const source::SourcePartContactFixture& input,
    const tl::fea::ShellBatchBinding& binding,const tl::fea::NodalStamp& stamp,const double* inverse,
    double measured_initial_kinetic,const case_data::CanonicalWall& canonical,const std::string& bytes,
    const SourcePartWallSettings& settings) {
    if(data_)return {Code::AlreadyInitialized,"Source wall setup is immutable after publication"};
    if(!input.prepared()||!inverse||!InitialStamp(stamp)||!detail::ValidWallSettings(settings)||
       !std::isfinite(measured_initial_kinetic)||measured_initial_kinetic<=0)
        return {Code::InvalidInput,"Invalid source, initial stamp, moving K0 or frozen wall settings"};
    try {
        if(!MatchesOriginal(input,binding))
            return {Code::InvalidInput,"Actual native shell binding differs from the complete original source inventory"};
        auto next=std::make_unique<Data>();next->settings=settings;next->stamp=stamp;
        next->initial_positions=input.coordinates();
        for(unsigned n=0;n<source::NodeCount;++n) {
            const double mass=binding.nodes()[n].native.mass;
            if(!std::isfinite(mass)||mass<=0||!std::isfinite(inverse[n])||inverse[n]<=0||!SameBits(inverse[n],1/mass))
                return {Code::MassMismatch,"Contact inverse mass differs from the actual native binding inverse",n};
            next->inverse_mass[n]=inverse[n];
        }
        std::string diagnostic;
        if(!next->source_geometry.Initialize(input,diagnostic))
            return {Code::GeometryFailure,"Original source contact geometry could not be prepared"};
        auto report=detail::CertifyWallPenalty(binding,*next->source_geometry.weights(),settings,&next->certificate);
        if(!report)return report;
        auto& certificate=next->certificate;
        if(measured_initial_kinetic<certificate.native_initial_kinetic.lower||
           measured_initial_kinetic>certificate.native_initial_kinetic.upper)
            return {Code::CertificateFailure,"Measured common K0 is outside the native-mass uniform-velocity enclosure"};
        certificate.measured_initial_kinetic=measured_initial_kinetic;
        certificate.initial_velocity=settings.initial_velocity;certificate.fixed_dt=stamp.fixed_dt;
        const auto bounds=next->source_geometry.reference_bounds();
        const double desired_wall_x=bounds[1].x+settings.leading_gap;
        // Retain this exact operation separately from the represented placed
        // coordinates. The original wall's owning loader fixes X at .05 m.
        const double translation_x=desired_wall_x-.05;
        if(!std::isfinite(desired_wall_x)||!std::isfinite(translation_x)||
           next->wall.Initialize(canonical,bytes,translation_x).status!=case_data::PlacedWallStatus::Ok)
            return {Code::GeometryFailure,"Authenticated canonical wall could not be placed by its declared X translation"};
        const double actual_wall_x=next->wall.geometry()->wall_x();
        if(!wall_penalty::EncloseLeadingGap(actual_wall_x,bounds[1].x,&certificate.leading_gap))
            return {Code::CertificateFailure,"Actual represented wall does not certify a strictly positive leading gap"};
        contact::PlanarWallBox motion;
        if(!wall_penalty::ExpandProjectedMotion(bounds,settings.motion_margin,&motion))
            return {Code::CertificateFailure,"Projected motion envelope cannot be rounded outward"};
        if(next->source_geometry.CheckWallCoverage(*next->wall.geometry(),motion.minimum,motion.maximum,
            settings.exposed_clearance,settings.wall_binding_id,&certificate.coverage).status!=contact::PlanarContactStatus::Ok)
            return {Code::GeometryFailure,"Entire projected source motion envelope is not covered by the placed finite mesh"};
        data_=std::move(next);return {Code::Ok,"Source wall geometry, native K0 and penalty design certified"};
    } catch(const std::bad_alloc&) {
        return {Code::ResourceLimit,"Bounded source wall setup allocation failed"};
    }
}
bool SourcePartWallSetup::initialized() const noexcept {return data_!=nullptr;}
const SourcePartWallSettings* SourcePartWallSetup::settings() const noexcept {return data_?&data_->settings:nullptr;}
const SourcePartWallCertificate* SourcePartWallSetup::certificate() const noexcept {return data_?&data_->certificate:nullptr;}
const SourcePartContactGeometry* SourcePartWallSetup::source_geometry() const noexcept {return data_?&data_->source_geometry:nullptr;}
const case_data::PlacedCanonicalWall* SourcePartWallSetup::placed_wall() const noexcept {return data_?&data_->wall:nullptr;}
const tl::fea::NodalStamp* SourcePartWallSetup::owner_stamp() const noexcept {return data_?&data_->stamp:nullptr;}
const double* SourcePartWallSetup::inverse_mass() const noexcept {return data_?data_->inverse_mass.data():nullptr;}
const double* SourcePartWallSetup::initial_positions() const noexcept {return data_?data_->initial_positions.data():nullptr;}
const std::uint8_t* SourcePartWallSetup::translation_fixed_bits() const noexcept {
    return data_?data_->translation_fixed_bits.data():nullptr;
}
} // namespace crash::cases::source_part_wall
