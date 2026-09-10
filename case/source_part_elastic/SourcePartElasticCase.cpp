#include "SourcePartElasticInternal.h"
#include <algorithm>
#include <cmath>
#include <new>

namespace crash::cases::source_part_elastic {
bool ValidConfig(const Config& c) noexcept {
    if(c.experiment!=Experiment::ElasticPulse&&c.experiment!=Experiment::UniformFlight) return false;
    for(double v : {c.dt,c.maximum_displacement,c.maximum_rotation,
                    c.maximum_strain,c.maximum_thickness_curvature,c.minimum_area_ratio,c.maximum_area_ratio,
                    c.minimum_thickness_ratio,c.maximum_thickness_ratio,c.maximum_energy_residual,
                    c.maximum_native_dt_fraction,c.relative_energy_residual})
        if(!std::isfinite(v)||v<=0) return false;
    for(double v:c.initial_velocity) if(!std::isfinite(v)) return false;
    if(c.experiment==Experiment::ElasticPulse) {
        if(!std::isfinite(c.pulse_duration)||c.pulse_duration<=0||!std::isfinite(c.acceleration)||c.acceleration<=0||
           c.dt>c.pulse_duration||c.initial_velocity[0]!=0||c.initial_velocity[1]!=0||c.initial_velocity[2]!=0) return false;
    } else if(c.pulse_duration!=0||c.acceleration!=0||c.spatial_axis!=0||c.direction!=std::array<double,3>{0,0,1}||
              (c.initial_velocity[0]==0&&c.initial_velocity[1]==0&&c.initial_velocity[2]==0)) return false;
    double norm=0;
    for(double v:c.direction) { if(!std::isfinite(v)) return false; norm+=v*v; }
    return std::abs(norm-1)<=8e-16 && c.spatial_axis<3 &&
        c.dt>=1e-12 && c.minimum_area_ratio<=1 && c.maximum_area_ratio>=1 &&
        c.minimum_thickness_ratio<=1 && c.maximum_thickness_ratio>=1 &&
        c.minimum_area_ratio<c.maximum_area_ratio &&
        c.minimum_thickness_ratio<c.maximum_thickness_ratio &&
        c.maximum_native_dt_fraction<=1 && c.configuration_id && c.qualification_id;
}
double PulseScale(double time,double duration) noexcept {
    if(!std::isfinite(time)||!std::isfinite(duration)||duration<=0||time<=0||time>=duration) return 0;
    constexpr double pi=3.141592653589793238462643383279502884;
    const double sine=std::sin(pi*(time/duration));
    return sine*sine;
}
SourcePartElasticCase::SourcePartElasticCase():impl_(new Impl) {}
SourcePartElasticCase::~SourcePartElasticCase()=default;
SourcePartElasticCase::Impl::~Impl() { if(device_pulse) cudaFree(device_pulse); }
Report SourcePartElasticCase::Initialize(const source::SourcePartContactFixture& source,const Config& config) {
    if(impl_->initialized) return Failure(Status::AlreadyInitialized,"Source-part case is immutable after initialization");
    if(!source.prepared()||!ValidConfig(config)) return Failure(Status::InvalidInput,"Unprepared source or invalid elastic case configuration");
    std::unique_ptr<Impl> staged(new(std::nothrow) Impl);
    if(!staged) return Failure(Status::ComponentFailure,"Source-part startup storage allocation failed");
    const auto result=staged->Initialize(source,config);
    if(!result) return result;
    impl_.swap(staged);
    return Success();
}
Report SourcePartElasticCase::Step() {
    if(!impl_->initialized) return Failure(Status::NotInitialized,"Source-part case is not initialized");
    auto result=impl_->Prepare();
    if(result) result=impl_->Evaluate();
    if(result) result=impl_->Observe();
    if(result) result=impl_->Commit();
    if(!result) impl_->Discard();
    return result;
}
Report SourcePartElasticCase::Capture(Snapshot* out) {
    if(!out) return Failure(Status::InvalidInput,"Missing accepted snapshot destination");
    if(!impl_->initialized) return Failure(Status::NotInitialized,"Source-part case is not initialized");
    const auto stamp=impl_->owner.accepted();
    if(stamp.owner_id!=impl_->accepted.stamp.owner_id||stamp.epoch!=impl_->accepted.stamp.epoch||
       stamp.time!=impl_->accepted.stamp.time)
        return Failure(Status::ComponentFailure,"Owner was advanced outside its source-part case");
    *out=impl_->accepted; // Only the complete accepted cache is exposed.
    // Output-cadence deformation metric: 6,786 unique pairs of accepted source
    // nodes. It is invariant under rigid translation/rotation and is not a
    // per-step geometry admission. Mechanics envelopes have already passed.
    for(std::size_t i=0;i<NodeCount;++i) for(std::size_t j=0;j<i;++j) {
        long double before=0,after=0;
        for(unsigned a=0;a<3;++a) {
            const long double x0=static_cast<long double>(impl_->source.coordinates()[3*i+a])-impl_->source.coordinates()[3*j+a];
            const long double x1=static_cast<long double>(out->position[3*i+a])-out->position[3*j+a];
            before+=x0*x0; after+=x1*x1;
        }
        out->diagnostics.maximum_chord_change=std::max(out->diagnostics.maximum_chord_change,
            static_cast<double>(std::abs(std::sqrt(after)-std::sqrt(before))));
    }
    return Success();
}
bool SourcePartElasticCase::initialized() const noexcept { return impl_->initialized; }
fe::FENodalState& SourcePartElasticCase::owner() noexcept { return impl_->owner; }
const source::SourcePartContactFixture& SourcePartElasticCase::source() const noexcept { return impl_->source; }
const source::SourceShellCollection& SourcePartElasticCase::collection() const noexcept { return impl_->collection; }
const fe::ShellBatchBinding& SourcePartElasticCase::binding() const noexcept { return impl_->binding; }
const Config& SourcePartElasticCase::config() const noexcept { return impl_->config; }
const Diagnostics& SourcePartElasticCase::diagnostics() const noexcept { return impl_->accepted.diagnostics; }
double SourcePartElasticCase::initial_kinetic_energy() const noexcept { return impl_->initial_kinetic; }
fe::NodalAllocationInfo SourcePartElasticCase::allocations() const noexcept {
    fe::NodalAllocationInfo result;
    for(auto a:{impl_->owner.allocations(),impl_->qeph.allocations(),impl_->t3.allocations(),impl_->publication.allocations()}) {
        result.device_bytes+=a.device_bytes; result.device_allocations+=a.device_allocations;
    }
    if(impl_->device_pulse) { result.device_bytes+=sizeof(impl_->pulse_force); ++result.device_allocations; }
    return result;
}
} // namespace crash::cases::source_part_elastic
