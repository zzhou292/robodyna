#include "State.h"
#include "lib_src/elements/type25/Type25Deformation.h"
#include <algorithm>
#include <limits>

namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyWallCase::Impl::CheckConnector() {
    const auto& common=candidate().diagnostics.shells;
    if(!connector)return common.has_connector?
        Failure(Status::SourceMismatch,"Unexpected connector participant in shell-only publication"):Success();
    const auto& d=common.connector;const auto& old=accepted().diagnostics.shells.connector;
    const auto& model=*bindings.connectors();const auto& settings=*setup.settings();
    if(!common.has_connector||!d.valid||!d.has_completed_interval||!d.accepted_force_assembled||
       d.phase!=fe::type25::BatchPhase::Prepared||d.owner_id!=prepared.owner_id||
       d.source_instance_id!=bindings.source_instance_id()||d.configuration_id!=settings.configuration_id||
       d.qualification_id!=settings.qualification_id||d.base_epoch!=prepared.kinematics.base_epoch||
       d.epoch!=d.base_epoch+1||d.attempt!=prepared.attempt||d.time!=prepared.proposed_time||
       d.base_time!=prepared.base_time||d.velocity_time!=prepared.velocity_time||
       d.base_velocity_time!=prepared.base_velocity_time||d.kick_dt!=prepared.kick_dt||
       d.element_count!=model.connection_count())
        return Failure(Status::SourceMismatch,"Prepared connector source and complete interval identity differ");
    const auto& values=connector->results[1-accepted_slot];const auto& previous=connector->results[accepted_slot];
    if(values.size()!=model.connection_count()||previous.size()!=values.size())
        return Failure(Status::ResourceLimit,"Complete connector result extent changed");
    std::array<long double,4> sum{},before{},scale{};std::size_t active=0,failed=0;
    double minimum_dt=std::numeric_limits<double>::infinity();
    for(std::size_t e=0;e<values.size();++e) {
        const auto& v=values[e];const auto& p=previous[e];const auto id=model.connections()[e].source_element_id;
        if(!fe::type25::detail::ValidHistory(v.history)||(!p.history.active&&v.history.active)||
           !tl::math::fixed3::Orthonormal(v.frame.axes)||!tl::math::fixed3::Orthonormal(v.frame.midpoint_axes)||
           !std::isfinite(v.frame.length_m)||v.frame.length_m<=0||
           !std::isfinite(v.frame.midpoint_length_m)||v.frame.midpoint_length_m<=0)
            return Failure(Status::EnvelopeFailure,"Invalid source connector frame or failure history",id);
        for(const auto& endpoint:v.endpoints)if(!tl::math::fixed3::Finite(endpoint.force_N)||!tl::math::fixed3::Finite(endpoint.couple_Nm))
            return Failure(Status::EnvelopeFailure,"Source connector wrench is nonfinite",id);
        if(!std::isfinite(v.translation_stiffness_N_per_m)||v.translation_stiffness_N_per_m<=0||
           !std::isfinite(v.rotation_stiffness_Nm_per_rad)||v.rotation_stiffness_Nm_per_rad<=0||
           !std::isfinite(v.critical_dt_s)||v.critical_dt_s<=0||
           config.fixed_dt>v.critical_dt_s*config.deformation.maximum_native_dt_fraction)
            return Failure(Status::EnvelopeFailure,"Source connector native timestep exceeded",id,SIZE_MAX,
                           config.fixed_dt,v.critical_dt_s*config.deformation.maximum_native_dt_fraction);
        minimum_dt=std::min(minimum_dt,v.critical_dt_s);active+=v.history.active;
        failed+=p.history.active&&!v.history.active;
        for(unsigned c=0;c<4;++c) {
            sum[c]+=v.history.internal_work_J[c];before[c]+=p.history.internal_work_J[c];
            scale[c]+=std::abs(v.history.internal_work_J[c])+std::abs(p.history.internal_work_J[c]);
        }
    }
    if(d.active_count!=active||d.newly_failed_count!=failed||d.minimum_native_dt!=minimum_dt)
        return Failure(Status::ComponentFailure,"Complete connector activity or timestep reduction differs");
    for(unsigned c=0;c<4;++c) {
        const long double budget=512*std::numeric_limits<double>::epsilon()*
            (scale[c]+std::abs(static_cast<long double>(d.internal_work_increment_J[c])));
        const long double residual=static_cast<long double>(d.internal_work_J[c])-old.internal_work_J[c]-d.internal_work_increment_J[c];
        if(!std::isfinite(d.internal_work_J[c])||!std::isfinite(d.internal_work_increment_J[c])||
           !std::isfinite(static_cast<double>(budget))||!std::isfinite(static_cast<double>(residual))||
           std::abs(sum[c]-d.internal_work_J[c])>budget||std::abs(before[c]-old.internal_work_J[c])>budget||std::abs(residual)>budget)
            return Failure(Status::EnvelopeFailure,"Connector native signed work reduction differs",0,SIZE_MAX,
                           static_cast<double>(residual),static_cast<double>(budget));
    }
    for(double work:{d.internal_kick_work,d.internal_drift_work})if(!std::isfinite(work))
        return Failure(Status::EnvelopeFailure,"Connector applied signed work is nonfinite");
    return Success();
}
} // namespace crash::cases::source_assembly_dynamics
