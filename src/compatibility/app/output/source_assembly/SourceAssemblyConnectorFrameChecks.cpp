#include "SourceAssemblyConnectorFields.h"
#include "lib_src/elements/type25/Type25Deformation.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::output::assembly::wall_fields {
namespace {
using Diagnostic=tl::fea::type25::BatchDiagnostics;
void Same(const Diagnostic& a,const Diagnostic& b) {
    Require(a.source_instance_id==b.source_instance_id&&a.owner_id==b.owner_id&&a.configuration_id==b.configuration_id&&
        a.qualification_id==b.qualification_id&&a.epoch==b.epoch&&a.base_epoch==b.base_epoch&&a.attempt==b.attempt&&
        a.phase==b.phase&&a.valid==b.valid&&a.has_completed_interval==b.has_completed_interval&&
        a.accepted_force_assembled==b.accepted_force_assembled&&a.element_count==b.element_count&&
        a.active_count==b.active_count&&a.newly_failed_count==b.newly_failed_count,"Captured connector identity changed");
    const double x[]{a.time,a.base_time,a.velocity_time,a.base_velocity_time,a.kick_dt,
                     a.internal_kick_work,a.internal_drift_work,a.minimum_native_dt};
    const double y[]{b.time,b.base_time,b.velocity_time,b.base_velocity_time,b.kick_dt,
                     b.internal_kick_work,b.internal_drift_work,b.minimum_native_dt};
    for(unsigned i=0;i<8;++i)Require(std::isfinite(x[i])&&Bits(x[i])==Bits(y[i]),"Captured connector phase/work changed");
    for(unsigned i=0;i<4;++i)Require(std::isfinite(a.internal_work_J[i])&&std::isfinite(a.internal_work_increment_J[i])&&
        Bits(a.internal_work_J[i])==Bits(b.internal_work_J[i])&&Bits(a.internal_work_increment_J[i])==Bits(b.internal_work_increment_J[i]),
        "Captured connector channel work changed");
}
}
void CheckConnectorFrame(const FrameView& v) {
    Require(v.bindings&&v.stamp&&v.diagnostics&&v.captured_shells&&v.setup&&v.setup->settings(),"Missing connector frame association");
    const auto* model=v.bindings->connectors();const auto& common=v.diagnostics->shells;const auto& captured=*v.captured_shells;
    if(!model) {
        Require(!v.connectors.diagnostics&&!v.connectors.elements&&!v.connectors.count&&!common.has_connector&&!captured.has_connector,
                "Shell-only output contains an undeclared connector");
        for(const auto* k:{&common.base_kinetic,&common.kinetic,&captured.base_kinetic,&captured.kinetic})
            Require(k->connector_translation==0&&k->connector_rotation==0,"Shell-only output contains connector kinetic");
        return;
    }
    Require(v.connectors.diagnostics&&v.connectors.elements&&v.connectors.count==model->connection_count()&&
            v.connectors.count<=MaxArchivedConnectors&&common.has_connector&&captured.has_connector,
            "Incomplete accepted connector output");
    const auto& d=*v.connectors.diagnostics;const auto& s=*v.stamp;const auto& settings=*v.setup->settings();
    Same(d,common.connector);Same(d,captured.connector);
    Require(d.valid&&d.phase==tl::fea::type25::BatchPhase::Accepted&&d.source_instance_id==v.bindings->source_instance_id()&&
        d.owner_id==s.owner_id&&d.configuration_id==settings.configuration_id&&d.qualification_id==settings.qualification_id&&
        d.epoch==s.epoch&&d.time==s.time&&d.velocity_time==s.velocity_time&&d.element_count==v.connectors.count&&
        d.has_completed_interval==bool(s.epoch)&&d.accepted_force_assembled==bool(s.epoch),
        "Connector result is not the complete enclosing accepted interval");
    if(s.epoch)Require(d.base_epoch==s.reaction_base_epoch&&d.base_time==s.reaction_time&&
        d.kick_dt==s.reaction_kick_dt&&d.attempt==common.qeph.attempt&&d.base_velocity_time==common.qeph.base_velocity_time,
        "Connector reaction/force-history phase differs");
    else Require(!d.base_epoch&&!d.attempt&&d.base_time==0&&d.base_velocity_time==0&&d.kick_dt==0&&
        d.newly_failed_count==0,"Initial connector fabricates a completed interval");
    std::array<long double,4> work{},scale{};std::size_t active=0;double minimum_dt=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<v.connectors.count;++i) {
        const auto& p=v.connectors.elements[i];
        Require(tl::fea::type25::detail::ValidHistory(p.history)&&tl::math::fixed3::Orthonormal(p.frame.axes)&&
            tl::math::fixed3::Orthonormal(p.frame.midpoint_axes)&&std::isfinite(p.frame.length_m)&&p.frame.length_m>0&&
            std::isfinite(p.frame.midpoint_length_m)&&p.frame.midpoint_length_m>0&&std::isfinite(p.critical_dt_s)&&p.critical_dt_s>0,
            "Connector output frame/history/timestep is invalid");
        for(const auto& w:p.endpoints)Require(tl::math::fixed3::Finite(w.force_N)&&tl::math::fixed3::Finite(w.couple_Nm),
                                            "Connector output wrench is invalid");
        Require(std::isfinite(p.translation_stiffness_N_per_m)&&p.translation_stiffness_N_per_m>0&&
                std::isfinite(p.rotation_stiffness_Nm_per_rad)&&p.rotation_stiffness_Nm_per_rad>0,
                "Connector output elementary stiffness is invalid");
        active+=p.history.active;minimum_dt=std::min(minimum_dt,p.critical_dt_s);
        for(unsigned c=0;c<4;++c){work[c]+=p.history.internal_work_J[c];scale[c]+=std::abs(p.history.internal_work_J[c]);}
    }
    Require(d.active_count==active&&d.newly_failed_count<=v.connectors.count-active&&d.minimum_native_dt==minimum_dt,
            "Connector output activity/timestep reduction differs");
    for(unsigned c=0;c<4;++c) {
        const long double budget=512*std::numeric_limits<double>::epsilon()*scale[c];
        Require(std::isfinite(static_cast<double>(budget))&&std::abs(work[c]-d.internal_work_J[c])<=budget,
                "Connector output native work reduction differs");
    }
}
} // namespace crash::output::assembly::wall_fields
