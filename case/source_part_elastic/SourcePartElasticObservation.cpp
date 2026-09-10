#include "SourcePartElasticInternal.h"
#include "lib_src/math/Quaternion.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::source_part_elastic {
namespace {
constexpr long double Roundoff=512*std::numeric_limits<double>::epsilon();
double Component(tl::math::Vec3 v,unsigned axis) { return axis==0?v.x:axis==1?v.y:v.z; }
template<class D>
Report FamilyEnvelope(const D& d,const Config& c) {
    if(!d.valid||!d.accepted_force_assembled||d.kinetic_available||!d.has_completed_interval)
        return Failure(Status::ComponentFailure,"Incomplete coupled source-family diagnostics");
    for(const auto item:{std::pair<double,double>{d.maximum_displacement,c.maximum_displacement},
            {d.maximum_absolute_strain,c.maximum_strain},{d.maximum_thickness_curvature,c.maximum_thickness_curvature}})
        if(!std::isfinite(item.first)||item.first<0||item.first>item.second)
            return Failure(Status::EnvelopeFailure,"Source shell deformation envelope exceeded",item.first,item.second);
    if(!std::isfinite(d.minimum_area_ratio)||d.minimum_area_ratio<c.minimum_area_ratio)
        return Failure(Status::EnvelopeFailure,"Source shell minimum area ratio exceeded",d.minimum_area_ratio,c.minimum_area_ratio);
    if(!std::isfinite(d.minimum_thickness_ratio)||d.minimum_thickness_ratio<c.minimum_thickness_ratio)
        return Failure(Status::EnvelopeFailure,"Source shell minimum thickness ratio exceeded",d.minimum_thickness_ratio,c.minimum_thickness_ratio);
    if(!std::isfinite(d.minimum_native_dt)||d.minimum_native_dt<=0||c.dt>d.minimum_native_dt*c.maximum_native_dt_fraction)
        return Failure(Status::EnvelopeFailure,"Source native timestep diagnostic guard exceeded",c.dt,d.minimum_native_dt*c.maximum_native_dt_fraction);
    return Success();
}
struct WorkTotals {
    std::array<long double,3> sum{},magnitude{},increment{},increment_magnitude{};
    void Add(unsigned field,double work,double change) {
        sum[field]+=work; magnitude[field]+=std::abs(work);
        increment[field]+=change; increment_magnitude[field]+=std::abs(change);
    }
};
Report WorkAgreement(const WorkTotals& actual,const std::array<long double,3>& previous_magnitude,
                     const std::array<double,3>& work,const std::array<double,3>& previous,
                     const std::array<double,3>& increment) {
    for(unsigned c=0;c<3;++c) {
        const long double magnitude=actual.magnitude[c]+previous_magnitude[c]+actual.increment_magnitude[c];
        const long double allowance=Roundoff*magnitude+1e-18L;
        const long double residual=static_cast<long double>(work[c])-previous[c]-increment[c];
        if(!std::isfinite(magnitude)||!std::isfinite(residual)||!std::isfinite(actual.sum[c])||
           !std::isfinite(actual.increment[c])||std::abs(residual)>allowance||
           std::abs(actual.sum[c]-work[c])>allowance||std::abs(actual.increment[c]-increment[c])>allowance)
            return Failure(Status::EnvelopeFailure,"Native parent/family source-work ledger mismatch",static_cast<double>(residual),static_cast<double>(allowance));
    }
    return Success();
}
}
Report SourcePartElasticCase::Impl::Observe() {
    auto& out=trial.diagnostics;
    const auto& shells=out.shells;
    const auto& old=accepted.diagnostics;
    if(!shells.valid) return Failure(Status::ComponentFailure,"Missing common shell diagnostics");
    auto report=FamilyEnvelope(shells.qeph,config);
    if(!report) return report;
    report=FamilyEnvelope(shells.t3,config);
    if(!report) return report;
    const auto& qd=shells.qeph;
    const double base_scale=PulseScale(prepared.base_time,config.pulse_duration);
    const double end_scale=PulseScale(prepared.proposed_time,config.pulse_duration);
    for(std::size_t j=0;j<endpoint_force.size();++j) endpoint_force[j]=pulse_force[j]*end_scale;
    endpoint_couple.fill(0);
    WorkTotals qwork,twork;
    auto gather=[&](const auto& reference,const auto& nodes,const auto& result,std::uint64_t parent) -> Report {
        const auto& h=result.proposed_history.data();
        if(!result.proposed_history.prepared()) return Failure(Status::ComponentFailure,"Unprepared source history",0,0,parent);
        const double area=result.kinematics.area/reference.area;
        const double thickness=h.thickness/reference.input.thickness;
        if(!std::isfinite(area)||area<config.minimum_area_ratio||area>config.maximum_area_ratio)
            return Failure(Status::EnvelopeFailure,"Native parent area ratio envelope exceeded",area,config.maximum_area_ratio,parent);
        if(!std::isfinite(thickness)||thickness<config.minimum_thickness_ratio||thickness>config.maximum_thickness_ratio)
            return Failure(Status::EnvelopeFailure,"Native parent thickness ratio envelope exceeded",thickness,config.maximum_thickness_ratio,parent);
        out.maximum_area_ratio=std::max(out.maximum_area_ratio,area);
        out.maximum_thickness_ratio=std::max(out.maximum_thickness_ratio,thickness);
        for(unsigned local=0;local<nodes.size();++local) for(unsigned a=0;a<3;++a) {
            const double force=Component(result.internal_force[local],a),couple=Component(result.internal_couple[local],a);
            if(!std::isfinite(force)||!std::isfinite(couple))
                return Failure(Status::EnvelopeFailure,"Nonfinite native force or couple",0,0,parent);
            endpoint_force[3*nodes[local]+a]-=force;
            endpoint_couple[3*nodes[local]+a]-=couple;
        }
        return Success();
    };
    for(std::size_t e=0;e<source::Q4Count;++e) {
        report=gather(binding.qeph_reference(e),binding.qeph_nodes(e),qresult[e],binding.qeph_source_id(e));
        if(!report) return report;
        const auto& h=qresult[e].proposed_history.data();
        for(unsigned c=0;c<2;++c) qwork.Add(c,h.internal_work[c],qresult[e].diagnostics.internal_work_increment[c]);
        qwork.Add(2,h.hourglass_viscous_work,qresult[e].diagnostics.hourglass_viscous_work_increment);
    }
    for(std::size_t e=0;e<source::T3Count;++e) {
        report=gather(binding.t3_reference(e),binding.t3_nodes(e),tresult[e],binding.t3_source_id(e));
        if(!report) return report;
        for(unsigned c=0;c<2;++c) twork.Add(c,tresult[e].proposed_history.data().internal_work[c],tresult[e].diagnostics.internal_work_increment[c]);
    }
    report=WorkAgreement(qwork,accepted_q_work_magnitude,
        {qd.internal_work[0],qd.internal_work[1],qd.hourglass_viscous_work},
        {old.shells.qeph.internal_work[0],old.shells.qeph.internal_work[1],old.shells.qeph.hourglass_viscous_work},
        {qd.internal_work_increment[0],qd.internal_work_increment[1],qd.hourglass_viscous_work_increment});
    if(!report) return report;
    report=WorkAgreement(twork,accepted_t_work_magnitude,
        {shells.t3.internal_work[0],shells.t3.internal_work[1],0},
        {old.shells.t3.internal_work[0],old.shells.t3.internal_work[1],0},
        {shells.t3.internal_work_increment[0],shells.t3.internal_work_increment[1],0});
    if(!report) return report;
    trial_q_work_magnitude=qwork.magnitude; trial_t_work_magnitude=twork.magnitude;
    out.total_internal_work=static_cast<double>(qwork.sum[0]+qwork.sum[1]+qwork.sum[2]+twork.sum[0]+twork.sum[1]);
    long double kick=0,drift=0,absolute_drift=0,kinetic=0,raw_kinetic=0;
    std::array<long double,3> centroid_delta{};
    long double mass=0;
    for(std::size_t n=0;n<NodeCount;++n) {
        const auto& m=binding.nodes()[n].native;
        mass+=m.mass;
        const auto* orientation=trial.orientation.data()+4*n;
        if(!tl::math::UnitQuaternion({orientation[0],orientation[1],orientation[2],orientation[3]}))
            return Failure(Status::EnvelopeFailure,"Candidate quaternion is not finite and unit length");
        const double rotation=2*std::atan2(std::hypot(std::hypot(orientation[1],orientation[2]),orientation[3]),std::abs(orientation[0]));
        out.maximum_rotation=std::max(out.maximum_rotation,rotation);
        for(unsigned a=0;a<3;++a) {
            const auto j=3*n+a;
            for(double field:{trial.position[j],trial.velocity[j],trial.omega[j],endpoint_force[j],endpoint_couple[j]})
                if(!std::isfinite(field)) return Failure(Status::EnvelopeFailure,"Nonfinite candidate nodal field or endpoint load");
            const long double force0=pulse_force[j]*base_scale,force1=pulse_force[j]*end_scale;
            const long double dx=static_cast<long double>(trial.position[j])-accepted.position[j];
            kick+=force0*prepared.kick_dt*(static_cast<long double>(accepted.velocity[j])+trial.velocity[j])*.5L;
            const long double component_work=.5L*(force0+force1)*dx;
            drift+=component_work; absolute_drift+=std::abs(component_work);
            trial.synchronized_velocity[j]=static_cast<double>(trial.velocity[j]+.5L*config.dt*endpoint_force[j]/m.mass);
            trial.synchronized_omega[j]=static_cast<double>(trial.omega[j]+.5L*config.dt*endpoint_couple[j]/m.isotropic_inertia);
            const long double v=trial.synchronized_velocity[j],w=trial.synchronized_omega[j];
            kinetic+=.5L*m.mass*v*v+.5L*m.isotropic_inertia*w*w;
            raw_kinetic+=.5L*m.mass*trial.velocity[j]*trial.velocity[j]+.5L*m.isotropic_inertia*trial.omega[j]*trial.omega[j];
            centroid_delta[a]+=m.mass*(static_cast<long double>(trial.position[j])-source.coordinates()[j]);
        }
    }
    if(out.maximum_rotation>config.maximum_rotation)
        return Failure(Status::EnvelopeFailure,"Source rotation envelope exceeded",out.maximum_rotation,config.maximum_rotation);
    for(auto& x:centroid_delta) x/=mass;
    for(std::size_t n=0;n<NodeCount;++n) {
        long double square=0;
        for(unsigned a=0;a<3;++a) {
            const long double x=static_cast<long double>(trial.position[3*n+a])-source.coordinates()[3*n+a]-centroid_delta[a];
            square+=x*x;
        }
        out.max_relative_displacement=std::max(out.max_relative_displacement,static_cast<double>(std::sqrt(square)));
    }
    out.synchronized_kinetic=static_cast<double>(kinetic);
    out.external_kick_work=static_cast<double>(old.external_kick_work+kick);
    out.external_drift_work=static_cast<double>(old.external_drift_work+drift);
    out.absolute_external_drift_work=static_cast<double>(old.absolute_external_drift_work+absolute_drift);
    const long double k0=static_cast<long double>(shells.base_kinetic.translation)+shells.base_kinetic.rotation;
    const long double k1=static_cast<long double>(shells.kinetic.translation)+shells.kinetic.rotation;
    const long double internal_kick=static_cast<long double>(qd.internal_kick_work)+shells.t3.internal_kick_work;
    const long double allowance=Roundoff*(std::abs(k0)+std::abs(k1)+std::abs(kick)+std::abs(internal_kick))+1e-18L;
    out.kinetic_work_allowance=static_cast<double>(allowance);
    out.kinetic_work_residual=static_cast<double>(k1-k0-kick-internal_kick);
    if(!std::isfinite(raw_kinetic)||std::abs(raw_kinetic-k1)>Roundoff*(std::abs(raw_kinetic)+std::abs(k1))+1e-18L)
        return Failure(Status::EnvelopeFailure,"Independent nodal kinetic sum disagrees with publication");
    if(!std::isfinite(out.kinetic_work_residual)||std::abs(out.kinetic_work_residual)>allowance)
        return Failure(Status::EnvelopeFailure,"Complete discrete kinetic-work identity failed",out.kinetic_work_residual,out.kinetic_work_allowance);
    out.energy_residual=static_cast<double>(kinetic+out.total_internal_work-out.external_drift_work);
    const double energy_allowance=config.maximum_energy_residual+config.relative_energy_residual*out.absolute_external_drift_work;
    if(!std::isfinite(out.energy_residual)||!std::isfinite(energy_allowance)||std::abs(out.energy_residual)>energy_allowance)
        return Failure(Status::EnvelopeFailure,"Synchronized source-part energy envelope exceeded",out.energy_residual,energy_allowance);
    return Success();
}
} // namespace crash::cases::source_part_elastic
