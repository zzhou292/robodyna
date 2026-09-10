#include "State.h"
#include "NativeGuardAttribution.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "lib_src/elements/t3/T3History.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
#include "lib_src/math/Quaternion.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::source_assembly_dynamics {
namespace {
constexpr long double Roundoff=512*std::numeric_limits<double>::epsilon();
bool Nonnegative(double x) noexcept { return std::isfinite(x)&&x>=0; }
template<class D> Report Family(const D& d,const Config& c,const fe::NodalPreparedView& p,
                               const source_assembly::SourceAssemblyWallSettings& s) {
    if(!d.valid||!d.has_completed_interval||!d.accepted_force_assembled||d.kinetic_available||
       d.phase!=decltype(d.phase)::Prepared||d.usage!=decltype(d.usage)::CoupledForces||
       d.owner_id!=p.owner_id||d.base_epoch!=p.kinematics.base_epoch||d.epoch!=d.base_epoch+1||
       d.attempt!=p.attempt||d.configuration_id!=s.configuration_id||d.qualification_id!=s.qualification_id||
       d.time!=p.proposed_time||d.base_time!=p.base_time||d.velocity_time!=p.velocity_time||
       d.base_velocity_time!=p.base_velocity_time||d.kick_dt!=p.kick_dt||
       d.kinetic_translation!=0||d.kinetic_rotation!=0||d.kinetic_physical_isotropic!=0||d.kinetic_added_isotropic!=0)
        return Failure(Status::ComponentFailure,"Prepared native family source/interval/kinetic scope is invalid");
    const auto& e=c.deformation;
    for(auto pair:{std::pair<double,double>{d.maximum_displacement,e.maximum_displacement},
                  {d.maximum_absolute_strain,e.maximum_strain},{d.maximum_thickness_curvature,e.maximum_thickness_curvature}})
        if(!Nonnegative(pair.first)||pair.first>pair.second)
            return Failure(Status::EnvelopeFailure,"Source family deformation envelope exceeded",0,SIZE_MAX,pair.first,pair.second);
    if(!std::isfinite(d.minimum_area_ratio)||d.minimum_area_ratio<e.minimum_area_ratio||
       !std::isfinite(d.minimum_thickness_ratio)||d.minimum_thickness_ratio<e.minimum_thickness_ratio||
       !std::isfinite(d.minimum_native_dt)||!(d.minimum_native_dt>0)||
       c.fixed_dt>d.minimum_native_dt*e.maximum_native_dt_fraction)
        return Failure(Status::EnvelopeFailure,NativeMinimumGuardFailure);
    for(double x:{d.internal_work[0],d.internal_work[1],d.internal_work_increment[0],d.internal_work_increment[1],
                  d.internal_kick_work,d.internal_drift_work})
        if(!std::isfinite(x))return Failure(Status::EnvelopeFailure,"Native family work is nonfinite");
    return Success();
}
template<class D,class ParentAt> Report AttributeFamilyFailure(Report report,const D& d,const Config& c,
    std::size_t count,ParentAt parent_at) {
    if(report.status!=Status::EnvelopeFailure||report.message!=NativeMinimumGuardFailure)return report;
    const auto& e=c.deformation;
    const auto detail=AttributeNativeGuard({d.minimum_area_ratio,d.minimum_thickness_ratio,d.minimum_native_dt},
        {e.minimum_area_ratio,e.minimum_thickness_ratio,c.fixed_dt,e.maximum_native_dt_fraction},count,parent_at);
    report.message=detail.message;report.source_parent=detail.source;report.measured=detail.measured;report.limit=detail.limit;
    return report;
}
struct Work {
    std::array<long double,3> sum{},magnitude{},increment{},increment_magnitude{};
    void Add(unsigned c,double value,double delta) noexcept {
        sum[c]+=value;magnitude[c]+=std::abs(value);increment[c]+=delta;increment_magnitude[c]+=std::abs(delta);
    }
};
Report WorkAgreement(const Work& x,const std::array<long double,3>& old_magnitude,
    const std::array<double,3>& current,const std::array<double,3>& old,const std::array<double,3>& increment) {
    for(unsigned c=0;c<3;++c) {
        const auto magnitude=x.magnitude[c]+old_magnitude[c]+x.increment_magnitude[c];
        const long double allowance=Roundoff*magnitude+1e-18L;
        const long double residual=static_cast<long double>(current[c])-old[c]-increment[c];
        if(!std::isfinite(magnitude)||!std::isfinite(residual)||!std::isfinite(x.sum[c])||
           !std::isfinite(x.increment[c])||std::abs(residual)>allowance||
           std::abs(x.sum[c]-current[c])>allowance||std::abs(x.increment[c]-increment[c])>allowance)
            return Failure(Status::EnvelopeFailure,"Complete native parent/family work ledger disagrees",0,SIZE_MAX,
                           static_cast<double>(residual),static_cast<double>(allowance));
    }
    return Success();
}
template<class R,class F> Report Parent(const R& reference,const F& result,std::size_t arity,
    const fe::ShellBatchSectionState& section,const fe::ShellBatchSectionState& old_section,
    const fe::sections::PointParameters& material,std::uint64_t source,const Config& c,
    const fe::NodalPreparedView& p,Diagnostics& d,long double& plastic_work) {
    const auto& h=result.proposed_history;
    if(!h.prepared()||!h.matches_reference(reference)||h.stamp().sample_index!=p.kinematics.base_epoch+1||
       h.stamp().time!=p.proposed_time||result.kinematics.sample_index!=h.stamp().sample_index||
       result.kinematics.base_time!=p.base_time||result.kinematics.dt!=c.fixed_dt)
        return Failure(Status::SourceMismatch,"Prepared native parent lost exact reference/history identity",source);
    const auto& values=h.data();const double area=result.kinematics.area/reference.area;
    const double thickness=values.thickness/reference.input.thickness;
    const auto& e=c.deformation;
    if(!std::isfinite(area)||area<e.minimum_area_ratio||area>e.maximum_area_ratio||
       !std::isfinite(thickness)||thickness<e.minimum_thickness_ratio||thickness>e.maximum_thickness_ratio||values.active!=1)
        return Failure(Status::EnvelopeFailure,"Native parent area/thickness/activity envelope exceeded",source);
    d.maximum_area_ratio=std::max(d.maximum_area_ratio,area);d.maximum_thickness_ratio=std::max(d.maximum_thickness_ratio,thickness);
    for(std::size_t local=0;local<arity;++local) {
        const auto f=result.internal_force[local],m=result.internal_couple[local];
        for(double x:{f.x,f.y,f.z,m.x,m.y,m.z})if(!std::isfinite(x))
            return Failure(Status::EnvelopeFailure,"Native parent force or couple is nonfinite",source);
    }
    for(double x:values.stress)if(!std::isfinite(x))return Failure(Status::EnvelopeFailure,"Native stress is nonfinite",source);
    for(double x:values.strain_curvature)if(!std::isfinite(x))return Failure(Status::EnvelopeFailure,"Native strain is nonfinite",source);
    const auto& sd=section.diagnostics;
    for(double x:{section.cumulative_plastic_work_J,sd.plastic_work_density_increment,sd.maximum_plastic_strain,
                  sd.mean_plastic_strain,sd.minimum_tangent_ratio,sd.mean_tangent_ratio,
                  sd.mean_yield_before_pa,sd.last_point_yield_before_pa})
        if(!Nonnegative(x))return Failure(Status::EnvelopeFailure,"Native section diagnostic is nonfinite or negative",source);
    if(section.cumulative_plastic_work_J<old_section.cumulative_plastic_work_J||!material.curve.count||!material.curve.plastic_strain)
        return Failure(Status::EnvelopeFailure,"Native plastic work decreased or source curve is absent",source);
    const auto maximum=material.curve.plastic_strain[material.curve.count-1];bool yielded=false;
    for(unsigned point=0;point<3;++point) {
        const auto& state=section.history.point[point];
        if(!std::isfinite(state.plastic_strain)||state.plastic_strain<old_section.history.point[point].plastic_strain||
           state.plastic_strain>maximum||!Nonnegative(state.filtered_rate_per_s))
            return Failure(Status::EnvelopeFailure,"Native section history decreased or exceeded its own source curve",source);
        for(double stress:state.stress)if(!std::isfinite(stress))
            return Failure(Status::EnvelopeFailure,"Native thickness-point stress is nonfinite",source);
        d.maximum_plastic_strain=std::max(d.maximum_plastic_strain,state.plastic_strain);
        if(state.plastic_strain>0) {++d.yielded_points;yielded=true;}
    }
    if(!fe::sections::MatchesLayeredJ2Resultants(section.history,values))
        return Failure(Status::ComponentFailure,"Native material resultants differ from their three thickness points",source);
    d.yielded_parents+=yielded;plastic_work+=section.cumulative_plastic_work_J;
    return Success();
}
}
Report SourceAssemblyWallCase::Impl::CheckShells() {
    auto& next=candidate();const auto& old=accepted();auto& d=next.diagnostics;
    if(!d.shells.valid)return Failure(Status::ComponentFailure,"Missing joined native diagnostics");
    const auto& b=bindings.shells();
    auto r=Family(d.shells.qeph,config,prepared,*setup.settings());
    if(!r)return AttributeFamilyFailure(r,d.shells.qeph,config,quads(),[&](std::size_t i) {
        const auto& f=next.parents.qeph[i];const auto& reference=b.qeph_reference(i);
        return NativeGuardParent{b.qeph_source_id(i),{f.kinematics.area/reference.area,
            f.proposed_history.data().thickness/reference.input.thickness,f.diagnostics.unscaled_element_dt}};
    });
    r=Family(d.shells.t3,config,prepared,*setup.settings());
    if(!r)return AttributeFamilyFailure(r,d.shells.t3,config,triangles(),[&](std::size_t i) {
        const auto& f=next.parents.t3[i];const auto& reference=b.t3_reference(i);
        return NativeGuardParent{b.t3_source_id(i),{f.kinematics.area/reference.area,
            f.proposed_history.data().thickness/reference.input.thickness,f.diagnostics.unscaled_element_dt}};
    });
    Work qw,tw;long double plastic_work=0;
    for(std::size_t e=0;e<quads();++e) {
        fe::sections::PointParameters material;
        if(!bindings.materials().Parameters(fe::ShellBindingFamily::Qeph,e,&material))
            return Failure(Status::SourceMismatch,"Missing original QEPH material mapping",b.qeph_source_id(e));
        const auto& f=next.parents.qeph[e];
        r=Parent(b.qeph_reference(e),f,4,next.parents.qsection[e],old.parents.qsection[e],material,
            b.qeph_source_id(e),config,prepared,d,plastic_work);if(!r)return r;
        for(unsigned c=0;c<2;++c)qw.Add(c,f.proposed_history.data().internal_work[c],f.diagnostics.internal_work_increment[c]);
        qw.Add(2,f.proposed_history.data().hourglass_viscous_work,f.diagnostics.hourglass_viscous_work_increment);
    }
    for(std::size_t e=0;e<triangles();++e) {
        fe::sections::PointParameters material;
        if(!bindings.materials().Parameters(fe::ShellBindingFamily::T3,e,&material))
            return Failure(Status::SourceMismatch,"Missing original T3 material mapping",b.t3_source_id(e));
        const auto& f=next.parents.t3[e];
        r=Parent(b.t3_reference(e),f,3,next.parents.tsection[e],old.parents.tsection[e],material,
            b.t3_source_id(e),config,prepared,d,plastic_work);if(!r)return r;
        for(unsigned c=0;c<2;++c)tw.Add(c,f.proposed_history.data().internal_work[c],f.diagnostics.internal_work_increment[c]);
    }
    const auto& qd=d.shells.qeph;const auto& oq=old.diagnostics.shells.qeph;
    r=WorkAgreement(qw,old.qwork_magnitude,{qd.internal_work[0],qd.internal_work[1],qd.hourglass_viscous_work},
        {oq.internal_work[0],oq.internal_work[1],oq.hourglass_viscous_work},
        {qd.internal_work_increment[0],qd.internal_work_increment[1],qd.hourglass_viscous_work_increment});if(!r)return r;
    const auto& td=d.shells.t3;const auto& ot=old.diagnostics.shells.t3;
    r=WorkAgreement(tw,old.twork_magnitude,{td.internal_work[0],td.internal_work[1],0},
        {ot.internal_work[0],ot.internal_work[1],0},{td.internal_work_increment[0],td.internal_work_increment[1],0});if(!r)return r;
    next.qwork_magnitude=qw.magnitude;next.twork_magnitude=tw.magnitude;
    d.native_internal_work=static_cast<double>(qw.sum[0]+qw.sum[1]+qw.sum[2]+tw.sum[0]+tw.sum[1]);
    d.cumulative_plastic_work=static_cast<double>(plastic_work);
    if(!std::isfinite(d.native_internal_work)||!Nonnegative(d.cumulative_plastic_work))
        return Failure(Status::EnvelopeFailure,"Complete native work reduction is nonfinite");
    return CheckRotations();
}
} // namespace crash::cases::source_assembly_dynamics
