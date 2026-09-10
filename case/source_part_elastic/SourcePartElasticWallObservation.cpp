#include "SourcePartElasticWallInternal.h"
#include "collision/Q4ContactBounds.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::source_part_elastic {
namespace {
namespace qb=wall_contact::q4_bounds;
constexpr long double Roundoff=512*std::numeric_limits<double>::epsilon();
bool AddImpulse(double a,double ae,double b,double be,double& value,double& error) {
    if(!std::isfinite(a)||a<0||!std::isfinite(ae)||ae<0||!std::isfinite(b)||b<0||!std::isfinite(be)||be<0) return false;
    qb::Interval x,y,sum;
    if(!qb::AddScalar(a,-ae,false,&x.lower)||!qb::AddScalar(a,ae,true,&x.upper)||
       !qb::AddScalar(b,-be,false,&y.lower)||!qb::AddScalar(b,be,true,&y.upper)) return false;
    x.lower=std::max(0.,x.lower); y.lower=std::max(0.,y.lower);
    wall_contact::Q4CertifiedIntegral result;
    if(!qb::Add(x,y,&sum)||!qb::Certify(a+b,sum,&result)) return false;
    value=result.value; error=result.error; return true;
}
bool FiniteNonnegative(double value) { return std::isfinite(value)&&value>=0; }
bool AddSigned(double a,double ae,double b,double be,double& value,double& error) {
    if(!std::isfinite(a)||!std::isfinite(b)||!FiniteNonnegative(ae)||!FiniteNonnegative(be)) return false;
    qb::Interval x,y,sum;
    double lower_error=0,upper_error=0;
    value=a+b;
    if(!qb::AddScalar(a,-ae,false,&x.lower)||!qb::AddScalar(a,ae,true,&x.upper)||
       !qb::AddScalar(b,-be,false,&y.lower)||!qb::AddScalar(b,be,true,&y.upper)||!qb::Add(x,y,&sum)||
       !qb::AbsoluteDifferenceUpper(value,sum.lower,&lower_error)||
       !qb::AbsoluteDifferenceUpper(value,sum.upper,&upper_error)) return false;
    error=std::max(lower_error,upper_error); return true;
}
}
std::array<double,3> CarriedAngularMomentum(const fe::ShellBatchBinding& binding,const Snapshot& state) noexcept {
    std::array<long double,3> angular{};
    for(unsigned n=0;n<NodeCount;++n) {
        const auto& m=binding.nodes()[n].native;
        for(unsigned a=0;a<3;++a) {
            const unsigned b=(a+1)%3,c=(a+2)%3;
            angular[a]+=static_cast<long double>(m.mass)*(static_cast<long double>(state.position[3*n+b])*state.velocity[3*n+c]-
                static_cast<long double>(state.position[3*n+c])*state.velocity[3*n+b])+static_cast<long double>(m.isotropic_inertia)*state.omega[3*n+a];
        }
    }
    return {static_cast<double>(angular[0]),static_cast<double>(angular[1]),static_cast<double>(angular[2])};
}
Report SourcePartElasticCase::Impl::GatherWallEndpoint() {
    const auto& results=wall->trial;
    const auto& d=results.diagnostics;
    const auto& weights=*wall->contributor.setup()->source_geometry()->weights();
    const auto certificate=[](wall_contact::Q4CertifiedIntegral value) {
        return wall_contact::nodal_wall_detail::Certificate(value);
    };
    if(!certificate(d.resultant)||!certificate(d.potential))
        return Failure(Status::EnvelopeFailure,"Contact global force/potential certificate is invalid");
    for(double value:{d.kick_work_roundoff,d.drift_work_roundoff,d.work_uncertainty,d.quadratic_work_upper,
        d.wall_kick_impulse,d.wall_kick_impulse_error,d.maximum_penetration})
        if(!FiniteNonnegative(value)) return Failure(Status::EnvelopeFailure,"Contact interval bound is nonfinite or negative");
    for(double value:{d.kick_work,d.drift_work,d.conservative_defect})
        if(!std::isfinite(value)) return Failure(Status::EnvelopeFailure,"Contact interval work is nonfinite");
    double defect_allowance=0;
    if(!qb::AddScalar(d.quadratic_work_upper,d.work_uncertainty,true,&defect_allowance)||
       d.conservative_defect < -d.work_uncertainty||d.conservative_defect>defect_allowance)
        return Failure(Status::EnvelopeFailure,"Contact conservative-work certificate failed",d.conservative_defect,defect_allowance);
    for(unsigned e=0;e<weights.parent_count();++e) {
        const auto& p=results.parents[e]; const auto& expected=weights.parent(e);
        if(!p.valid||p.parent_element_id!=expected.parent_element_id||p.parent_face_id!=expected.parent_face_id||
           p.feature_id!=expected.feature_id||p.family!=expected.family||p.arity!=expected.arity)
            return Failure(Status::ComponentFailure,"Contact parent identity differs from original source weights");
    }
    wall->trial_metrics.active_nodes=0;
    for(unsigned n=0;n<NodeCount;++n) {
        const auto& point=results.nodes[n];
        if(!point.valid||point.node!=n||weights.node(n).node!=n||point.base_epoch!=accepted.stamp.epoch||
           point.attempt!=prepared.attempt||point.fixed||!certificate(point.force)||!certificate(point.potential)||
           point.force_world.x!=-point.force.value||point.force_world.y!=0||point.force_world.z!=0)
            return Failure(Status::ComponentFailure,"Contact unique node identity, force or potential is invalid");
        // The contributor already assembled every parent share into this one
        // nodal result. Adding parent forces here would count contact twice.
        const auto j=3*n;
        qb::Interval exact;
        double lower_error=0,upper_error=0;
        const double before=endpoint_force[j],added=before+point.force_world.x;
        if(!qb::Add({before,before},{point.force_world.x,point.force_world.x},&exact)||
           !qb::AbsoluteDifferenceUpper(added,exact.lower,&lower_error)||
           !qb::AbsoluteDifferenceUpper(added,exact.upper,&upper_error)||
           !qb::AddScalar(point.force.error,std::max(lower_error,upper_error),true,&wall->endpoint_force_uncertainty[n]))
            return Failure(Status::EnvelopeFailure,"Contact endpoint addition uncertainty cannot be enclosed");
        endpoint_force[j]=added; // Zero offset/friction: no Y/Z force or direct couple.
        if(point.force.value>0) ++wall->trial_metrics.active_nodes;
    }
    return Success();
}
Report SourcePartElasticCase::Impl::ObserveWall(long double kinetic) {
    const auto& d=wall->trial.diagnostics;
    auto& metrics=wall->trial_metrics;
    const auto& previous=wall->accepted_metrics;
    if(!AddImpulse(previous.wall_kick_impulse,previous.wall_kick_impulse_error,d.wall_kick_impulse,
        d.wall_kick_impulse_error,metrics.wall_kick_impulse,metrics.wall_kick_impulse_error))
        return Failure(Status::EnvelopeFailure,"Cumulative wall kick impulse cannot be enclosed");
    metrics.carried_angular_momentum=CarriedAngularMomentum(binding,trial);
    const double moment[]{d.wall_kick_moment.x,d.wall_kick_moment.y,d.wall_kick_moment.z};
    const double moment_error[]{d.wall_kick_moment_error.x,d.wall_kick_moment_error.y,d.wall_kick_moment_error.z};
    for(unsigned a=0;a<3;++a)
        if(!std::isfinite(metrics.carried_angular_momentum[a])||!AddSigned(previous.wall_kick_moment[a],
            previous.wall_kick_moment_error[a],moment[a],moment_error[a],metrics.wall_kick_moment[a],metrics.wall_kick_moment_error[a]))
            return Failure(Status::EnvelopeFailure,"Carried angular momentum or wall moment observation is nonfinite");
    std::array<long double,3> momentum{},initial_momentum{},momentum_magnitude{};
    double kinetic_uncertainty=0;
    for(unsigned n=0;n<NodeCount;++n) {
        const double mass=binding.nodes()[n].native.mass;
        for(unsigned a=0;a<3;++a) {
            const long double p=static_cast<long double>(mass)*trial.velocity[3*n+a];
            const long double p0=static_cast<long double>(mass)*config.initial_velocity[a];
            momentum[a]+=p; initial_momentum[a]+=p0;
            momentum_magnitude[a]+=std::abs(p)+std::abs(p0);
        }
        // Force certificates and rounded endpoint force addition both affect
        // the derived synchronized X velocity. Propagate that uncertainty to K.
        qb::Interval dv,linear,square,total;
        const double force_error=wall->endpoint_force_uncertainty[n];
        if(!qb::Scale({force_error,force_error},.5*config.dt,&dv)||!qb::DividePositive(dv,mass,&dv)||
           !qb::Scale(dv,std::abs(trial.synchronized_velocity[3*n]),&linear)||
           !qb::MultiplyPositive(dv,dv,&square)||!qb::Scale(square,.5,&square)||
           !qb::Add(linear,square,&total)||!qb::Scale(total,mass,&total)||
           !qb::AddScalar(kinetic_uncertainty,total.upper,true,&kinetic_uncertainty))
            return Failure(Status::EnvelopeFailure,"Synchronized contact kinetic uncertainty cannot be enclosed");
    }
    metrics.synchronized_kinetic_uncertainty=kinetic_uncertainty;
    for(unsigned a=0;a<3;++a) {
        const long double impulse=a==0?metrics.wall_kick_impulse:0;
        const long double impulse_error=a==0?metrics.wall_kick_impulse_error:0;
        const long double allowance=Roundoff*(accepted.stamp.epoch+2.L)*(momentum_magnitude[a]+std::abs(impulse))+impulse_error+1e-18L;
        const long double residual=momentum[a]-initial_momentum[a]+impulse;
        metrics.carried_momentum_residual[a]=static_cast<double>(residual);
        metrics.carried_momentum_allowance[a]=static_cast<double>(allowance);
        if(!std::isfinite(residual)||!std::isfinite(allowance)||std::abs(residual)>allowance)
            return Failure(Status::EnvelopeFailure,"Carried source momentum disagrees with carried wall impulse",static_cast<double>(residual),static_cast<double>(allowance));
    }
    long double source_work_magnitude=0;
    for(unsigned c=0;c<3;++c) source_work_magnitude+=trial_q_work_magnitude[c]+trial_t_work_magnitude[c];
    const long double arithmetic=Roundoff*(std::abs(kinetic)+source_work_magnitude+std::abs(d.potential.value)+initial_kinetic);
    const long double uncertainty=kinetic_uncertainty+d.potential.error+arithmetic;
    auto& output=trial.diagnostics;
    const long double residual=kinetic+output.total_internal_work+d.potential.value-initial_kinetic;
    const long double allowance=config.maximum_energy_residual+config.relative_energy_residual*initial_kinetic;
    const long double error_upper=std::abs(residual)+uncertainty;
    output.energy_residual=static_cast<double>(residual);
    metrics.physical_energy_uncertainty=static_cast<double>(uncertainty);
    metrics.energy_allowance=static_cast<double>(allowance);
    if(!std::isfinite(error_upper)||!std::isfinite(allowance)||error_upper>allowance)
        return Failure(Status::EnvelopeFailure,"Source wall physical energy envelope exceeded",static_cast<double>(error_upper),metrics.energy_allowance);
    const long double negative_work_allowance=.01L*initial_kinetic;
    if(output.total_internal_work-arithmetic < -negative_work_allowance)
        return Failure(Status::EnvelopeFailure,"Source wall native internal work is below its frozen floor",output.total_internal_work,static_cast<double>(negative_work_allowance));
    if(metrics.active_nodes) {
        const auto epoch=accepted.stamp.epoch+1;
        if(!metrics.first_contact_epoch) metrics.first_contact_epoch=epoch;
        metrics.last_contact_epoch=epoch;
        ++metrics.contact_intervals;
    }
    return Success();
}
} // namespace crash::cases::source_part_elastic
