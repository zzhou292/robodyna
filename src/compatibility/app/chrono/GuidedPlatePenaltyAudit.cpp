#include "GuidedPlatePenaltyAudit.h"
#include "GuidedPlatePenaltyContact.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace crash::reference {
namespace {
namespace detail=penalty_detail;
namespace bounds=tlfea::contact::q4_bounds;
namespace shell=tl::fea::reissner;
using Status=ElasticCouponStatus;
using Interval=tlfea::contact::Q4IntegralInterval;
constexpr double EnergyFraction=.01;
constexpr double ForceRelativeAllowance=1e-8;
constexpr double DerivativeAbsoluteAllowance=1e-7; // N, plus relative and measured certificate terms.

void Require(bool condition,const char* error) { if (!condition) throw std::runtime_error(error); }
double Magnitude(shell::Vec3 v) { return std::max({std::abs(v.x),std::abs(v.y),std::abs(v.z)}); }
double EnergyAllowance(double reference,double a,double b) {
    constexpr double dimensional=ElasticCouponData::young_modulus*ElasticCouponData::thickness*
        ElasticCouponData::length*ElasticCouponData::width;
    return ForceRelativeAllowance*std::max({reference,std::abs(a),std::abs(b)})+
        128*std::numeric_limits<double>::epsilon()*dimensional;
}
void EvaluateShell(const GuidedPlateModel& model,const ElasticCouponConfiguration& configuration,double initial_energy,
                   ElasticCouponEvaluation& tl,ElasticCouponEvaluation& chrono,std::string& error) {
    if (model.shell().EvaluateTL(configuration,tl,error)!=Status::kSuccess ||
        model.shell().EvaluateChrono(configuration,chrono,error)!=Status::kSuccess) throw std::runtime_error(error);
    Require(std::isfinite(tl.energy)&&std::isfinite(chrono.energy)&&tl.energy>=0&&chrono.energy>=0&&
        std::abs(tl.energy-chrono.energy)<=EnergyAllowance(initial_energy,tl.energy,chrono.energy),
        "Penalty screen Chrono/TL energy agreement failed");
    constexpr double force_scale=ElasticCouponData::young_modulus*ElasticCouponData::thickness*ElasticCouponData::width;
    for (unsigned n=0;n<kCouponNodes;++n) {
        const double force=ForceRelativeAllowance*std::max(Magnitude(tl.force[n]),Magnitude(chrono.force[n]))+
            128*std::numeric_limits<double>::epsilon()*force_scale;
        const double moment=ForceRelativeAllowance*std::max(Magnitude(tl.couple[n]),Magnitude(chrono.couple[n]))+
            128*std::numeric_limits<double>::epsilon()*force_scale*ElasticCouponData::length;
        Require(Magnitude(shell::detail::Subtract(tl.force[n],chrono.force[n]))<=force&&
            Magnitude(shell::detail::Subtract(tl.couple[n],chrono.couple[n]))<=moment,
            "Penalty screen Chrono/TL force agreement failed");
    }
}
Interval Depth(const GuidedPlateModel& model,const ElasticCouponConfiguration& x) {
    Interval depth;
    for (const auto& point:x.position) {
        Interval gap; Require(bounds::Difference(point.x,model.wall().wall_x(),&gap),"Penalty screen depth subtraction failed");
        depth.lower=std::max(depth.lower,gap.lower); depth.upper=std::max(depth.upper,gap.upper);
    }
    return depth;
}
struct Point {
    ElasticCouponEvaluation tl,chrono;
    detail::ContactSample contact;
};
Point Evaluate(const GuidedPlateModel& model,const GuidedPlateModalReport& modal,double scale,
               detail::ContactScratch& scratch,std::string& error) {
    ElasticCouponConfiguration configuration;
    if (ApplyGuidedPlateIncrement(model.shell().data().reference_configuration,modal.initial_mode_increment,
            scale,configuration,error)!=Status::kSuccess) throw std::runtime_error(error);
    Point out; EvaluateShell(model,configuration,modal.initial_elastic_energy,out.tl,out.chrono,error);
    if (detail::Contact(model,configuration,scratch,out.contact,error)!=Status::kSuccess) throw std::runtime_error(error);
    return out;
}
double Restoring(const ElasticCouponEvaluation& value,const std::array<double,kGuidedPlateDofs>& direction) {
    double total=0;
    for (unsigned free=0;free<kCouponFreeNodes.size();++free) {
        const auto node=kCouponFreeNodes[free]; const unsigned i=4*free;
        total+=value.force[node].x*direction[i]+value.couple[node].x*direction[i+1]+
            value.couple[node].y*direction[i+2]+value.couple[node].z*direction[i+3];
    }
    return total;
}
GuidedPlatePenaltyDerivative Derivative(const GuidedPlateModel& model,const GuidedPlateModalReport& modal,
    double scale,double peak,double h,bool backward,const Point& base,detail::ContactScratch& scratch,std::string& error) {
    // Amplitude a increases into the wall; original modal increments point out.
    std::array<double,kGuidedPlateDofs> direction;
    for (unsigned i=0;i<kGuidedPlateDofs;++i) direction[i]=-modal.initial_mode_increment[i]/peak;
    const auto minus=Evaluate(model,modal,scale+h/peak,scratch,error);
    const auto other=Evaluate(model,modal,backward?scale+2*h/peak:scale-h/peak,scratch,error);
    auto energy=[](const Point& p) { return p.tl.energy+p.contact.potential.value; };
    const double numerator=backward?3*energy(base)-4*energy(minus)+energy(other):energy(other)-energy(minus);
    const double contact_error=backward?3*base.contact.potential.error+4*minus.contact.potential.error+other.contact.potential.error:
        minus.contact.potential.error+other.contact.potential.error;
    double restoring=Restoring(base.tl,direction),force_error=0;
    for (unsigned free=0;free<kCouponFreeNodes.size();++free) {
        const auto node=kCouponFreeNodes[free]; const double weight=direction[4*free];
        const auto truth=base.contact.nodal_magnitude[node];
        restoring+=base.contact.force_x[node]*weight;
        force_error+=std::abs(weight)*std::max(std::abs(-truth.lower-base.contact.force_x[node]),
                                             std::abs(-truth.upper-base.contact.force_x[node]));
    }
    GuidedPlatePenaltyDerivative out; out.step_m=h; out.backward=backward;
    out.derivative_N=numerator/(2*h); out.restoring_force_N=restoring;
    out.contact_uncertainty_N=contact_error/(2*h)+force_error;
    // Observational derivative check: second-order truncation plus the actual
    // C2 energy/force certificate and subtraction roundoff. No dynamics gate
    // or certificate tolerance is changed by this finite-difference allowance.
    const double roundoff=128*std::numeric_limits<double>::epsilon()*
        (std::abs(energy(base))+std::abs(energy(minus))+std::abs(energy(other)))/h;
    out.allowance_N=DerivativeAbsoluteAllowance+patch_audit::DerivativeTolerance*
        std::max(std::abs(out.derivative_N),std::abs(restoring))+out.contact_uncertainty_N+roundoff;
    out.error_N=std::abs(out.derivative_N+restoring);
    Require(std::isfinite(out.allowance_N)&&std::isfinite(out.error_N)&&out.error_N<=out.allowance_N,
        "Penalty screen force/potential derivative agreement failed");
    return out;
}
GuidedPlatePenaltySample Sample(const GuidedPlateModel& model,const GuidedPlateModalReport& modal,double depth,
                               bool cap,detail::ContactScratch& scratch,std::string& error) {
    const double peak=std::max(-modal.initial_mode_increment[8],-modal.initial_mode_increment[12]);
    Require(std::isfinite(peak)&&peak>0,"Penalty screen requires coherent negative initial tip increments");
    GuidedPlatePenaltySample out; out.requested_penetration=depth;
    out.modal_scale=-(model.data().initial_gap+depth)/peak;
    ElasticCouponConfiguration configuration;
    for (;;) {
        if (ApplyGuidedPlateIncrement(model.shell().data().reference_configuration,modal.initial_mode_increment,
                out.modal_scale,configuration,error)!=Status::kSuccess) throw std::runtime_error(error);
        out.maximum_depth=Depth(model,configuration);
        if (out.maximum_depth.upper<=depth) break;
        Require(out.inward_scale_adjustments<64,"Penalty screen cannot represent an inside-cap modal sample");
        out.modal_scale=std::nextafter(out.modal_scale,0.); ++out.inward_scale_adjustments;
    }
    out.actual_penetration=0;
    for (const auto& p:configuration.position) out.actual_penetration=std::max(out.actual_penetration,p.x-model.wall().wall_x());
    Require(out.maximum_depth.lower>0&&depth-out.maximum_depth.lower<1e-12,
        "Penalty screen represented depth is not at its requested boundary");
    const auto point=Evaluate(model,modal,out.modal_scale,scratch,error);
    out.shell_energy_tl=point.tl.energy; out.shell_energy_chrono=point.chrono.energy;
    out.shell_energy_allowance=EnergyAllowance(modal.initial_elastic_energy,point.tl.energy,point.chrono.energy);
    out.contact_potential=point.contact.potential; out.contact_resultant=point.contact.resultant;
    out.strip_potential=point.contact.strip_potential; out.strip_resultant=point.contact.strip_resultant;
    out.strip_energy_allowance=point.contact.strip_energy_allowance; out.strip_force_allowance=point.contact.strip_force_allowance;
    double lower=0;
    Require(bounds::AddScalar(std::min(point.tl.energy,point.chrono.energy),-out.shell_energy_allowance,false,&lower)&&
        bounds::AddScalar(lower,out.contact_potential.lower,false,&out.total_potential_lower),
        "Penalty screen lower energy arithmetic failed");
    for (unsigned level=0;level<2;++level)
        out.derivative[level]=Derivative(model,modal,out.modal_scale,peak,patch_audit::TranslationDifference/(1u<<level),
                                        cap,point,scratch,error);
    return out;
}
} // namespace

ElasticCouponStatus AuditGuidedPlatePenalty(const GuidedPlateModel& model,const GuidedPlateModalReport& modal,
                                          GuidedPlatePenaltyReport& output,std::string& error) {
    try {
        const auto& data=model.data(); const auto* spec=FindGuidedPlateExperiment(data.experiment);
        Require(spec&&modal.experiment==data.experiment&&modal.qualification_id==data.qualification_id&&
            modal.contact_rate_bound==model.contact_stiffness().rate_bound&&
            std::isfinite(modal.initial_elastic_energy)&&modal.initial_elastic_energy>0,
            "Penalty screen modal/model identity or initial energy is invalid");
        const double mean_tip=.5*modal.initial_mode_increment[8]+.5*modal.initial_mode_increment[12];
        Require(std::isfinite(mean_tip)&&std::abs(mean_tip-data.initial_tip_displacement)<=
            32*std::numeric_limits<double>::epsilon()*std::abs(data.initial_tip_displacement),
            "Penalty screen initial amplitude differs from its named experiment");
        ElasticCouponConfiguration initial;
        if (ApplyGuidedPlateIncrement(model.shell().data().reference_configuration,modal.initial_mode_increment,1,
                initial,error)!=Status::kSuccess) throw std::runtime_error(error);
        ElasticCouponEvaluation tl,chrono;
        EvaluateShell(model,initial,modal.initial_elastic_energy,tl,chrono,error);
        Require(std::abs(tl.energy-modal.initial_elastic_energy)<=EnergyAllowance(tl.energy,tl.energy,modal.initial_elastic_energy),
            "Penalty screen initial mode energy differs from its modal audit");
        GuidedPlatePenaltyReport next; next.experiment=data.experiment; next.qualification_id=data.qualification_id;
        next.initial_energy=modal.initial_elastic_energy; next.enforced=data.experiment==GuidedPlateExperiment::PenaltyMarginV1;
        Require(bounds::MultiplyScalar(1+EnergyFraction,next.initial_energy,true,&next.admitted_energy_upper),
            "Penalty screen admitted energy upper bound overflow");
        detail::ContactScratch scratch;
        next.sample[0]=Sample(model,modal,data.target_penetration,false,scratch,error);
        next.sample[1]=Sample(model,modal,data.maximum_penetration,true,scratch,error);
        next.target_sufficient=next.sample[0].total_potential_lower>next.admitted_energy_upper;
        Require(!next.enforced||next.target_sufficient,"Penalty-margin-v1 prescribed target energy is insufficient");
        output=next; error.clear(); return Status::kSuccess;
    } catch (const std::exception& exception) { error=exception.what(); return Status::kAuditRejected; }
}
} // namespace crash::reference
