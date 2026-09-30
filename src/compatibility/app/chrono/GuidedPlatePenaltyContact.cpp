#include "GuidedPlatePenaltyContact.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::reference::penalty_detail {
namespace bounds=sc::q4_bounds;
namespace {
using Interval=sc::Q4IntegralInterval;
bool Expand(const sc::PreparedQ4PlanarParent& p,sc::Q4CertifiedIntegral& value) {
    Interval ratio,truth;
    return bounds::DividePositive(p.area_enclosure,p.projected_area,&ratio) &&
        bounds::MultiplyPositive({value.lower,value.upper},ratio,&truth) && bounds::Certify(value.value,truth,&value);
}
// Independent analytic integral of the width-averaged affine gap. This does
// not replace C2's actual bilinear input. Positive-part Lipschitz bounds cover
// width asymmetry, C3 area uncertainty and conservative binary64 conversion.
bool Strip(const sc::PreparedQ4PlanarParent& parent,const ElasticCouponConfiguration& x,
           double wall_x,double kappa,const sc::Q4IntegrationResult& actual,ContactSample& out) {
    long double g[4];
    for (unsigned i=0;i<4;++i) g[i]=static_cast<long double>(x.position[parent.parent.nodes[i]].x)-wall_x;
    const long double a=(g[1]+g[2])/2,b=(g[0]+g[3])/2,lo=std::min(a,b),hi=std::max(a,b);
    long double first=0,second=0;
    if (hi>0) {
        if (lo>=0) { first=(lo+hi)/2; second=(lo*lo+lo*hi+hi*hi)/3; }
        else { first=hi*hi/(2*(hi-lo)); second=hi*hi*hi/(3*(hi-lo)); }
    }
    const long double area=parent.projected_area,k=kappa;
    const long double force=k*area*first,energy=k*area*second/2;
    const long double asym=std::max({std::abs(g[0]-b),std::abs(g[3]-b),std::abs(g[1]-a),std::abs(g[2]-a)});
    const long double max_gap=std::max({0.L,g[0],g[1],g[2],g[3]});
    const long double area_error=std::max(area-parent.area_enclosure.lower,parent.area_enclosure.upper-area);
    const long double eps=std::numeric_limits<double>::epsilon();
    const long double force_error=k*(area*asym+area_error*max_gap)+actual.resultant.error+128*eps*k*area*(max_gap+asym);
    const long double energy_error=k*(area*asym*(max_gap+asym/2)+area_error*max_gap*max_gap/2)+
        actual.potential.error+128*eps*k*area*(max_gap+asym)*(max_gap+asym);
    if (!std::isfinite(force) || !std::isfinite(energy) ||
        std::abs(force-actual.resultant.value)>force_error || std::abs(energy-actual.potential.value)>energy_error) return false;
    out.strip_resultant+=static_cast<double>(force); out.strip_potential+=static_cast<double>(energy);
    double f=0,e=0;
    return bounds::Round(static_cast<double>(force_error),true,&f) &&
        bounds::Round(static_cast<double>(energy_error),true,&e) &&
        bounds::AddScalar(out.strip_force_allowance,f,true,&out.strip_force_allowance) &&
        bounds::AddScalar(out.strip_energy_allowance,e,true,&out.strip_energy_allowance);
}
} // namespace

ElasticCouponStatus Contact(const GuidedPlateModel& model,const ElasticCouponConfiguration& configuration,
                           ContactScratch& scratch,ContactSample& output,std::string& error) {
    auto reject=[&](const char* message) { error=message; return ElasticCouponStatus::kAuditRejected; };
    const auto& guided=model.data(); const auto& shell=model.shell().data();
    std::array<double,3*kCouponNodes> x{},v{};
    for (unsigned n=0;n<kCouponNodes;++n) {
        x[3*n]=configuration.position[n].x; x[3*n+1]=configuration.position[n].y; x[3*n+2]=configuration.position[n].z;
    }
    const sc::Q4SurfaceView surface{{x.data(),kCouponNodes,3,1},{v.data(),kCouponNodes,3,1},guided.parents.data(),kCouponElements};
    const sc::Q4FixedYZMassView mass{shell.inverse_mass.data(),guided.translation_fixed_bits.data(),kCouponNodes,0};
    const auto reference=model.contact_geometry().view();
    ContactSample next; Interval potential,resultant;
    for (unsigned p=0;p<kCouponElements;++p) {
        sc::Q4PreparedIntegration prepared;
        if (sc::PrepareQ4PlanarIntegration(reference,surface,mass,p,guided.stiffness_per_area,
                guided.maximum_penetration,1,&prepared)!=sc::PlanarContactStatus::Ok || !prepared.covered)
            return reject("Penalty screen C3 preparation failed");
        sc::Q4RectangularResult result;
        const auto report=sc::IntegrateQ4NormalContactRectangular(prepared.input,guided.integration,scratch.view(),&result);
        if (report.status!=sc::Q4IntegrationStatus::Ok) {
            error="Penalty screen C2 rejected parent="+std::to_string(p)+", status="+std::to_string(static_cast<int>(report.status));
            return ElasticCouponStatus::kAuditRejected;
        }
        auto& integral=result.integration;
        if (!Expand(reference.parents[p],integral.potential) || !Expand(reference.parents[p],integral.resultant) ||
            integral.potential.error>guided.integration.energy_error || integral.resultant.error>guided.integration.force_error)
            return reject("Penalty screen exact-area potential/resultant exceeds original certificate budget");
        for (unsigned n=0;n<4;++n) {
            auto& force=integral.force[n]; const auto global=integral.nodal.nodes[n];
            if (global>=kCouponNodes || !Expand(reference.parents[p],force) || force.error>guided.integration.force_error ||
                !bounds::Add(next.nodal_magnitude[global],{force.lower,force.upper},&next.nodal_magnitude[global]))
                return reject("Penalty screen exact-area nodal force certificate failed");
            next.force_x[global]+=integral.nodal.forces[n].x;
            if (!std::isfinite(next.force_x[global])) return reject("Penalty screen nodal force reduction overflow");
        }
        if (!bounds::Add(potential,{integral.potential.lower,integral.potential.upper},&potential) ||
            !bounds::Add(resultant,{integral.resultant.lower,integral.resultant.upper},&resultant))
            return reject("Penalty screen certificate reduction failed");
        next.potential.value+=integral.potential.value; next.resultant.value+=integral.resultant.value;
        if (!Strip(reference.parents[p],configuration,reference.wall_x,guided.stiffness_per_area,integral,next))
            return reject("Penalty screen independent affine-strip agreement failed");
    }
    if (!bounds::Certify(next.potential.value,potential,&next.potential) ||
        !bounds::Certify(next.resultant.value,resultant,&next.resultant))
        return reject("Penalty screen aggregate certificate failed");
    output=next; error.clear(); return ElasticCouponStatus::kSuccess;
}
} // namespace crash::reference::penalty_detail
