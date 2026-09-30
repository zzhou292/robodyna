#include "ShellPatchInertia.h"

#include <cmath>

namespace crash::reference {
namespace {
namespace tlr=tl::fea::reissner;
using Status=ElasticCouponStatus;

Status Reject(const char* message,std::string& diagnostic,Status status=Status::kInvalidConfiguration) {
    diagnostic=message;
    return status;
}
bool Positive(double value) { return std::isfinite(value)&&value>0; }
bool OriginalMass(const tlr::ShellNodalMass& mass) {
    return tlr::mass_detail::ValidMass(mass)&&
           mass.artificial_drilling_inertia==mass.physical_tangential_inertia;
}
bool Valid(const ShellPatchInertia& value) {
    if (!value.prepared) return false;
    for (const double area:value.element_area) if (!Positive(area)) return false;
    for (std::size_t n=0;n<kCouponNodes;++n) {
        const auto& original=value.original[n];
        const double added=value.added_tangential_inertia[n];
        const double total=value.counterfactual.total_isotropic_inertia[n];
        if (!OriginalMass(original)||!Positive(added)||value.added_drilling_inertia[n]!=added||
            value.counterfactual.mass[n]!=original.mass||!Positive(total)||
            total!=original.physical_tangential_inertia+added||
            total<=original.physical_tangential_inertia) return false;
    }
    return true;
}
}  // namespace

Status AssembleShellPatchInertia(
    const std::array<tlr::ShellMass,kCouponElements>& element_mass,
    const std::array<std::array<std::size_t,4>,kCouponElements>& connectivity,
    ShellPatchInertia& output,std::string& diagnostic) {
    ShellPatchInertia candidate;
    bool used[kCouponNodes]{};
    for (std::size_t e=0;e<kCouponElements;++e) {
        const auto& element=element_mass[e];
        if (element.drilling_policy!=tlr::ShellDrillingInertiaPolicy::kEqualPhysicalTangential)
            return Reject("Shell patch inertia requires explicit equal-physical-tangential drilling",diagnostic);
        bool local[kCouponNodes]{};
        double area=0;
        for (std::size_t i=0;i<4;++i) {
            const std::size_t n=connectivity[e][i];
            if (n>=kCouponNodes||local[n]||!OriginalMass(element.node[i]))
                return Reject("Shell patch inertia has invalid connectivity or original mass contribution",diagnostic);
            local[n]=true;
            area+=element.node[i].area;
            if (!Positive(area)) return Reject("Shell patch whole element area is unrepresentable",diagnostic,
                                               Status::kNonfiniteResult);
        }
        candidate.element_area[e]=area;
        for (std::size_t i=0;i<4;++i) {
            const std::size_t n=connectivity[e][i];
            const auto& source=element.node[i];
            auto& sum=candidate.original[n];
            sum.area+=source.area;
            sum.mass+=source.mass;
            sum.physical_tangential_inertia+=source.physical_tangential_inertia;
            sum.artificial_drilling_inertia+=source.artificial_drilling_inertia;
            // Wider temporary avoids rejecting a finite quotient merely because
            // m*A overflows binary64 before /12. No error enclosure is claimed.
            const double added=static_cast<double>(
                static_cast<long double>(source.mass)*static_cast<long double>(area)/12);
            if (!OriginalMass(sum)||!Positive(added))
                return Reject("Shell patch assembled mass or positive area inertia is unrepresentable",diagnostic,
                              Status::kNonfiniteResult);
            candidate.added_tangential_inertia[n]+=added;
            candidate.added_drilling_inertia[n]+=added;
            if (!Positive(candidate.added_tangential_inertia[n]))
                return Reject("Shell patch added area inertia overflowed",diagnostic,Status::kNonfiniteResult);
            used[n]=true;
        }
    }
    for (std::size_t n=0;n<kCouponNodes;++n) {
        if (!used[n]) return Reject("Shell patch inertia requires all six physical nodes",diagnostic);
        candidate.counterfactual.mass[n]=candidate.original[n].mass;
        candidate.counterfactual.total_isotropic_inertia[n]=
            candidate.original[n].physical_tangential_inertia+candidate.added_tangential_inertia[n];
    }
    candidate.prepared=true;
    if (!Valid(candidate))
        return Reject("Shell patch total isotropic inertia is unrepresentable",diagnostic,Status::kNonfiniteResult);
    output=candidate;
    diagnostic.clear();
    return Status::kSuccess;
}

Status ComputeShellPatchKineticEnergy(
    const ShellPatchInertia& inertia,
    const std::array<tlr::Vec3,kCouponNodes>& director,
    const std::array<tlr::Vec3,kCouponNodes>& velocity,
    const std::array<tlr::Vec3,kCouponNodes>& angular_velocity,
    ShellPatchKineticEnergy& output,std::string& diagnostic) {
    if (!Valid(inertia)) return Reject("Shell patch kinetic ledger requires consistent prepared inertia",diagnostic);
    ShellPatchKineticEnergy candidate;
    for (std::size_t n=0;n<kCouponNodes;++n) {
        if (!tlr::mass_detail::UnitDirector(director[n])||!tlr::detail::Finite(velocity[n])||
            !tlr::detail::Finite(angular_velocity[n]))
            return Reject("Shell patch kinetic ledger has invalid world kinematics",diagnostic);
        const auto tangent=tlr::detail::Cross(angular_velocity[n],director[n]);
        const double normal=tlr::detail::Dot(angular_velocity[n],director[n]);
        const double tangent2=tlr::detail::Dot(tangent,tangent),normal2=normal*normal;
        const auto& original=inertia.original[n];
        candidate.translation+=.5*original.mass*tlr::detail::Dot(velocity[n],velocity[n]);
        candidate.physical_rotation+=.5*original.physical_tangential_inertia*tangent2;
        candidate.original_artificial_drilling+=.5*original.artificial_drilling_inertia*normal2;
        candidate.added_tangential+=.5*inertia.added_tangential_inertia[n]*tangent2;
        candidate.added_drilling+=.5*inertia.added_drilling_inertia[n]*normal2;
    }
    const std::array<double,5> values{{candidate.translation,candidate.physical_rotation,
        candidate.original_artificial_drilling,candidate.added_tangential,candidate.added_drilling}};
    for (const double value:values)
        if (!std::isfinite(value)||value<0)
            return Reject("Shell patch kinetic energy overflowed",diagnostic,Status::kNonfiniteResult);
    output=candidate;
    diagnostic.clear();
    return Status::kSuccess;
}

}  // namespace crash::reference
