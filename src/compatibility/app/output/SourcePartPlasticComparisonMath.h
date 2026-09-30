#pragma once
#include "SourcePartWallComparisonInput.h"

namespace crash::output::plastic_comparison {
namespace wc = wall_comparison;
inline constexpr std::size_t Channels=14;
using Differences=std::array<double,Channels>;
inline constexpr const char* Names[]{"position_m","rotation_rad","synchronized_velocity_m_per_s",
    "synchronized_omega_rad_per_s","point_plastic_strain","parent_plastic_work_J","total_plastic_work_J",
    "synchronized_kinetic_J","native_internal_work_J","contact_potential_J","total_native_contact_energy_J",
    "carried_wall_impulse_N_s","wall_reaction_N","energy_residual_J"};

inline double Scalar(const Value& value) {
    Require(value.IsNumber()&&std::isfinite(value.GetDouble()),"Nonfinite plastic comparison scalar");
    return value.GetDouble();
}
struct NumericalResponse {
    double hourglass_viscous_work=0,viscous_fraction_of_initial_kinetic=0;
    double carried_rotation_total=0,carried_rotation_physical=0,carried_rotation_added=0;
    double carried_velocity_time=0;
};
// Per-run diagnostics. Carried rotational energies refer to the explicitly
// reported midpoint time; they are not a common-endpoint comparison channel.
inline NumericalResponse ReadNumericalResponse(const Value& final,double initial_kinetic) {
    Require(std::isfinite(initial_kinetic)&&initial_kinetic>0,"Invalid numerical-work reference energy");
    const auto& qeph=wc::Field(final,"qeph");
    const auto& kinetic=wc::Field(final,"carried_kinetic");
    NumericalResponse r;
    r.hourglass_viscous_work=wc::Real(qeph,"hourglass_viscous_work_J");
    r.carried_velocity_time=wc::Real(qeph,"velocity_time_s");
    r.carried_rotation_total=wc::Real(kinetic,"rotation_total_J");
    r.carried_rotation_physical=wc::Real(kinetic,"rotation_physical_isotropic_J");
    r.carried_rotation_added=wc::Real(kinetic,"rotation_added_isotropic_J");
    Require(r.hourglass_viscous_work>=0&&r.carried_rotation_total>=0&&
        r.carried_rotation_physical>=0&&r.carried_rotation_added>=0&&r.carried_velocity_time>=0,
        "Negative numerical-work or carried-rotation diagnostic");
    const long double sum=static_cast<long double>(r.carried_rotation_physical)+r.carried_rotation_added;
    const auto roundoff=512*std::numeric_limits<double>::epsilon()*(sum+r.carried_rotation_total);
    Require(std::abs(sum-r.carried_rotation_total)<=roundoff,"Carried native inertia energy partitions disagree");
    r.viscous_fraction_of_initial_kinetic=r.hourglass_viscous_work/initial_kinetic;
    Require(std::isfinite(r.viscous_fraction_of_initial_kinetic),"Nonfinite numerical-work energy ratio");
    return r;
}
inline std::array<double,2> PlasticDifference(const Value& a,const Value& b) {
    const auto& x=wc::Field(a,"plastic_sections");const auto& y=wc::Field(b,"plastic_sections");
    Require(x.IsArray()&&y.IsArray()&&x.Size()==94&&y.Size()==94,"Incomplete plastic comparison sections");
    std::array<double,2> result{};
    for(unsigned e=0;e<94;++e) {
        const auto& p=x[e];const auto& q=y[e];
        Require(p.IsArray()&&q.IsArray()&&p.Size()==13&&q.Size()==13,"Invalid plastic comparison row");
        for(unsigned i=0;i<3;++i)Require(p[i].IsUint64()&&q[i].IsUint64()&&p[i].GetUint64()==q[i].GetUint64(),
            "Plastic comparison parent/family identities differ");
        Require(p[0].GetUint64()==e,"Plastic comparison parent ordering changed");
        Require(p[12].IsArray()&&q[12].IsArray()&&p[12].Size()==3&&q[12].Size()==3,
            "Plastic comparison needs three material points per parent");
        for(unsigned layer=0;layer<3;++layer) {
            const auto& px=p[12][layer];const auto& qx=q[12][layer];
            Require(px.IsArray()&&qx.IsArray()&&px.Size()==7&&qx.Size()==7,"Incomplete plastic comparison point");
            const double sa=Scalar(px[5]),sb=Scalar(qx[5]);
            Require(sa>=0&&sb>=0&&sa<=.3&&sb<=.3,"Plastic comparison strain exceeds original curve");
            result[0]=std::max(result[0],std::abs(sa-sb));
        }
        const double wa=Scalar(p[3]),wb=Scalar(q[3]);
        Require(wa>=0&&wb>=0,"Negative accumulated plastic work");
        result[1]=std::max(result[1],std::abs(wa-wb));
    }
    return result;
}
inline Differences Difference(const Value& a,const Value& b) {
    // Kinematics checks exact shared physical time. Its four output scales are
    // converted back to dimensional differences; raw staggered v/omega are not compared.
    const auto k=source_comparison::Kinematics(a,b);
    const auto plastic=PlasticDifference(a,b);
    const auto ca=wc::Contact(a),cb=wc::Contact(b);
    const long double ea=static_cast<long double>(ca.synchronized_kinetic)+ca.native_internal_work+ca.potential;
    const long double eb=static_cast<long double>(cb.synchronized_kinetic)+cb.native_internal_work+cb.potential;
    Differences result{k[0]*.001,k[1]*.01,k[2],k[3]*100,plastic[0],plastic[1],
        std::abs(wc::Real(a,"cumulative_plastic_work_J")-wc::Real(b,"cumulative_plastic_work_J")),
        std::abs(ca.synchronized_kinetic-cb.synchronized_kinetic),
        std::abs(ca.native_internal_work-cb.native_internal_work),std::abs(ca.potential-cb.potential),
        static_cast<double>(std::abs(ea-eb)),std::abs(ca.wall_impulse-cb.wall_impulse),
        std::abs(ca.wall_reaction-cb.wall_reaction),
        std::abs(wc::Real(a,"energy_residual_J")-wc::Real(b,"energy_residual_J"))};
    for(double value:result)Require(std::isfinite(value)&&value>=0,"Invalid plastic comparison difference");
    return result;
}
} // namespace crash::output::plastic_comparison
