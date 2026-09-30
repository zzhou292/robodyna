#include "SourcePartElasticNativeDiagnostics.h"
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace crash::cases::source_part_elastic::test {
namespace {
namespace oracle=tn::force_test;
template<class Range> void Values(std::ostream& out,const char* name,const Range& values) {
    out<<name<<'='; for(const auto value:values) out<<' '<<value; out<<'\n';
}
void History(std::ostream& out,const char* name,const tn::HistoryValues& values) {
    std::array<double,26> packed{};tn::detail::PackHistory(values,packed);Values(out,name,packed);
}
template<class Range> void Vectors(std::ostream& out,const char* name,const Range& values) {
    out<<name<<'=';for(const auto v:values)out<<" ["<<v.x<<','<<v.y<<','<<v.z<<']';out<<'\n';
}
}
void PrintT3Failure(std::uint64_t parent,const tn::Reference& reference,const tn::HistoryValues& base,
    const tn::HistoryValues* previous_cuda,const tn::HistoryStamp* previous_stamp,
    const tn::PrescribedInterval& input,const tn::ForceTrial& native_trial,const t::ForceTrial& cuda_trial) {
    const auto& r=reference.data().input;
    const auto cuda_values=t3_force_port_test::Native(cuda_trial.proposed_history.data());
    const auto analytic=oracle::Independent(r,base,input);
    const auto rates=oracle::kt::Independent(input);
    const bool current_cuda_base=previous_cuda&&previous_stamp&&previous_stamp->time==input.base_time&&
        previous_stamp->sample_index+1==input.sample_index;
    std::ostringstream out;out<<std::scientific<<std::setprecision(std::numeric_limits<long double>::max_digits10);
    out<<"T3_FAILURE_DIAGNOSTIC_BEGIN source_parent="<<parent<<" sample="<<input.sample_index
        <<" base_time="<<input.base_time<<" dt="<<input.dt<<" current_cuda_base="<<current_cuda_base<<'\n';
    out<<"reference E="<<r.young_modulus<<" nu="<<r.poisson_ratio<<" rho="<<r.density<<" t="<<r.thickness<<'\n';
    Values(out,"source_node_ids",r.node_ids);Vectors(out,"reference_x",r.position);
    Vectors(out,"endpoint_x",input.position);Vectors(out,"midpoint_v",input.velocity);Vectors(out,"midpoint_omega",input.angular_velocity);
    out<<"history_columns=stress[5],material_stress[5],bending_stress[3],strain_curvature[8],thickness,internal_work[2],equivalent_strain_rate,active\n";
    History(out,"native_base",base);
    if(previous_cuda) {
        History(out,"previous_cuda_base",*previous_cuda);
        out<<"previous_cuda_stamp="<<previous_stamp->time<<' '<<previous_stamp->sample_index<<'\n';
    }
    History(out,"native_proposed",native_trial.proposed_history.data());History(out,"cuda_proposed",cuda_values);
    Values(out,"analytic_from_native_base",analytic.values);
    if(current_cuda_base) Values(out,"analytic_from_cuda_base",oracle::Independent(r,*previous_cuda,input).values);
    Values(out,"native_raw_rates",native_trial.kinematics.raw_rate);Values(out,"cuda_raw_rates",cuda_trial.kinematics.raw_rate);
    Values(out,"analytic_raw_rates",rates.raw);Values(out,"native_normalized_rates",native_trial.kinematics.normalized_rate);
    Values(out,"cuda_normalized_rates",cuda_trial.kinematics.normalized_rate);Values(out,"analytic_normalized_rates",rates.normalized);
    out<<"areas native="<<native_trial.kinematics.area<<" cuda="<<cuda_trial.kinematics.area<<" analytic="<<rates.area<<'\n';
    std::array<double,8> cuda_dx{},native_dx{};
    std::array<long double,8> analytic_dx{};
    for(unsigned k=0;k<8;++k) {
        cuda_dx[k]=cuda_trial.kinematics.raw_rate[k]*(input.dt/cuda_trial.kinematics.area);
        native_dx[k]=native_trial.kinematics.raw_rate[k]*(input.dt/native_trial.kinematics.area);
        analytic_dx[k]=rates.normalized[k]*input.dt;
    }
    Values(out,"cuda_raw_times_dt_over_area",cuda_dx);Values(out,"native_raw_times_dt_over_area",native_dx);
    Values(out,"analytic_strain_increment",analytic_dx);
    const long double E=r.young_modulus,nu=r.poisson_ratio,a11=E/(1-nu*nu),a12=nu*a11;
    out<<"stress0_error native_minus_analytic="<<static_cast<long double>(native_trial.proposed_history.data().stress[0])-analytic.values[0]
        <<" cuda_minus_analytic="<<static_cast<long double>(cuda_values.stress[0])-analytic.values[0]<<'\n';
    out<<"stress0_terms analytic_a11_d0="<<a11*analytic_dx[0]<<" analytic_a12_d1="<<a12*analytic_dx[1]
        <<" analytic_viscous="<<analytic.values[0]-analytic.values[5]
        <<" cuda_viscous="<<static_cast<long double>(cuda_values.stress[0])-cuda_values.material_stress[0]
        <<" native_viscous="<<static_cast<long double>(native_trial.proposed_history.data().stress[0])-native_trial.proposed_history.data().material_stress[0]<<'\n';
    if(current_cuda_base) {
        const auto own_base=oracle::Independent(r,*previous_cuda,input);
        out<<"stress0_decomposition base_material_shift="<<static_cast<long double>(previous_cuda->material_stress[0])-base.material_stress[0]
            <<" cuda_a11_increment_difference="<<a11*(cuda_dx[0]-analytic_dx[0])
            <<" cuda_a12_increment_difference="<<a12*(cuda_dx[1]-analytic_dx[1])
            <<" cuda_minus_own_base_analytic="<<static_cast<long double>(cuda_values.stress[0])-own_base.values[0]<<'\n';
    }
    // Diagnostic sensitivity only: neither evaluated source input nor pinned
    // analytic oracle is changed. The continuum strain-rate map is invariant
    // to common translation; removing it exposes absolute-velocity cancellation.
    auto relative=input;const auto origin=input.velocity[0];
    for(auto& v:relative.velocity) {v.x-=origin.x;v.y-=origin.y;v.z-=origin.z;}
    Values(out,"diagnostic_relative_velocity_analytic_rates",oracle::kt::Independent(relative).normalized);
    Values(out,"diagnostic_relative_velocity_analytic_history",oracle::Independent(r,base,relative).values);
    out<<"T3_FAILURE_DIAGNOSTIC_END\n";
    const auto text=out.str();
    std::cerr<<text.substr(0,32*1024); // Fixed single-element record, hard bounded.
}
} // namespace crash::cases::source_part_elastic::test
