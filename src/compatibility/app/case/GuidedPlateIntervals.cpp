#include "GuidedPlateIntervals.h"
#include "output/ArtifactInventory.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>

namespace crash::case_data {
using output::Require;
namespace {
constexpr std::size_t kColumnCap=64,kNumericBytes=25,kHeaderCap=8192;
struct Field { const char* name; double value; };
class Row {
  public:
    explicit Row(bool header):header_(header) {}
    void Add(const char* name,double value) {
        Require(std::isfinite(value),"Nonfinite guided interval field");
        if(header_) { AddText(name); return; }
        char bytes[32]; const auto r=std::to_chars(bytes,bytes+sizeof(bytes),value,std::chars_format::scientific,16);
        Require(r.ec==std::errc{}&&r.ptr-bytes<=kNumericBytes,"Guided FP64 encoding exceeds forecast");
        AddText(std::string(bytes,r.ptr));
    }
    void Add(const char* name,std::uint64_t value) {
        if(header_) { AddText(name); return; }
        char bytes[21]; const auto r=std::to_chars(bytes,bytes+sizeof(bytes),value);
        Require(r.ec==std::errc{}&&r.ptr-bytes<=20,"Guided integer encoding exceeds forecast");
        AddText(std::string(bytes,r.ptr));
    }
    std::string Finish() {
        Require(count_<=kColumnCap,"Guided interval column cap exceeded"); text_+='\n';
        Require(text_.size()<=(header_?kHeaderCap:count_*(kNumericBytes+1)),"Guided interval row exceeds forecast");
        return std::move(text_);
    }
  private:
    void AddText(const std::string& text) { if(count_++)text_+=','; text_+=text; }
    bool header_; std::size_t count_=0; std::string text_;
};
std::string Make(const tl::fea::NodalStamp& base,const GuidedPlateMetrics& m,bool header,std::initializer_list<Field> fields) {
    Row row(header); row.Add("owner_id",base.owner_id); row.Add("base_epoch",base.epoch); row.Add("attempt",m.shell.attempt);
    row.Add("force_eval_time_s",base.time); row.Add("accepted_epoch",m.stamp.epoch); row.Add("accepted_time_s",m.stamp.time);
    for(const auto& f:fields)row.Add(f.name,f.value); return row.Finish();
}
std::array<std::string,3> Build(const tl::fea::NodalStamp& base,const GuidedPlateMetrics& m,bool header) {
    const auto& s=m.shell; const auto& c=m.contact; const auto& b=m.applied_contact; const auto& w=m.work;
    auto combined=Make(base,m,header,{
        {"kinetic_energy_J",w.kinetic_energy},{"total_energy_J",w.total_energy},{"initial_energy_J",m.initial_energy},
        {"relative_energy_error",w.relative_energy_error},{"maximum_relative_energy_error",m.maximum_relative_energy_error},
        {"combined_kinetic_residual_J",w.combined_kinetic_residual},{"kinetic_arithmetic_budget_J",w.kinetic_arithmetic_budget},
        {"shell_coordinate_work_budget_J",w.shell_coordinate_work_budget},{"contact_defect_lower_limit_J",w.contact_defect_lower_limit},
        {"contact_defect_upper_limit_J",w.contact_defect_upper_limit},
        {"wall_impulse_x_Ns",m.wall_impulse.x},{"wall_impulse_y_Ns",m.wall_impulse.y},{"wall_impulse_z_Ns",m.wall_impulse.z},
        {"wall_moment_impulse_x_Nms",m.wall_moment_impulse.x},{"wall_moment_impulse_y_Nms",m.wall_moment_impulse.y},{"wall_moment_impulse_z_Nms",m.wall_moment_impulse.z},
        {"accumulated_shell_midpoint_work_J",m.shell_midpoint_work},{"accumulated_contact_midpoint_work_J",m.contact_midpoint_work},
        {"accumulated_shell_coordinate_work_J",m.shell_coordinate_work},{"accumulated_contact_coordinate_work_J",m.contact_coordinate_work},
        {"peak_penetration_m",m.peak_penetration},{"last_operator_norm_per_s2",m.last_operator_norm},
        {"last_operator_epoch",static_cast<double>(m.last_operator_epoch)},{"full_state_audit_reads",static_cast<double>(m.full_state_audit_reads)}});
    auto shell=Make(base,m,header,{
        {"elastic_energy_J",s.elastic_energy},{"bending_energy_J",s.bending_energy},{"kinetic_translation_J",s.kinetic_translation},
        {"kinetic_physical_rotation_J",s.kinetic_physical_rotation},{"kinetic_artificial_drilling_J",s.kinetic_artificial_drilling},
        {"base_elastic_energy_J",s.base_elastic_energy},{"base_kinetic_energy_J",s.base_kinetic_energy},
        {"elastic_increment_J",s.elastic_energy_increment},{"kinetic_increment_J",s.kinetic_energy_increment},
        {"kinetic_midpoint_work_J",s.kinetic_midpoint_work},{"shell_only_kinetic_work_residual_J",s.kinetic_work_residual},
        {"force_coordinate_work_J",s.force_coordinate_work},{"force_potential_defect_J",s.conservative_force_coordinate_defect},
        {"mass_weighted_increment_squared",s.mass_weighted_increment_squared},
        {"maximum_displacement_m",s.maximum_displacement},{"maximum_director_departure_rad",s.maximum_director_departure},
        {"maximum_pair_angle_rad",s.maximum_pair_angle},{"maximum_membrane_shear_strain",s.maximum_membrane_strain},
        {"maximum_thickness_curvature",s.maximum_thickness_curvature},{"minimum_signed_area_ratio",s.minimum_signed_area_ratio},
        {"minimum_area_norm_ratio",s.minimum_area_norm_ratio},{"maximum_area_norm_ratio",s.maximum_area_norm_ratio},
        {"minimum_display_triangle_area_ratio",s.minimum_display_triangle_area_ratio}});
    auto contact=Make(base,m,header,{
        {"wall_reaction_x_N_at_base",b.wall_reaction.x},{"wall_reaction_y_N_at_base",b.wall_reaction.y},{"wall_reaction_z_N_at_base",b.wall_reaction.z},
        {"wall_moment_x_Nm_at_base",b.wall_moment.x},{"wall_moment_y_Nm_at_base",b.wall_moment.y},{"wall_moment_z_Nm_at_base",b.wall_moment.z},
        {"force_error_x_N_at_base",b.force_error.x},{"force_error_y_N_at_base",b.force_error.y},{"force_error_z_N_at_base",b.force_error.z},
        {"wall_moment_error_x_Nm_at_base",b.wall_moment_error.x},{"wall_moment_error_y_Nm_at_base",b.wall_moment_error.y},{"wall_moment_error_z_Nm_at_base",b.wall_moment_error.z},
        {"wall_reaction_x_N_at_endpoint",c.wall_reaction.x},{"wall_reaction_y_N_at_endpoint",c.wall_reaction.y},{"wall_reaction_z_N_at_endpoint",c.wall_reaction.z},
        {"wall_moment_x_Nm_at_endpoint",c.wall_moment.x},{"wall_moment_y_Nm_at_endpoint",c.wall_moment.y},{"wall_moment_z_Nm_at_endpoint",c.wall_moment.z},
        {"force_error_x_N_at_endpoint",c.force_error.x},{"force_error_y_N_at_endpoint",c.force_error.y},{"force_error_z_N_at_endpoint",c.force_error.z},
        {"potential_J",c.potential.value},{"potential_lower_J",c.potential.lower},{"potential_upper_J",c.potential.upper},{"potential_error_J",c.potential.error},
        {"base_potential_J",c.base_potential},{"base_potential_error_J",c.base_potential_error},{"potential_increment_J",c.potential_increment},
        {"kinetic_midpoint_work_J",c.kinetic_midpoint_work},{"kinetic_midpoint_roundoff_J",c.kinetic_midpoint_roundoff},
        {"force_coordinate_work_J",c.force_coordinate_work},{"force_coordinate_roundoff_J",c.force_coordinate_roundoff},
        {"force_potential_defect_J",c.conservative_force_coordinate_defect},{"continuum_work_uncertainty_J",c.continuum_work_uncertainty},
        {"quadratic_work_upper_J",c.quadratic_work_upper},{"active_area_lower_m2",c.active_area.lower},{"active_area_upper_m2",c.active_area.upper},
        {"maximum_penetration_m",c.maximum_penetration},{"surface_power_W",c.surface_power},{"surface_power_error_W",c.surface_power_error},
        {"stiffness_rate_bound_per_s2",c.stiffness_rate_bound},{"leaves",static_cast<double>(c.leaves)},
        {"visited",static_cast<double>(c.visited)},{"deepest_leaf",static_cast<double>(c.deepest_leaf)}});
    return {std::move(combined),std::move(shell),std::move(contact)};
}
} // namespace
const std::array<std::string,3>& GuidedPlateIntervalHeaders() {
    static const auto headers=Build({},GuidedPlateMetrics{},true); return headers;
}
std::array<std::string,3> GuidedPlateIntervalRows(const tl::fea::NodalStamp& base,const GuidedPlateMetrics& m) {
    Require(tl::fea::IsCollocatedNodalTiming(base.temporal_scheme,base.velocity_phase)&&
            tl::fea::IsCollocatedNodalTiming(m.stamp.temporal_scheme,m.stamp.velocity_phase),
            "Guided interval schema requires collocated nodal velocity");
    // Integer-valued diagnostic counters remain exactly representable in this
    // bounded experiment; identity columns are always encoded as uint64.
    Require(m.last_operator_epoch<=1000000&&m.full_state_audit_reads<=1000000,"Guided audit counter output exceeds scope");
    return Build(base,m,false);
}
GuidedPlateOutputForecast ForecastGuidedPlateOutput(std::uint64_t steps,unsigned every) {
    Require(steps&&steps<=1000000&&every,"Invalid guided output schedule");
    GuidedPlateOutputForecast result;
    result.frames=1+steps/every+(steps%every!=0);
    Require(result.frames<=output::kArtifactFrameCap,"Guided output frame cap exceeded");
    result.total_bytes=kGuidedStaticReserve+result.frames*(kGuidedFieldFileCap+kGuidedMeshFileCap+kGuidedObjFileCap);
    const auto& headers=GuidedPlateIntervalHeaders();
    std::size_t segment_count=0;
    for(unsigned n=0;n<3;++n) {
        const auto columns=1+std::count(headers[n].begin(),headers[n].end(),',');
        Require(columns<=kColumnCap&&headers[n].size()<=kHeaderCap,"Invalid guided ledger layout");
        const std::size_t row=columns*(kNumericBytes+1);
        result.ledgers[n]=output::PlanCsvLedger(kGuidedIntervalFiles[n],headers[n],steps,row);
        result.ledger_bytes[n]=result.ledgers[n].total_bytes;
        segment_count+=result.ledgers[n].segments.size();
        result.segmented=result.segmented||result.ledgers[n].segments.size()>1;
        Require(result.total_bytes<=output::kArtifactTotalCap&&
                result.ledger_bytes[n]<=output::kArtifactTotalCap-result.total_bytes,"Guided archive forecast exceeds byte caps");
        result.total_bytes+=result.ledger_bytes[n];
    }
    // Configuration, three canonical wall files, frame index, final metrics.
    Require(3*result.frames+segment_count+6<=output::kArtifactInventoryCap,"Guided archive inventory forecast exceeds capacity");
    return result;
}
} // namespace crash::case_data
