#include "SourcePartPlasticFields.h"
#include "output/SourcePartPlasticSchema.h"
#include "case/source_part_plastic/SourcePartMaterial.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_part_plastic {
using namespace output;
namespace elastic=source_part_elastic;
bool PlasticEnabled(const elastic::Config& c) {return c.material_model!=elastic::MaterialModel::ElasticLaw1;}
const char* MaterialModelName(const elastic::Config& c) {
    if(c.material_model==elastic::MaterialModel::ExperimentalRateIndependentTabulatedJ2)return "experimental_rate_independent_tabulated_j2";
    if(c.material_model==elastic::MaterialModel::SourceCowperSymonds)return "source_cowper_symonds_law44";
    return "elastic_law1";
}
std::string MaterialPolicy(const elastic::Config& c) {
    if(c.material_model==elastic::MaterialModel::ExperimentalRateIndependentTabulatedJ2)return SourcePartMaterial::ExperimentalRateOffPolicy;
    if(c.material_model==elastic::MaterialModel::SourceCowperSymonds)
        return "Source tabulated LAW44 plane stress with C/P rate effects and resolved total-rate filter; evolving accepted force thickness; original native mass retained; no failure or curve extrapolation; source attachments unapplied";
    return "Experimental LAW1 E=200 GPa nu=0.3; original density and thickness; source MAT024/ELFORM2 not reproduced";
}
void AppendSourcePartPlasticConfiguration(Document& d,const elastic::SourcePartElasticCase& run) {
    const auto& c=run.config();Require(PlasticEnabled(c)&&c.material.prepared(),"Plastic archive needs its authenticated source material");
    const auto& m=c.material.declaration();
    String(d,"material_model",MaterialModelName(c));
    Integer(d,"source_material_id",m.material_id);Integer(d,"source_section_id",m.section_id);Integer(d,"source_curve_id",m.curve_id);
    Integer(d,"source_elform",m.source_elform);Integer(d,"source_nip",m.through_thickness_points);
    Number(d,"source_supplied_sigy_Pa",m.supplied_sigy_pa);Number(d,"source_rate_coefficient_per_s",m.source_rate_coefficient_per_s);
    Number(d,"source_rate_exponent",m.source_rate_exponent);Integer(d,"source_vp",m.source_vp);
    FiniteArray(d,"source_plastic_strain",c.material.plastic_strain().data(),MaterialCurvePoints);
    FiniteArray(d,"source_yield_stress_Pa",c.material.yield_stress_pa().data(),MaterialCurvePoints);
    const auto curve_hash=PlasticCurveSha256(c.material.plastic_strain().data(),c.material.yield_stress_pa().data(),MaterialCurvePoints);
    Require(curve_hash==SourcePartPlasticCurveSha256,"Plastic curve changed from the original source");String(d,"source_curve_semantic_sha256",curve_hash);
    Boolean(d,"source_rate_effects_active",c.material_model==elastic::MaterialModel::SourceCowperSymonds);
    Boolean(d,"rate_filter_enabled",c.rate.enabled);
    Number(d,"rate_cowper_symonds_c_per_s",c.rate.cowper_symonds_c_per_s);
    Number(d,"rate_cowper_symonds_p",c.rate.cowper_symonds_p);Number(d,"rate_filter_cutoff_hz",c.rate.cutoff_hz);
    const double angular_cutoff=c.rate.enabled?2.*std::atan2(0.,-1.)*c.rate.cutoff_hz:0;
    Number(d,"rate_filter_angular_cutoff_per_s",angular_cutoff);
    Number(d,"rate_filter_alpha",std::fmin(1.,angular_cutoff*c.dt));
    String(d,"rate_filter_policy","Native VP2 total equivalent strain rate; alpha=min(1,2*pi*cutoff_hz*dt); accepted point filter history starts at zero");
    String(d,"stabilization_policy","Native QEPH CZFINTN1 NPT3 plastic stabilization using actual section ETSE and last-point SIGY; existing generalized work; T3 has no QEPH stabilization");
    String(d,"plastic_thickness_policy","ITHICK1: accepted reported thickness at interval base drives forces and section strains; original native mass remains fixed");
    String(d,"plastic_work_policy","Native weighted point plastic-work diagnostic; already represented in total generalized shell work, never added a second time");
    const double positions[]{-.5,0,.5},force[]{.25,.5,.25};
    const double moment[]{-static_cast<double>(.0833333f),0,static_cast<double>(.0833333f)};
    FiniteArray(d,"section_position_over_thickness",positions,3);FiniteArray(d,"section_force_weight",force,3);
    FiniteArray(d,"section_moment_weight",moment,3);
    String(d,"plastic_section_columns","source_parent_index,source_element_id,family_index,cumulative_plastic_work_J,plastic_work_density_increment_J_m3,maximum_plastic_strain,mean_plastic_strain,minimum_tangent_ratio,mean_tangent_ratio,mean_yield_before_Pa,last_point_yield_before_Pa,reported_thickness_m,points");
    String(d,"plastic_point_columns","stress_XX_Pa,stress_YY_Pa,stress_XY_Pa,stress_YZ_Pa,stress_ZX_Pa,equivalent_plastic_strain,filtered_rate_per_s");
    String(d,"plastic_history_timing","Accepted endpoint section histories jointly published with both shell families and the sole nodal owner; epoch zero is initialized history");
    double volumes[source::ParentCount]{};unsigned q=0,t=0;
    for(unsigned p=0;p<source::ParentCount;++p) {
        const auto& parent=run.source().parents()[p];
        volumes[p]=parent.arity==4?run.binding().qeph_reference(q++).area*m.thickness_m:run.binding().t3_reference(t++).area*m.thickness_m;
    }
    FiniteArray(d,"plastic_reference_volume_m3",volumes,source::ParentCount);
}
void AppendSourcePartPlasticSummary(Document& d,const PlasticSummary& s) {
    Require(s.enabled&&std::isfinite(s.maximum_plastic_strain)&&s.maximum_plastic_strain>=0&&
        std::isfinite(s.mean_plastic_strain)&&s.mean_plastic_strain>=0&&std::isfinite(s.cumulative_plastic_work_J)&&
        s.cumulative_plastic_work_J>=0&&s.yielded_points<=3*source::ParentCount&&s.yielded_parents<=source::ParentCount,
        "Invalid accepted plastic summary");
    Number(d,"maximum_plastic_strain",s.maximum_plastic_strain);Number(d,"mean_plastic_strain",s.mean_plastic_strain);
    Number(d,"cumulative_plastic_work_J",s.cumulative_plastic_work_J);Integer(d,"yielded_points",s.yielded_points);
    Integer(d,"yielded_parents",s.yielded_parents);
}
void AppendSourcePartPlasticFrame(Document& d,elastic::SourcePartElasticCase& run,const elastic::Snapshot& frame) {
    Require(PlasticEnabled(run.config())&&frame.plastic.enabled,"Plastic frame lacks the selected material state");
    const auto before=run.owner().accepted();
    Require(before.owner_id==frame.stamp.owner_id&&before.epoch==frame.stamp.epoch&&Bits(before.time)==Bits(frame.stamp.time),
        "Plastic history readback does not identify the captured owner state");
    SourcePartPlasticState state;const auto result=run.CapturePlasticSectionHistory(&state);Require(bool(result),result.message);
    const auto after=run.owner().accepted();Require(after.owner_id==before.owner_id&&after.epoch==before.epoch&&Bits(after.time)==Bits(before.time),
        "Plastic owner changed during accepted history readback");
    AppendSourcePartPlasticSummary(d,frame.plastic);String(d,"material_model",MaterialModelName(run.config()));
    Integer(d,"plastic_accepted_epoch",frame.stamp.epoch);Integer(d,"plastic_owner_id",frame.stamp.owner_id);
    Integer(d,"plastic_attempt",frame.stamp.epoch?frame.diagnostics.shells.qeph.attempt:0);
    Number(d,"plastic_accepted_time_s",frame.stamp.time);
    Value rows(rapidjson::kArrayType);unsigned q=0,t=0;
    for(unsigned p=0;p<source::ParentCount;++p) {
        const auto& source_parent=run.source().parents()[p];const auto index=source_parent.arity==4?q++:t++;
        const auto& value=source_parent.arity==4?state.qeph[index]:state.t3[index];
        Value row(rapidjson::kArrayType),points(rapidjson::kArrayType);
        row.PushBack(p,d.GetAllocator());row.PushBack(Value().SetUint64(source_parent.source_id),d.GetAllocator());row.PushBack(index,d.GetAllocator());
        const double diagnostics[]{value.cumulative_plastic_work_J,value.diagnostics.plastic_work_density_increment,
            value.diagnostics.maximum_plastic_strain,value.diagnostics.mean_plastic_strain,value.diagnostics.minimum_tangent_ratio,
            value.diagnostics.mean_tangent_ratio,value.diagnostics.mean_yield_before_pa,value.diagnostics.last_point_yield_before_pa,
            source_parent.arity==4?state.qeph_reported_thickness[index]:state.t3_reported_thickness[index]};
        for(double x:diagnostics){Require(std::isfinite(x),"Nonfinite plastic section diagnostic");row.PushBack(x,d.GetAllocator());}
        for(const auto& point:value.history.point) {
            const double record[]{point.stress[0],point.stress[1],point.stress[2],point.stress[3],point.stress[4],point.plastic_strain,point.filtered_rate_per_s};
            points.PushBack(FiniteArray(d,record,7),d.GetAllocator());
        }
        row.PushBack(points,d.GetAllocator());rows.PushBack(row,d.GetAllocator());
    }
    d.AddMember("plastic_sections",rows,d.GetAllocator());
}
}
