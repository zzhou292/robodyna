#include "SourceAssemblySectionFields.h"
#include "output/SurfaceBindingFields.h"
#include <cmath>

namespace crash::output::assembly {
namespace {
bool Valid(SectionView view,std::size_t count) noexcept {
    if(view.count!=count||!view.values||!view.reported_thickness_m)return false;
    for(std::size_t i=0;i<count;++i) {
        const auto& s=view.values[i];const auto& d=s.diagnostics;
        if(!std::isfinite(view.reported_thickness_m[i])||!(view.reported_thickness_m[i]>0))return false;
        const double diagnostic[]{s.cumulative_plastic_work_J,d.plastic_work_density_increment,
            d.maximum_plastic_strain,d.mean_plastic_strain,d.minimum_tangent_ratio,d.mean_tangent_ratio,
            d.mean_yield_before_pa,d.last_point_yield_before_pa};
        for(double x:diagnostic)if(!std::isfinite(x)||x<0)return false;
        for(const auto& point:s.history.point) {
            for(double x:point.stress)if(!std::isfinite(x))return false;
            if(!std::isfinite(point.plastic_strain)||point.plastic_strain<0||
                !std::isfinite(point.filtered_rate_per_s)||point.filtered_rate_per_s<0)return false;
        }
    }
    return true;
}
const SectionView& Family(const ParentMapping& p,const SectionView& q,const SectionView& t) noexcept {
    return p.family==source::ShellFamily::Qeph?q:t;
}
}
bool ValidSections(const SourceAssemblySurface& surface,SectionView q,SectionView t) noexcept {
    const auto& d=surface.source().data();
    // Preflight BOTH ranges before examining either borrowed field.
    if(q.count!=d.qeph_count||t.count!=d.t3_count||!q.values||!q.reported_thickness_m||
        !t.values||!t.reported_thickness_m)return false;
    return Valid(q,d.qeph_count)&&Valid(t,d.t3_count);
}
bool CopyParentScalars(const SourceAssemblySurface& surface,SectionView q,SectionView t,
                       std::vector<ReplayParentScalar>& out) noexcept {
    if(out.size()!=surface.parents().size()||!ValidSections(surface,q,t))return false;
    for(const auto& p:surface.parents())
        out[p.source_index]={p.element,Family(p,q,t).values[p.family_index].diagnostics.maximum_plastic_strain};
    return true;
}
Document SectionFieldDocument(const SourceAssemblySurface& surface,SectionView q,SectionView t) {
    Require(ValidSections(surface,q,t),"Invalid complete assembly section fields");
    Document doc;doc.SetObject();auto& a=doc.GetAllocator();
    String(doc,"field_contract","robo-dyna.source-assembly-section-fields.v1");
    String(doc,"source_inventory_schema",surface.source().data().schema);
    String(doc,"source_inventory_sha256",surface.source().data().identity.sha256);
    Integer(doc,"source_inventory_bytes",surface.source().data().identity.bytes);
    Integer(doc,"owner_id",surface.binding().identity.owner);
    Integer(doc,"run_id",surface.binding().identity.run);Integer(doc,"topology_id",surface.binding().identity.topology);
    AppendSurfaceBinding(doc,surface.binding());
    String(doc,"parent_columns","source_parent_index,source_element_id,source_part_id,source_material_id,source_section_id,source_curve_id,source_elform,family,family_index,first_triangle,triangle_count");
    String(doc,"section_columns","cumulative_plastic_work_J,plastic_work_density_increment_J_m3,maximum_plastic_strain,mean_plastic_strain,minimum_tangent_ratio,mean_tangent_ratio,mean_yield_before_Pa,last_point_yield_before_Pa,reported_thickness_m,points");
    String(doc,"point_columns","stress_XX_Pa,stress_YY_Pa,stress_XY_Pa,stress_YZ_Pa,stress_ZX_Pa,equivalent_plastic_strain,filtered_rate_per_s");
    String(doc,"coordinate_frame","canonical assembled world metres; physical scale 1");
    String(doc,"work_semantics","Native point diagnostics only; cumulative plastic work is not added again to shell work or a reconstructed connected energy");
    double positions[3],forces[3],moments[3];
    for(unsigned i=0;i<3;++i) {
        positions[i]=tl::fea::sections::LayerPosition(i);forces[i]=tl::fea::sections::LayerForceWeight(i);
        moments[i]=tl::fea::sections::LayerMomentWeight(i);
    }
    FiniteArray(doc,"section_position_over_thickness",positions,3);
    FiniteArray(doc,"section_force_weight",forces,3);FiniteArray(doc,"section_moment_weight",moments,3);
    Value parents(rapidjson::kArrayType),sections(rapidjson::kArrayType);
    for(const auto& p:surface.parents()) {
        Value map(rapidjson::kArrayType),row(rapidjson::kArrayType),points(rapidjson::kArrayType);
        for(std::uint64_t x:{std::uint64_t(p.source_index),p.element,p.part,p.material,p.section,p.curve,std::uint64_t(p.source_elform)})
            map.PushBack(Value().SetUint64(x),a);
        map.PushBack(Value(p.family==source::ShellFamily::Qeph?"QEPH":"T3",a),a);
        for(std::size_t x:{p.family_index,p.first_triangle,p.triangle_count})map.PushBack(Value().SetUint64(x),a);
        const auto& view=Family(p,q,t);const auto& s=view.values[p.family_index];const auto& d=s.diagnostics;
        for(double x:{s.cumulative_plastic_work_J,d.plastic_work_density_increment,d.maximum_plastic_strain,
            d.mean_plastic_strain,d.minimum_tangent_ratio,d.mean_tangent_ratio,d.mean_yield_before_pa,
            d.last_point_yield_before_pa,view.reported_thickness_m[p.family_index]})row.PushBack(x,a);
        for(const auto& point:s.history.point) {
            const double values[]{point.stress[0],point.stress[1],point.stress[2],point.stress[3],point.stress[4],
                point.plastic_strain,point.filtered_rate_per_s};
            points.PushBack(FiniteArray(doc,values,7),a);
        }
        row.PushBack(points,a);sections.PushBack(row,a);parents.PushBack(map,a);
    }
    doc.AddMember("source_parents",parents,a);doc.AddMember("sections",sections,a);
    return doc;
}
} // namespace crash::output::assembly
