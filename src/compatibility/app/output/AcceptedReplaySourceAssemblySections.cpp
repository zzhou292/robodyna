#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
std::vector<ReplayParentScalar> CheckAssemblySections(const Bundle& b,const Entry& e,const Value& v) {
    const auto& a=*b.assembly;const auto& s=a.source.data();
    Require(Text(v,"field_contract")=="robo-dyna.source-assembly-section-fields.v1"&&
        Text(v,"source_inventory_schema")==s.schema&&Text(v,"source_inventory_sha256")==s.identity.sha256&&
        Unsigned(v,"source_inventory_bytes")==s.identity.bytes&&Unsigned(v,"owner_id")==e.owner&&
        Unsigned(v,"run_id")==b.info.run_id&&Unsigned(v,"topology_id")==b.info.topology_id&&
        Text(v,"coordinate_frame")=="canonical assembled world metres; physical scale 1","Assembly section source scope changed");
    Require(Text(v,"parent_columns")=="source_parent_index,source_element_id,source_part_id,source_material_id,source_section_id,source_curve_id,source_elform,family,family_index,first_triangle,triangle_count"&&
        Text(v,"section_columns")=="cumulative_plastic_work_J,plastic_work_density_increment_J_m3,maximum_plastic_strain,mean_plastic_strain,minimum_tangent_ratio,mean_tangent_ratio,mean_yield_before_Pa,last_point_yield_before_Pa,reported_thickness_m,points"&&
        Text(v,"point_columns")=="stress_XX_Pa,stress_YY_Pa,stress_XY_Pa,stress_YZ_Pa,stress_ZX_Pa,equivalent_plastic_strain,filtered_rate_per_s",
        "Assembly native section field columns changed");
    CheckAssemblySurface(b,v);
    const auto& position=WallNumbers(v,"section_position_over_thickness",3);const auto& weight=WallNumbers(v,"section_force_weight",3);
    const auto& moment=WallNumbers(v,"section_moment_weight",3);
    const double z[]={-.5,0,.5},w[]={.25,.5,.25},m[]={-static_cast<double>(.0833333f),0,static_cast<double>(.0833333f)};
    for(unsigned j=0;j<3;++j){AssemblyEqual(position[j].GetDouble(),z[j]);AssemblyEqual(weight[j].GetDouble(),w[j]);AssemblyEqual(moment[j].GetDouble(),m[j]);}
    const auto& parents=WallArray(v,"source_parents",s.parents.size());const auto& sections=WallArray(v,"sections",s.parents.size());
    std::vector<ReplayParentScalar> display;display.reserve(s.parents.size());std::size_t triangle=0;
    for(std::size_t i=0;i<s.parents.size();++i) {
        const auto& p=s.parents[i];const auto& ids=AssemblyRow(parents[i],11);
        const std::uint64_t expected[]={i,p.source_id,p.part_id,p.material_id,p.section_id,p.curve_id,p.source_elform};
        for(unsigned j=0;j<7;++j)Require(AssemblyId(ids[j])==expected[j],"Assembly section source parent/material mapping changed");
        Require(ids[7].IsString()&&std::string(ids[7].GetString())==(p.arity==4?"QEPH":"T3")&&
            AssemblyId(ids[8])==p.family_index&&AssemblyId(ids[9])==triangle&&AssemblyId(ids[10])==(p.arity==4?2u:1u),
            "Assembly section family/triangle mapping changed");triangle+=p.arity==4?2:1;
        const auto& row=AssemblyRow(sections[i],10);
        for(unsigned j=0;j<9;++j)Require(AssemblyNumber(row[j])>=0,"Invalid native section diagnostic");
        const double initial=s.sections[p.section_index].thickness_m[0],thickness=row[8].GetDouble();
        Require(thickness>0&&thickness/initial>=a.minimum_thickness_ratio&&thickness/initial<=a.maximum_thickness_ratio,
            "Assembly native thickness exceeded its declared envelope");
        const auto& points=AssemblyRow(row[9],3);double maximum=0;long double mean=0;
        for(unsigned j=0;j<3;++j) {
            const auto& point=AssemblyNumbers(points[j],7);const double pla=point[5].GetDouble(),rate=point[6].GetDouble();
            Require(pla>=0&&pla<=s.curves[p.curve_index].plastic_strain.back()&&rate>=0,"Assembly point history exceeds its source curve");
            maximum=std::max(maximum,pla);mean+=static_cast<long double>(w[j])*pla;
            if(!e.epoch)for(const auto& value:point.GetArray())Require(value.GetDouble()==0,"Initial native point history is not pristine");
        }
        AssemblyEqual(row[2].GetDouble(),maximum);AssemblyNear(row[3].GetDouble(),mean,3);
        Require(row[4].GetDouble()<=row[5].GetDouble()&&row[5].GetDouble()<=1,"Invalid native section tangent reduction");
        if(!e.epoch) {
            Require(row[0].GetDouble()==0&&row[1].GetDouble()==0,"Initial plastic work is not zero");AssemblyEqual(thickness,initial);
        }
        display.push_back({p.source_id,maximum});
    }
    return display;
}
std::vector<ReplayParentScalar> ReadSourceAssemblyDisplay(const Bundle& b,const Entry& e) {
    const auto frame=Json(VerifiedBytes(b,e.mesh.substr(0,e.mesh.size()-10)+".fields.json"));
    return CheckAssemblySections(b,e,Member(frame,"sections"));
}
} // namespace crash::output::replay_detail
