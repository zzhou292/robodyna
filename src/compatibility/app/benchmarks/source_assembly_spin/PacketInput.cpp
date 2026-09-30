#include "AnalysisInternal.h"
#include <map>
namespace crash::benchmarks::assembly_spin {
void CheckSharedMotion(const Context& c,const Value& row) {
    const char* key[]{"position_endpoint_xyz_m","velocity_previous_midpoint_xyz_m_s","omega_previous_midpoint_xyz_rad_s"};
    const char* selected_key[]{"position_endpoint_m","velocity_previous_midpoint_m_s","omega_previous_midpoint_rad_s"};
    for(bool next:{false,true}) {
        std::map<std::size_t,std::array<double,9>> seen;
        const auto& packet=json::Array(row,next?"enclosing_candidate_parents":"parents",c.parents.size(),c.parents.size());
        for(std::size_t p=0;p<c.parents.size();++p)for(unsigned local=0;local<4;++local) {
            const auto index=c.parents[p].parent->nodes[local];std::array<double,9> value{};
            for(unsigned kind=0;kind<3;++kind) {
                const auto& array=Numbers(packet[static_cast<unsigned>(p)],key[kind],12);
                for(unsigned axis=0;axis<3;++axis)value[3*kind+axis]=json::Real(array[3*local+axis]);
                if(!next&&index==c.global_node) {
                    const auto& selected=Numbers(row,selected_key[kind],3);
                    for(unsigned axis=0;axis<3;++axis)json::Same(value[3*kind+axis],json::Real(selected[axis]));
                }
            }
            const auto [it,inserted]=seen.emplace(index,value);
            if(!inserted)for(unsigned i=0;i<9;++i)json::Same(value[i],it->second[i]);
        }
    }
}
void CheckParent(const Context& c,const ParentSource& selected,const Value& row,std::uint64_t epoch,double time,double origin) {
    const auto& p=*selected.parent;
    Require(json::Unsigned(row,"source_element_id")==p.source_id&&json::Unsigned(row,"source_part_id")==p.part_id&&
        json::Unsigned(row,"source_material_id")==p.material_id&&json::Unsigned(row,"source_section_id")==p.section_id&&
        json::Unsigned(row,"source_curve_id")==p.curve_id&&json::Unsigned(row,"source_parent_index")==p.index&&
        json::Unsigned(row,"family_index")==p.family_index&&json::Unsigned(row,"local_node")==selected.local,"Spin source parent association differs");
    const auto& ids=json::Array(row,"source_nodes",4,4);
    for(unsigned n=0;n<4;++n)Require(json::Unsigned(ids[n])==c.inventory.data().nodes[p.nodes[n]].source_id,"Spin native node order differs");
    Numbers(row,"position_endpoint_xyz_m",12);Numbers(row,"velocity_previous_midpoint_xyz_m_s",12);Numbers(row,"omega_previous_midpoint_xyz_rad_s",12);
    Numbers(row,"positive_internal_force_xyz_N",12);Numbers(row,"positive_internal_couple_xyz_N_m",12);
    const auto& h=json::Member(row,"retained_history");Require(json::Unsigned(h,"epoch")==epoch,"Spin native history epoch differs");json::Same(json::Real(h,"time_s"),time);
    const auto& k=json::Member(row,"retained_kinematics");json::Flag(k,"available",epoch!=0);
    if(epoch){Require(json::Unsigned(k,"origin_endpoint_epoch")==epoch,"Spin native kinematic epoch differs");
        json::Same(json::Real(k,"origin_base_time_s"),origin);json::Same(json::Real(k,"origin_dt_s"),c.dt);}
    json::TextIs(row,"point_columns","XX_Pa,YY_Pa,XY_Pa,YZ_Pa,ZX_Pa,PLA,filtered_rate_per_s");
    const auto& points=json::Array(row,"points",3,3);for(const auto& point:points.GetArray()) {
        Require(point.IsArray()&&point.Size()==7,"Spin native point shape differs");for(const auto& x:point.GetArray())json::Real(x);
        Require(json::Real(point[5u])>=0&&json::Real(point[6u])>=0,"Spin native point history is negative");
    }
    Require(json::Real(row,"cumulative_plastic_work_J")>=0,"Spin native cumulative plastic work is negative");
}
layered::QephHistory History(const native::Reference& ref,const Value& row) {
    const auto& h=json::Member(row,"retained_history");native::HistoryValues values;
    Array(h,"FOR_Pa",values.stress);Array(h,"FOR_G_Pa",values.material_stress);Array(h,"MOM_Pa",values.bending_stress);
    Array(h,"HOURG_native_mixed_units",values.stabilization);Array(h,"STRA_native",values.strain_curvature);
    Array(h,"internal_work_J",values.internal_work);values.thickness=json::Real(h,"reported_thickness_m");
    values.hourglass_viscous_work=json::Real(h,"hourglass_viscous_work_J");values.active=json::Real(h,"active");
    layered::QephHistory result;
    Require(native::PreparePrescribedHistory(ref,values,{json::Real(h,"time_s"),json::Unsigned(h,"epoch")},result.shell)==native::Status::kSuccess,
            "Recorded native history cannot bind to the original source reference");
    const auto& points=json::Array(row,"points",3,3);
    for(unsigned i=0;i<3;++i){Require(points[i].IsArray()&&points[i].Size()==7,"Spin native point shape differs");
        for(unsigned j=0;j<7;++j)result.points[i][j]=json::Real(points[i][j]);}
    return result;
}
native::PrescribedInterval Interval(const Value& row,double dt) {
    native::PrescribedInterval interval;const auto& k=json::Member(row,"retained_kinematics");json::Flag(k,"available",true);
    interval.base_time=json::Real(k,"origin_base_time_s");interval.dt=dt;interval.sample_index=json::Unsigned(k,"origin_endpoint_epoch");
    auto nodes=[&](const char* key,auto& output) {const auto& a=Numbers(row,key,12);
        for(unsigned i=0;i<4;++i)output[i]={json::Real(a[3*i]),json::Real(a[3*i+1]),json::Real(a[3*i+2])};};
    nodes("position_endpoint_xyz_m",interval.position_endpoint);nodes("velocity_previous_midpoint_xyz_m_s",interval.velocity_midpoint);
    nodes("omega_previous_midpoint_xyz_rad_s",interval.omega_midpoint);return interval;
}
}
