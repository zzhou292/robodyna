#include "AcceptedReplaySourcePartWall.h"
#include "SourcePartPlasticSchema.h"
#include <algorithm>
#include <limits>

namespace crash::output::replay_detail {
namespace {
void Near(double a,double b,const char* message) {
    const double budget=256*std::numeric_limits<double>::epsilon()*std::max({std::abs(a),std::abs(b),1e-30});
    Require(std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=budget,message);
}
double Scalar(const Value& v) {Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Invalid plastic scalar");return v.GetDouble();}
std::uint64_t Identifier(const Value& v) {Require(v.IsUint64(),"Invalid plastic identifier");return v.GetUint64();}
}
void ReadSourcePartPlasticConfiguration(Bundle& b,const Document& c) {
    b.plastic_initial_thickness=Real(c,"thickness_m");
    b.plastic_min_thickness_ratio=Real(c,"minimum_thickness_ratio");b.plastic_max_thickness_ratio=Real(c,"maximum_thickness_ratio");
    Require(b.plastic_min_thickness_ratio>0&&b.plastic_max_thickness_ratio>=b.plastic_min_thickness_ratio,"Invalid plastic thickness envelope");
    b.info.material_model=Text(c,"material_model");b.info.material_policy=Text(c,"material_policy");
    const bool rates=b.info.material_model=="source_cowper_symonds_law44";
    Require(rates||b.info.material_model=="experimental_rate_independent_tabulated_j2","Unsupported plastic material model");
    Require(!b.info.material_policy.empty()&&!Text(c,"stabilization_policy").empty()&&!Text(c,"plastic_work_policy").empty()&&
        !Text(c,"plastic_history_timing").empty(),"Missing explicit plastic material/history policies");
    Require(Unsigned(c,"source_material_id")==2000157&&Unsigned(c,"source_section_id")==2000157&&Unsigned(c,"source_curve_id")==2100270&&
        Unsigned(c,"source_elform")==2&&Unsigned(c,"source_nip")==3&&Unsigned(c,"source_vp")==0&&Real(c,"source_supplied_sigy_Pa")==270e6&&
        Real(c,"source_rate_coefficient_per_s")==8000&&Real(c,"source_rate_exponent")==8,"Source plastic material tuple changed");
    WallBool(c,"source_rate_effects_active",rates);WallBool(c,"rate_filter_enabled",rates);
    const double rate_c=Real(c,"rate_cowper_symonds_c_per_s"),rate_p=Real(c,"rate_cowper_symonds_p"),cutoff=Real(c,"rate_filter_cutoff_hz");
    Require(rates?(rate_c==8000&&rate_p==8&&cutoff==10000):(rate_c==0&&rate_p==0&&cutoff==0),"Plastic rate settings differ from the declared source branch");
    const double angular_cutoff=rates?2.*std::atan2(0.,-1.)*cutoff:0;
    Require(Bits(Real(c,"rate_filter_angular_cutoff_per_s"))==Bits(angular_cutoff)&&
        Bits(Real(c,"rate_filter_alpha"))==Bits(std::fmin(1.,angular_cutoff*b.fixed_dt)),"Source filter coefficient differs from its actual fixed interval");
    Require(Text(c,"rate_filter_policy")=="Native VP2 total equivalent strain rate; alpha=min(1,2*pi*cutoff_hz*dt); accepted point filter history starts at zero",
        "Unsupported source rate/filter policy");
    Require(Text(c,"stabilization_policy")=="Native QEPH CZFINTN1 NPT3 plastic stabilization using actual section ETSE and last-point SIGY; existing generalized work; T3 has no QEPH stabilization",
        "Plastic stabilization policy changed");
    Require(Text(c,"plastic_thickness_policy")=="ITHICK1: accepted reported thickness at interval base drives forces and section strains; original native mass remains fixed",
        "Unsupported plastic thickness policy");
    const auto& strain=WallNumbers(c,"source_plastic_strain",46);const auto& stress=WallNumbers(c,"source_yield_stress_Pa",46);
    double x[46]{},y[46]{};
    for(unsigned i=0;i<46;++i) {
        x[i]=strain[i].GetDouble();y[i]=stress[i].GetDouble();
        Require(x[i]>=0&&y[i]>0&&(!i||(x[i]>x[i-1]&&y[i]>=y[i-1])),"Plastic curve is not monotone");
        b.plastic_curve.push_back({x[i],y[i]});
    }
    Require(PlasticCurveSha256(x,y,46)==SourcePartPlasticCurveSha256&&Text(c,"source_curve_semantic_sha256")==SourcePartPlasticCurveSha256,
        "Plastic curve differs from the original46-point source");
    b.plastic_curve_maximum=x[45];
    const auto& z=WallNumbers(c,"section_position_over_thickness",3);const auto& wf=WallNumbers(c,"section_force_weight",3);
    const auto& wm=WallNumbers(c,"section_moment_weight",3);
    for(unsigned i=0;i<3;++i)Require(z[i].GetDouble()==.5*(double(i)-1)&&wf[i].GetDouble()==(i==1?.5:.25)&&
        wm[i].GetDouble()==(double(i)-1)*static_cast<double>(.0833333f),"Plastic section table changed");
    Require(Text(c,"plastic_section_columns")=="source_parent_index,source_element_id,family_index,cumulative_plastic_work_J,plastic_work_density_increment_J_m3,maximum_plastic_strain,mean_plastic_strain,minimum_tangent_ratio,mean_tangent_ratio,mean_yield_before_Pa,last_point_yield_before_Pa,reported_thickness_m,points"&&
        Text(c,"plastic_point_columns")=="stress_XX_Pa,stress_YY_Pa,stress_XY_Pa,stress_YZ_Pa,stress_ZX_Pa,equivalent_plastic_strain,filtered_rate_per_s",
        "Unsupported plastic point/section record layout");
    const auto& parents=WallArray(c,"source_parents",94);const auto& volume=WallNumbers(c,"plastic_reference_volume_m3",94);
    unsigned qi=0,ti=0;double total_volume=0;
    for(unsigned p=0;p<94;++p) {
        const auto arity=Unsigned(parents[p],"arity"),index=Unsigned(parents[p],"family_index");
        Require(index==(arity==4?qi++:ti++)&&volume[p].GetDouble()>0,"Plastic native family/volume mapping changed");
        b.plastic_source_parents.push_back({Unsigned(parents[p],"source_element_id"),arity,index});
        b.plastic_reference_volume.push_back(volume[p].GetDouble());total_volume+=volume[p].GetDouble();
    }
    Require(qi==88&&ti==6,"Plastic section family union incomplete");
    double total_mass=0;for(const auto& node:WallArray(c,"reference_nodes",117).GetArray())total_mass+=Real(node,"mass_kg");
    Near(total_volume*Real(c,"density_kg_m3"),total_mass,"Plastic reference volume differs from native total mass");
}
void CheckSourcePartPlasticFields(const Bundle& b,const Entry& e,const Document& f) {
    Require(Text(f,"material_model")==b.info.material_model&&Unsigned(f,"plastic_owner_id")==e.owner&&Unsigned(f,"plastic_accepted_epoch")==e.epoch&&
        Unsigned(f,"plastic_attempt")== (e.epoch?e.interval_attempt:0)&&Bits(Real(f,"plastic_accepted_time_s"))==Bits(e.time),
        "Plastic history belongs to another material/accepted owner epoch");
    const auto& rows=WallArray(f,"plastic_sections",94);
    double maximum=0,weighted=0,volume=0,total_work=0;unsigned yielded_points=0,yielded_parents=0;
    for(unsigned p=0;p<94;++p) {
        const auto& row=rows[p];Require(row.IsArray()&&row.Size()==13,"Plastic section row shape mismatch");
        const auto& id=b.plastic_source_parents[p];
        Require(Identifier(row[0])==p&&Identifier(row[1])==id[0]&&Identifier(row[2])==id[2],"Plastic source parent/family identity changed");
        const double work=Scalar(row[3]),increment=Scalar(row[4]),record_max=Scalar(row[5]),record_mean=Scalar(row[6]),tangent=Scalar(row[7]);
        Require(work>=0&&increment>=0&&record_max>=0&&record_mean>=0&&tangent>=0&&tangent<=1,"Invalid plastic section diagnostic");
        const double mean_tangent=Scalar(row[8]),mean_yield=Scalar(row[9]),last_yield=Scalar(row[10]);
        Require(mean_tangent>=tangent&&mean_tangent<=1&&mean_yield>=0&&last_yield>=0,"Invalid native plastic stabilization input");
        const double thickness=Scalar(row[11]),ratio=thickness/b.plastic_initial_thickness;
        Require(thickness>=1e-30&&std::isfinite(ratio)&&ratio>=b.plastic_min_thickness_ratio&&ratio<=b.plastic_max_thickness_ratio,
            "Accepted plastic thickness exceeds its declared envelope");
        if(!e.epoch)Require(Bits(thickness)==Bits(b.plastic_initial_thickness),"Initial plastic thickness differs from original source");
        const auto& points=row[12];Require(points.IsArray()&&points.Size()==3,"Plastic section lacks three material points");
        double local_max=0,local_mean=0;bool parent_yielded=false;
        for(unsigned k=0;k<3;++k) {
            const auto& point=points[k];Require(point.IsArray()&&point.Size()==7,"Plastic material point shape mismatch");
            for(const auto& scalar:point.GetArray())Scalar(scalar);
            const double plastic=point[5].GetDouble(),rate=point[6].GetDouble();
            Require(plastic>=0&&plastic<=b.plastic_curve_maximum&&rate>=0,"Plastic history exceeds its source domain");
            if(b.info.material_model=="experimental_rate_independent_tabulated_j2")Require(rate==0,"Rate-off material acquired filter history");
            if(!e.epoch)for(const auto& scalar:point.GetArray())Require(scalar.GetDouble()==0,"Initial plastic history was fabricated");
            local_max=std::max(local_max,plastic);local_mean+=(k==1?.5:.25)*plastic;
            yielded_points+=plastic>0;parent_yielded|=plastic>0;
        }
        Near(local_max,record_max,"Plastic maximum does not match actual material points");
        Near(local_mean,record_mean,"Plastic mean does not match actual material points");
        if(!e.epoch)Require(work==0&&increment==0&&record_max==0&&record_mean==0&&tangent==1&&mean_tangent==1&&mean_yield==0&&last_yield==0,"Initial section diagnostic is nonzero");
        maximum=std::max(maximum,local_max);weighted+=b.plastic_reference_volume[p]*local_mean;
        volume+=b.plastic_reference_volume[p];total_work+=work;yielded_parents+=parent_yielded;
    }
    Near(Real(f,"maximum_plastic_strain"),maximum,"Plastic frame maximum summary mismatch");
    Near(Real(f,"mean_plastic_strain"),weighted/volume,"Plastic frame mean summary mismatch");
    Near(Real(f,"cumulative_plastic_work_J"),total_work,"Plastic frame work summary mismatch");
    Require(Unsigned(f,"yielded_points")==yielded_points&&Unsigned(f,"yielded_parents")==yielded_parents,"Plastic yield counts mismatch");
    if(e.epoch==b.info.final_epoch) {
        const char* names[]{"maximum_plastic_strain","mean_plastic_strain","cumulative_plastic_work_J"};
        for(unsigned i=0;i<3;++i)Require(Bits(Real(f,names[i]))==Bits(b.plastic_final_values[i]),"Final plastic metrics differ from accepted history");
        Require(yielded_points==b.plastic_final_counts[0]&&yielded_parents==b.plastic_final_counts[1],"Final plastic counts differ from accepted history");
    }
}
std::vector<ReplayParentScalar> ReadSourcePartPlasticDisplay(const Bundle& b,const Entry& e) {
    // Re-read only the requested frame, under the same byte/hash cap. This
    // avoids retaining all histories during Open or coupling scientific field
    // validation to a display consumer. Call after CheckFrameFields succeeds.
    const auto fields=Json(VerifiedBytes(b,e.mesh.substr(0,e.mesh.size()-10)+".fields.json"));
    const auto& rows=WallArray(fields,"plastic_sections",94);
    std::vector<ReplayParentScalar> result;result.reserve(rows.Size());
    for(const auto& row:rows.GetArray()) {
        const auto& points=row[12];
        double maximum=0;
        for(const auto& point:points.GetArray())maximum=std::max(maximum,point[5].GetDouble());
        result.push_back({row[1].GetUint64(),maximum});
    }
    return result;
}
}
