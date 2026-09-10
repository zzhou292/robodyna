#pragma once
#include "SourcePartPlasticComparisonInput.h"

namespace crash::output::plastic_comparison::test {
inline Document Copy(const Value& value) {
    Document result;result.CopyFrom(value,result.GetAllocator());return result;
}
inline Document Configuration(bool fine=false) {
    Document d;d.Parse(R"({"owner_id":1,"run_id":2,"topology_id":3,
      "fixed_dt_s":0.000000059604644775390625,"required_steps":131072,"frame_every":256,
      "requested_horizon_s":0.0078125,"interval_ledger_segments":[],
      "material_model":"source_cowper_symonds","rate_cowper_symonds_c_per_s":8000,
      "rate_cowper_symonds_p":8,"rate_filter_cutoff_hz":10000,"rate_filter_alpha":0.003,
      "reference_nodes":[{"mass_kg":0.2,"isotropic_inertia_kg_m2":0.0001}],
      "wall_setup":{"fixed_dt_s":0.000000059604644775390625,"declared_leading_gap_m":0.02},
      "future_physical_extension":{"must_match":true}})");
    if(fine) {
        d["owner_id"].SetUint64(4);d["run_id"].SetUint64(5);d["topology_id"].SetUint64(6);
        d["fixed_dt_s"].SetDouble(wc::BaseStep/2);d["required_steps"].SetUint64(262144);
        d["frame_every"].SetUint64(512);d["rate_filter_alpha"].SetDouble(.0015);
        d["wall_setup"]["fixed_dt_s"].SetDouble(wc::BaseStep/2);
    }
    return d;
}
inline Document Frame() {
    Document d;d.SetObject();auto& a=d.GetAllocator();Number(d,"accepted_time_s",0);
    double vectors[351]{},quaternions[468]{};
    for(unsigned n=0;n<117;++n)quaternions[4*n]=1;
    for(const char* field:{"position_xyz_m","synchronized_velocity_xyz_m_per_s",
        "synchronized_omega_world_xyz_rad_per_s","velocity_xyz_m_per_s"})FiniteArray(d,field,vectors,351);
    FiniteArray(d,"orientation_wxyz",quaternions,468);
    for(const char* field:{"synchronized_kinetic_J","total_internal_work_J","cumulative_wall_kick_impulse_N_s",
        "cumulative_plastic_work_J","energy_residual_J"})Number(d,field,0);
    d.AddMember("contact",Value(rapidjson::kNullType),a);
    Value sections(rapidjson::kArrayType);
    for(unsigned e=0;e<94;++e) {
        Value row(rapidjson::kArrayType),points(rapidjson::kArrayType);
        row.PushBack(e,a);row.PushBack(2213000+e,a);row.PushBack(e<88?e:e-88,a);
        for(unsigned i=3;i<12;++i)row.PushBack(0.,a);
        for(unsigned l=0;l<3;++l) {double point[7]{};points.PushBack(FiniteArray(d,point,7),a);}
        row.PushBack(points,a);sections.PushBack(row,a);
    }
    d.AddMember("plastic_sections",sections,a);return d;
}
} // namespace crash::output::plastic_comparison::test
