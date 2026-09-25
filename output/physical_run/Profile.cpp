#include "Types.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
namespace {
constexpr const char* NativeHistoryPayload="native_type25_accepted_base_force_history_v1";
constexpr const char* LegacyFixedHistoryPayload="fixed_main_type25_accepted_base_force_history_v1";
}
bool SameProfile(Profile a,Profile b) noexcept {
    return a.type45==b.type45 && a.structural_limit==b.structural_limit && a.beam18==b.beam18 && a.self_contact==b.self_contact && a.native_contact==b.native_contact;
}
Document ProfileDocument(Profile p) {
    Require(!(p.native_contact&&(p.self_contact||p.type45||p.beam18)),"Native scene profile cannot claim legacy vehicle participants");
    Document d;d.SetObject();
    String(d,"schema",p.native_contact?NativeContactProfileSchema:p.self_contact?SelfContactProfileSchema:ProfileSchema);
    String(d,"purpose","selected_physical_model_accepted_visualization_not_restart");
    std::string participants=p.native_contact?"qeph,t3,native_type25":"qeph,t3,qbat,type25,type13,solids";
    if(p.type45)participants+=",type45";
    if(p.beam18)participants+=",beam18";
    if(p.self_contact)participants+=",self_contact";
    String(d,"participants",participants);
    String(d,"structural_limit",p.structural_limit?"post_cin_local_physical_bound_s":"unavailable");
    for(const auto* name:{"kinetic_energy","total_energy","internal_work","contact_force","contact_penetration","contact_work","joint_work"})
        String(d,name,"unavailable");
    if(p.self_contact)String(d,"self_contact","frictionless_fixed_triangles_v1_accepted_base_force_candidate_policy");
    if(p.native_contact)String(d,"native_contact",NativeHistoryPayload);
    return d;
}
Profile ReadProfile(const Value& v) {
    using namespace array_json;
    if(v.IsObject()&&v.HasMember("native_contact")) {
        Keys(v,{"schema","purpose","participants","structural_limit","kinetic_energy","total_energy","internal_work",
            "contact_force","contact_penetration","contact_work","joint_work","native_contact"});
        Require(Text(v["schema"])==NativeContactProfileSchema&&Text(v["purpose"])=="selected_physical_model_accepted_visualization_not_restart"&&
            Text(v["participants"])=="qeph,t3,native_type25"&&(Text(v["native_contact"])==NativeHistoryPayload||Text(v["native_contact"])==LegacyFixedHistoryPayload),
            "Unknown native physical observation profile");
        Profile p;p.native_contact=true;const auto structural=Text(v["structural_limit"]);
        p.structural_limit=structural=="post_cin_local_physical_bound_s";
        Require(p.structural_limit||structural=="unavailable","Unknown structural observation");
        for(const auto* name:{"kinetic_energy","total_energy","internal_work","contact_force","contact_penetration","contact_work","joint_work"})
            Require(Text(v[name])=="unavailable","Unqualified native physical diagnostic availability");
        return p;
    }
    const bool self=v.IsObject() && v.HasMember("self_contact");
    if(self)Keys(v,{"schema","purpose","participants","structural_limit","kinetic_energy","total_energy","internal_work",
        "contact_force","contact_penetration","contact_work","joint_work","self_contact"});
    else Keys(v,{"schema","purpose","participants","structural_limit","kinetic_energy","total_energy","internal_work",
        "contact_force","contact_penetration","contact_work","joint_work"});
    Require(Text(v["schema"])==(self?SelfContactProfileSchema:ProfileSchema) && Text(v["purpose"])=="selected_physical_model_accepted_visualization_not_restart",
        "Unsupported physical observation profile");
    auto roles=Text(v["participants"]);
    Profile p;
    p.self_contact=self;
    if(self) {
        Require(Text(v["self_contact"])=="frictionless_fixed_triangles_v1_accepted_base_force_candidate_policy" &&
            roles.size()>13 && roles.compare(roles.size()-13,13,",self_contact")==0,
            "Unknown self-contact observation profile or participant order");
        roles.resize(roles.size()-13);
    }
    p.type45=roles=="qeph,t3,qbat,type25,type13,solids,type45" ||
             roles=="qeph,t3,qbat,type25,type13,solids,type45,beam18";
    p.beam18=roles=="qeph,t3,qbat,type25,type13,solids,beam18" ||
             roles=="qeph,t3,qbat,type25,type13,solids,type45,beam18";
    Require(p.type45 || p.beam18 || roles=="qeph,t3,qbat,type25,type13,solids",
            "Unknown physical participant set");
    const auto structural=Text(v["structural_limit"]);
    p.structural_limit=structural=="post_cin_local_physical_bound_s";
    Require(p.structural_limit || structural=="unavailable","Unknown structural-limit availability");
    for(const auto* name:{"kinetic_energy","total_energy","internal_work","contact_force","contact_penetration","contact_work","joint_work"})
        Require(Text(v[name])=="unavailable","Unqualified physical diagnostic availability");
    return p;
}
} // namespace crash::output::physical_run
