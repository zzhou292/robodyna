#include "Types.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
bool SameProfile(Profile a,Profile b) noexcept {
    return a.type45==b.type45 && a.structural_limit==b.structural_limit;
}
Document ProfileDocument(Profile p) {
    Document d;d.SetObject();
    String(d,"schema",ProfileSchema);
    String(d,"purpose","selected_physical_model_accepted_visualization_not_restart");
    String(d,"participants",p.type45?"qeph,t3,qbat,type25,type13,solids,type45":"qeph,t3,qbat,type25,type13,solids");
    String(d,"structural_limit",p.structural_limit?"post_cin_local_physical_bound_s":"unavailable");
    for(const auto* name:{"kinetic_energy","total_energy","internal_work","contact_force","contact_penetration","contact_work","joint_work"})
        String(d,name,"unavailable");
    return d;
}
Profile ReadProfile(const Value& v) {
    using namespace array_json;
    Keys(v,{"schema","purpose","participants","structural_limit","kinetic_energy","total_energy","internal_work",
        "contact_force","contact_penetration","contact_work","joint_work"});
    Require(Text(v["schema"])==ProfileSchema && Text(v["purpose"])=="selected_physical_model_accepted_visualization_not_restart",
        "Unsupported physical observation profile");
    const auto roles=Text(v["participants"]);
    Profile p;
    p.type45=roles=="qeph,t3,qbat,type25,type13,solids,type45";
    Require(p.type45 || roles=="qeph,t3,qbat,type25,type13,solids","Unknown physical participant set");
    const auto structural=Text(v["structural_limit"]);
    p.structural_limit=structural=="post_cin_local_physical_bound_s";
    Require(p.structural_limit || structural=="unavailable","Unknown structural-limit availability");
    for(const auto* name:{"kinetic_energy","total_energy","internal_work","contact_force","contact_penetration","contact_work","joint_work"})
        Require(Text(v[name])=="unavailable","Unqualified physical diagnostic availability");
    return p;
}
} // namespace crash::output::physical_run
