#include "Internal.h"
#include <algorithm>
namespace crash::modelio::vehicle::rigid_part::detail {
void CheckOriginal(const VehicleSourcePlan& source) {
    const auto& d=source.canonical().data();
    Require(d.inputs.source_member.bytes==42846753 && d.inputs.source_member.sha256==
        "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301" &&
        d.inputs.canonical_manifest.sha256=="c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8" &&
        d.inputs.tire_policy=="omit_original_tire_shells" && source.counts().parents==349645 &&
        source.parts().size()==867,"Original rigid source/selection authority changed");
    const auto& u=d.inputs.units;
    Require(u.mass=="t" && u.length=="mm" && u.time=="s" && u.mass_to_kg==1000 &&
            u.length_to_m==.001 && u.time_to_s==1,"Original rigid units changed");
    constexpr std::uint64_t excluded[]{2000211,2000360,2000363,2000367,2000487,2000488,2000489,2000490};
    Require(d.excluded_parts.size()==std::size(excluded) &&
            std::equal(d.excluded_parts.begin(),d.excluded_parts.end(),std::begin(excluded)),
            "Original rigid tire exclusions changed");
}
SourceData Declarations(const VehicleSourcePlan& source,Limits limits) {
    SourceData out;
    out.part_to_body.assign(source.parts().size(),SIZE_MAX);
    const auto& u=source.canonical().data().inputs.units;
    for (std::size_t p=0;p<source.parts().size();++p) {
        const auto& part=source.parts()[p];
        if (part.unresolved_sources[2].keyword!="*MAT_RIGID") continue;
        Require(out.bodies.size()<limits.parts,"Rigid body count exceeds cap");
        Body body;
        body.source_part_index=p;
        body.source_part_id=part.part_id;
        body.shell_count=part.shell_count;
        body.declaration=ReadDeclaration(part,{u.mass_to_kg,u.length_to_m,u.time_to_s});
        out.part_to_body[p]=out.bodies.size();
        out.bodies.push_back(std::move(body));
        out.shell_count+=part.shell_count;
    }
    Require(out.bodies.size()==22 && out.shell_count==5102,"Original rigid shell coverage changed");
    return out;
}
}
