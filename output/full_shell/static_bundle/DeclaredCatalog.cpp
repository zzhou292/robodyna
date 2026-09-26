#include "InputChecks.h"
#include <algorithm>
#include <map>
#include <set>
namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
void Member(const CanonicalData& data,const Value& value) {
    Keys(value,{"file","sha256","bytes"});
    Require(Text(value["file"])==data.inputs.source_member.file&&Text(value["sha256"])==data.inputs.source_member.sha256&&
        UInt(value["bytes"])==data.inputs.source_member.bytes,"Declared source member differs from authority");
}
void UnitsMatch(const CanonicalData& data,const Value& value) {
    Keys(value,{"mass","length","time","mass_to_kg","length_to_m","time_to_s"});
    const Units u{Text(value["mass"]),Text(value["length"]),Text(value["time"]),Real(value["mass_to_kg"]),Real(value["length_to_m"]),Real(value["time_to_s"])};
    CheckUnits(u);Require(SameUnits(u,data.inputs.units),"Declared source unit authority differs");
}
std::size_t Count(const Value& v,const char* name,std::size_t cap) {
    const auto n=UInt(Field(v,name));Require(n<=cap,"Declared source count exceeds bound");return std::size_t(n);
}
std::uint64_t Positive(const Value& v){auto n=UInt(v);Require(n,"Zero declared source ID");return n;}
}
void ReadDeclaredCatalog(CanonicalData& d,const Value& doc) {
    using namespace array_json;
    Keys(doc,{"schema","purpose","source_format","member","units","formulation_scheme","keyword_annotations","counts","materials","sections","parts","arrays"});
    Require(Text(doc["schema"])==DeclaredCanonicalSchema&&Text(doc["purpose"])=="declared_shell_geometry_only_not_simulation_or_restart"&&
        (Text(doc["source_format"])=="robo_dyna.native_contact_scene.v1"||
         Text(doc["source_format"])=="robo_dyna.native_contact_scene.v2"||
         Text(doc["source_format"])=="robo_dyna.native_contact_scene.v3")&&Text(doc["formulation_scheme"])=="openradioss_property_type1_ishell"&&
        Text(doc["keyword_annotations"])=="not_applicable_zero_channels","Unknown declared shell source semantics");
    Member(d,doc["member"]);UnitsMatch(d,doc["units"]);
    const auto& counts=doc["counts"];Keys(counts,{"nodes","shells","solids","beams"});
    d.canonical_nodes=Count(counts,"nodes",d.limits.nodes);d.canonical_shells=Count(counts,"shells",d.limits.parents);
    Require(d.canonical_nodes&&d.canonical_shells&&!Count(counts,"solids",0)&&!Count(counts,"beams",0),"Declared source is not a complete shell-only scope");
    const auto& materials=doc["materials"];const auto& sections=doc["sections"];const auto& parts=doc["parts"];
    Require(materials.IsArray()&&!materials.Empty()&&materials.Size()<=65536&&sections.IsArray()&&!sections.Empty()&&sections.Size()<=65536&&
        parts.IsArray()&&!parts.Empty()&&parts.Size()<=65536,"Declared material/section/part table exceeds cap");
    std::set<std::uint64_t> mids;std::map<std::uint64_t,unsigned> sids;std::map<std::uint64_t,PartDeclaration> pids;
    for(const auto& m:materials.GetArray()){Keys(m,{"source_material_id","source_law"});Require(Text(m["source_law"])=="LAW44","Unknown declared material source law");Require(mids.insert(Positive(m["source_material_id"])).second,"Duplicate declared material");}
    for(const auto& s:sections.GetArray()) {
        Keys(s,{"source_section_id","source_formulation_code","source_nip"});
        const auto code=UInt(s["source_formulation_code"]);
        Require(code==24&&UInt(s["source_nip"])==3&&sids.emplace(Positive(s["source_section_id"]),unsigned(code)).second,"Unknown or duplicate declared shell section");
    }
    for(const auto& p:parts.GetArray()) {
        Keys(p,{"source_part_id","source_material_id","source_section_id"});PartDeclaration part;
        part.part=Positive(p["source_part_id"]);part.material=Positive(p["source_material_id"]);part.section=Positive(p["source_section_id"]);
        Require(mids.count(part.material)&&sids.count(part.section),"Declared part references missing material/section");
        part.source_elform=sids.at(part.section);part.shell_section=true;
        Require(pids.emplace(part.part,part).second,"Duplicate declared part");
    }
    for(const auto& part:pids)d.parts.push_back(part.second);
    ReadCanonicalArrays(d,doc["arrays"],d.canonical_nodes,d.canonical_shells,0,0);
}
void CheckDeclaredMemberFormat(const Value& canonical,const std::string& bytes) {
    // Only the declared source envelope is interpreted here. Geometry and
    // constitutive admission remain in their owning source/physical factories.
    const auto member=Parse(bytes,4u<<20);const auto format=Text(canonical["source_format"]);
    if(format=="robo_dyna.native_contact_scene.v3")
        Keys(member,{"schema","units","wall","patch","material","thickness_mm","run","contact_surface","coupling"});
    else if(format=="robo_dyna.native_contact_scene.v2")
        Keys(member,{"schema","units","wall","patch","material","thickness_mm","run","contact_surface"});
    else Keys(member,{"schema","units","wall","patch","material","thickness_mm","run"});
    Require(Text(member["schema"])==format,"Declared source-format provenance differs from authenticated member");
}
void ReadDeclaredScope(CanonicalData& d,const Value& doc) {
    using namespace array_json;
    Keys(doc,{"schema","purpose","selection","canonical_sha256","member","units","coverage"});
    Require(Text(doc["schema"])==DeclaredScopeSchema&&Text(doc["purpose"])=="complete_declared_shell_source_only"&&
        Text(doc["selection"])=="all_declared_shells"&&d.inputs.tire_policy=="retain_all"&&
        Text(doc["canonical_sha256"])==d.inputs.canonical_manifest.sha256,"Declared source selection/identity differs");
    Member(d,doc["member"]);UnitsMatch(d,doc["units"]);
    for(const auto& part:d.parts)d.selected_parts.push_back(part.part);
    const auto& c=doc["coverage"];Keys(c,{"nodes","shells","q4","t3","retained_shell_ids_sha256","selected_node_ids_sha256","excluded_shell_ids_sha256"});
    d.retained_nodes=Count(c,"nodes",d.limits.nodes);d.retained_shells=Count(c,"shells",d.limits.parents);
    d.retained_q4=Count(c,"q4",d.retained_shells);d.retained_t3=Count(c,"t3",d.retained_shells);
    Require(d.retained_nodes==d.canonical_nodes&&d.retained_shells==d.canonical_shells&&d.retained_q4+d.retained_t3==d.retained_shells,
        "Declared all-shell source coverage is incomplete");
    d.retained_shell_ids_sha256=Text(c["retained_shell_ids_sha256"]);d.selected_node_ids_sha256=Text(c["selected_node_ids_sha256"]);
    d.excluded_shell_ids_sha256=Text(c["excluded_shell_ids_sha256"]);
    for(const auto* hash:{&d.retained_shell_ids_sha256,&d.selected_node_ids_sha256,&d.excluded_shell_ids_sha256})arrays::CheckHash(*hash);
}
void CheckDeclaredAnnotations(const CanonicalData& d) {
    // These old binary channels describe keyword-line/blank/code annotations.
    // The named JSON source does not invent such records; its schema declares
    // the channels unavailable and requires canonical zeros in every slot.
    const auto zero=[&](const auto& values){for(const auto value:values)Require(value==0,"Declared source fabricated keyword annotations");};
    for(const char* name:{"node_source_lines","shells_source_lines","solids_source_lines","beams_source_lines"}) {
        const auto& a=FindArray(d,name);zero(arrays::Decode<std::uint32_t>(a.descriptor,a.bytes));
    }
    for(const char* name:{"node_blank_masks","shells_blank_masks","solids_blank_masks","beams_blank_masks"}) {
        const auto& a=FindArray(d,name);zero(arrays::Decode<std::uint16_t>(a.descriptor,a.bytes));
    }
    const auto& a=FindArray(d,"node_codes");zero(arrays::Decode<std::int32_t>(a.descriptor,a.bytes));
}
} // namespace crash::output::full_shell::source::detail
