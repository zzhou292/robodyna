#include "Internal.h"
#include <algorithm>
#include <set>

namespace crash::modelio::vehicle::detail {
namespace {
using Index=std::map<std::uint64_t,const Value*>;
Index Rows(const Value& values,const char* key) {
    Require(values.IsArray(),"Missing original declaration array");Index index;
    for(const auto& row:values.GetArray())Require(index.emplace(Unsigned(row,key),&row).second,"Duplicate source declaration");
    return index;
}
const Value& Find(const Index& index,std::uint64_t id) {
    const auto it=index.find(id);Require(it!=index.end(),"Missing original declaration identity");return *it->second;
}
template<class T> std::size_t FindTyped(const std::vector<T>& values,std::uint64_t id) {
    const auto it=std::lower_bound(values.begin(),values.end(),id,[](const T& v,std::uint64_t n){return v.id<n;});
    Require(it!=values.end()&&it->id==id,"Missing typed vehicle declaration");return it-values.begin();
}
std::set<std::uint64_t> ExpectedSupported(const Value& declarations,const Index& parts) {
    std::set<std::uint64_t> result;
    for(const auto& p:parts)if(Text(*p.second,"disposition")=="existing_declaration_adapter_candidate")result.insert(p.first);
    for(const char* name:{"linear_law44_candidates","layered_law1_candidates"})
        for(const auto& p:Member(Member(declarations,name),"parts").GetArray())
            if(Member(p,"part_and_section_candidate").IsTrue())result.insert(Unsigned(p,"source_part_id"));
    return result;
}
}
Declarations ReadDeclarations(const source::CanonicalData& source,const Value& doc,Limits limits) {
    output::Document scope;scope.Parse<rapidjson::kParseFullPrecisionFlag>(source.scope_bytes.data(),source.scope_bytes.size());
    Require(!scope.HasParseError(),"Invalid authenticated source scope");UniqueKeys(scope);
    const auto& original=Member(scope,"declarations");
    const auto parts=Rows(Member(original,"parts"),"source_part_id");const auto& tables=Member(original,"tables");
    const auto materials=Rows(Member(tables,"material"),"identity"),sections=Rows(Member(tables,"section"),"identity"),
               curves=Rows(Member(tables,"curve"),"identity");
    const auto expected=ExpectedSupported(original,parts);
    Declarations next;next.typed.schema=assembly::SectionInventorySchema;
    const auto& typed=Member(doc,"supported_declarations");TextIs(typed,"schema",assembly::SectionInventorySchema);
    assembly::ReadLimits read;read.parts=limits.parts;read.tables=limits.tables;read.curve_points=limits.curve_points;
    assembly::reader::ReadMaterialPolicy(typed,next.typed);assembly::reader::ReadDeclarations(typed,read,next.typed);
    for(const auto& p:next.typed.parts){
        const auto& original=Find(parts,p.id);CheckSource(p.source,original,"source_part_sha256");CheckTypedCards(p.source,p.cards);
        Require(p.title==Text(original,"title"),"Vehicle source part title changed");
    }
    for(const auto& m:next.typed.materials){CheckSource(m.source,Find(materials,m.id));CheckTypedCards(m.source,m.cards);}
    for(const auto& s:next.typed.sections){CheckSource(s.source,Find(sections,s.id));CheckTypedCards(s.source,s.cards);}
    for(const auto& c:next.typed.curves){CheckSource(c.source,Find(curves,c.id));CheckTypedCards(c.source,c.cards,20);}
    std::vector<bool> used_materials(next.typed.materials.size()),used_sections(next.typed.sections.size()),used_curves(next.typed.curves.size());
    const auto& rows=Array(doc,"parts",limits.parts,1);
    Require(rows.Size()==source.selected_parts.size(),"Complete selected vehicle part coverage changed");
    std::size_t supported=0;
    for(const auto& value:rows.GetArray()) {
        PartDisposition p;p.part_id=Unsigned(value,"part_id");p.material_id=Unsigned(value,"material_id");
        p.section_id=Unsigned(value,"section_id");p.shell_count=Unsigned(value,"shells",limits.parents);
        Require(p.part_id==source.selected_parts[next.parts.size()]&&p.shell_count,"Vehicle selected part ordering/count changed");
        const auto& canonical=source::FindPart(source,p.part_id);const auto& origin=Find(parts,p.part_id);
        Require(p.material_id==canonical.material&&p.section_id==canonical.section&&
            p.shell_count==Unsigned(Member(origin,"counts"),"shells"),"Vehicle part/material/section identity differs");
        const auto status=Text(value,"status");const bool available=status=="supported_declaration";
        Require((available||status=="unresolved")&&available==bool(expected.count(p.part_id)),"Vehicle declaration disposition changed");
        const auto& obligations=Array(value,"obligations",3);
        std::size_t mi=SIZE_MAX,si=SIZE_MAX;
        if(available) {
            Require(obligations.Empty()&&!value.HasMember("source_blocks"),"Supported declaration has unresolved source obligations");
            p.status=Disposition::SupportedDeclaration;p.typed_index=FindTyped(next.typed.parts,p.part_id);++supported;
            const auto& t=next.typed.parts[p.typed_index];
            Require(t.material_id==p.material_id&&t.section_id==p.section_id,"Typed vehicle part association changed");
            mi=FindTyped(next.typed.materials,p.material_id);si=FindTyped(next.typed.sections,p.section_id);
            used_materials[mi]=true;used_sections[si]=true;
            if(const auto id=next.typed.materials[mi].curve_id)used_curves[FindTyped(next.typed.curves,id)]=true;
        } else {
            Require(!obligations.Empty(),"Unresolved source declaration lacks obligations");
            std::set<std::string> stages;
            for(const auto& v:obligations.GetArray()) {
                Obligation o{Text(v,"stage"),Text(v,"reason")};
                Require((o.stage=="part"||o.stage=="section"||o.stage=="material")&&stages.insert(o.stage).second&&
                    !o.reason.empty()&&o.reason.size()<=1024,"Invalid unresolved source obligation");p.obligations.push_back(std::move(o));
            }
            const auto& blocks=Member(value,"source_blocks");unsigned b=0;
            for(const char* name:{"part","section","material"})p.unresolved_sources[b++]=Block(Member(blocks,name));
            CheckSource(p.unresolved_sources[0],origin,"source_part_sha256");
            CheckSource(p.unresolved_sources[1],Find(sections,p.section_id));CheckSource(p.unresolved_sources[2],Find(materials,p.material_id));
        }
        next.parts.push_back(std::move(p));next.materials.push_back(mi);next.sections.push_back(si);
    }
    Require(supported==next.typed.parts.size(),"Unreferenced typed vehicle part");
    for(const auto* used:{&used_materials,&used_sections,&used_curves})
        Require(std::all_of(used->begin(),used->end(),[](bool b){return b;}),"Unreferenced typed vehicle declaration");
    return next;
}
} // namespace crash::modelio::vehicle::detail
