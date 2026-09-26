#pragma once
#include "../Internal.h"
#include "modelio/native_spring_ids/tests/Fixture.h"
#include <iomanip>
#include <sstream>
namespace crash::cases::vehicle_wall::native::test {
namespace ns=modelio::native_spring_ids;
namespace source=output::full_shell::source;
namespace helper=ns::test;
inline Declaration Decl(){Declaration d;d.profile=Profile::EnvelopeFixedElasticV1;return d;}
inline source::Units Units(){return {"tonne","mm","s",1000,.001,1};}
inline AllocatedIds Ids(){return {{{101,102,103,104}},105,106,107,108,109,110,111};}
inline std::array<tlfea::contact::Vec3,2> Box(){return {{{0,-.75,0},{1,.75,1.5}}};}
inline std::string RealRow(std::uint64_t id,double rho,double young,double poisson) {
    std::ostringstream out;out<<std::setw(10)<<id<<std::setw(10)<<rho<<std::setw(10)<<young<<std::setw(10)<<poisson;return out.str();
}
struct NamespaceFixture {
    source::CanonicalData canonical;
    std::vector<std::pair<std::string,std::string>> storage;
    std::string wall_sha;
    std::size_t next=0;
    explicit NamespaceFixture(std::uint64_t offset=10000,const std::string& extra={}) {
        helper::FileBuilder main("main.key"),auxiliary("aux.key"),wall("wall.key"),entry("combine.key");
        main.Add("*KEYWORD");
        std::vector<std::string> nodes;
        for(unsigned id=1;id<=6;++id)nodes.push_back(helper::Card({id},8));
        main.Add("*NODE",nodes);
        main.Add("*PART",{"beam",helper::Card({10,10,10})});
        main.Add("*SECTION_BEAM",{helper::Card({10,9}),helper::Card({1,1})});
        main.Add("*MAT_SPOTWELD",{helper::Card({10,1,1}),helper::Card({1})});
        const auto beam_line=std::uint32_t(main.next_line+1);
        main.Add("*ELEMENT_BEAM",{helper::Card({20,10,1,2,3,0,0,0,0,2},8)});
        main.Add("*CONSTRAINED_SPOTWELD_ID",{helper::Card({8}),helper::Card({1,2})});
        main.Add("*CONSTRAINED_JOINT_SPHERICAL_ID",{helper::Card({31}),helper::Card({4,5})});
        if(!extra.empty())main.Add(extra.c_str(),{helper::Card({999999})});
        main.Add("*END");
        auxiliary.Add("*KEYWORD");auxiliary.Add("*ELEMENT_DISCRETE",{helper::Card({100,12,4,5},8)});auxiliary.Add("*END");
        wall.Add("*KEYWORD");wall.Add("*NODE",{helper::Card({1},8),helper::Card({2},8),helper::Card({3},8),helper::Card({4},8)});
        wall.Add("*PART",{"original display",helper::Card({13,13,13})});
        wall.Add("*SECTION_SHELL",{helper::Card({13,2}),"         1         1         1         1"});
        wall.Add("*MAT_RIGID",{RealRow(13,7.86e-12,200000,.3),helper::Card({1,7,7})});
        wall.Add("*ELEMENT_SHELL",{helper::Card({1001,13,1,2,3,4},8)});wall.Add("*END");
        entry.Add("*KEYWORD");entry.Add("*INCLUDE",{main.name});entry.Add("*INCLUDE",{auxiliary.name});
        entry.Add("*DEFINE_TRANSFORMATION",{helper::Card({9}),"    TRANSL       1.0       0.0       0.0"});
        entry.Add("*INCLUDE_TRANSFORM",{wall.name,helper::Card({offset,offset,offset,offset,offset,offset,offset}),helper::Card({offset}),"",helper::Card({9})});
        entry.Add("*END");
        output::Document doc,files,materials,sections;doc.SetObject();files.SetObject();materials.SetArray();sections.SetArray();
        for(const auto* file:{&main,&auxiliary,&wall,&entry}) {
            const auto metadata=file->Metadata();output::array_json::Child(files,file->name.c_str(),metadata);
            storage.emplace_back(file->name,file->bytes);
        }
        output::Document material;material.SetObject();output::Integer(material,"source_material_id",10);
        output::String(material,"keyword","*MAT_SPOTWELD");output::Value copy;
        copy.CopyFrom(material,materials.GetAllocator());materials.PushBack(copy,materials.GetAllocator());
        output::Document section;section.SetObject();output::Integer(section,"source_section_id",10);
        output::String(section,"keyword","*SECTION_BEAM");output::String(section,"formulation_field_raw","9");
        copy.CopyFrom(section,sections.GetAllocator());sections.PushBack(copy,sections.GetAllocator());
        output::array_json::Child(doc,"source_files",files);output::array_json::Child(doc,"materials",materials);output::array_json::Child(doc,"sections",sections);
        rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);doc.Accept(writer);
        canonical.canonical_bytes.assign(buffer.GetString(),buffer.GetSize());
        canonical.inputs.source_member={main.name,output::Sha256(main.bytes),main.bytes.size()};
        canonical.inputs.canonical_manifest={"manifest.json",output::Sha256(canonical.canonical_bytes),canonical.canonical_bytes.size()};
        canonical.inputs.units=Units();canonical.canonical_nodes=6;canonical.parts.push_back({10,10,10,0,false});
        const std::uint64_t ids[]{1,2,3,4,5,6},beam[]{20,10,1,2,3,0,0,0,0,2};
        Add("node_ids",ids,6,1);Add("beams_records",beam,10,10);Add("beams_source_lines",&beam_line,1,1);
        Add<std::uint64_t>("shells_records",nullptr,0,6);Add<std::uint64_t>("solids_records",nullptr,0,10);
        wall_sha=output::Sha256(wall.bytes);
    }
    template<class T> void Add(const char* name,const T* data,std::size_t count,std::size_t columns) {
        source::NamedArray a;a.name=name;a.descriptor.file=std::string(name)+".bin";
        a.descriptor.layout={output::arrays::detail::Type<T>::value,count/columns,columns,{}};
        a.bytes=output::arrays::Encode<T>(a.descriptor.layout,data,count);
        a.descriptor.bytes=a.bytes.size();a.descriptor.sha256=output::Sha256(a.bytes);canonical.arrays[next++]=std::move(a);
    }
    ns::ImportMembers Members() const {
        ns::ImportMembers input;input.profile=ns::Profile::DirectKeywordR14FreshRadiossPoSortById;input.entry_member="combine.key";
        for(const auto& member:storage)input.members.push_back({member.first,member.second});return input;
    }
    auto Build(Limits limits={}) const {
        const auto input=Members();const auto context=ns::detail::BuildContext(canonical,input,{});
        return detail::NamespaceValues(canonical,context,input,wall_sha,limits);
    }
};
} // namespace crash::cases::vehicle_wall::native::test
