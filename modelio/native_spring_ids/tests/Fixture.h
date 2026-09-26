#pragma once
#include "../Internal.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
namespace crash::modelio::native_spring_ids::test {
inline std::string Card(std::initializer_list<std::uint64_t> values, unsigned width = 10) {
    std::ostringstream text; for (const auto value : values) text << std::setw(width) << value; return text.str();
}
struct FileBuilder {
    std::string name, bytes;
    std::size_t next_line = 1;
    output::Document blocks, counts;
    explicit FileBuilder(std::string value) : name(std::move(value)) { blocks.SetArray(); counts.SetObject(); }
    void Add(const char* keyword, const std::vector<std::string>& cards = {}) {
        const auto first = next_line; std::string raw = std::string(keyword)+"\n"; ++next_line;
        std::size_t records = 0;
        for (const auto& card : cards) { raw += card+"\n"; ++next_line; records += !card.empty(); }
        output::Document row; row.SetObject();
        output::String(row,"file",name); output::String(row,"keyword",keyword);
        output::Integer(row,"first_line",first); output::Integer(row,"last_line",next_line-1);
        output::Integer(row,"data_records",records); output::String(row,"source_block_sha256",output::Sha256(raw));
        output::Value copy; copy.CopyFrom(row,blocks.GetAllocator()); blocks.PushBack(copy,blocks.GetAllocator());
        if (counts.HasMember(keyword)) counts[keyword].SetUint64(counts[keyword].GetUint64()+1);
        else output::Integer(counts,keyword,1);
        bytes += raw;
    }
    output::Document Metadata() const {
        output::Document result; result.SetObject(); output::String(result,"sha256",output::Sha256(bytes));
        output::array_json::Child(result,"blocks",blocks); output::array_json::Child(result,"keyword_counts",counts); return result;
    }
};
struct Fixture {
    source::CanonicalData canonical;
    std::vector<std::pair<std::string,std::string>> storage;
    std::size_t next_array = 0;
    explicit Fixture(std::uint64_t discrete = 100, bool wall_spring = false) {
        FileBuilder main("yaris-coarse-v1l.key"), auxiliary("auxiliary.key"), wall("wall.key"), root("combine.key");
        main.Add("*KEYWORD");
        main.Add("*PART", {"beam",Card({10,10,10})});
        main.Add("*SECTION_BEAM", {Card({10,9}),Card({1,1})});
        main.Add("*MAT_SPOTWELD", {Card({10,1,1}),Card({1})});
        const auto beam_line = main.next_line+1;
        main.Add("*ELEMENT_BEAM", {Card({20,10,1,2,3,0,0,0,0,2},8)});
        main.Add("*ELEMENT_DISCRETE", {Card({90,12,4,5},8)});
        main.Add("*CONSTRAINED_SPOTWELD_ID", {Card({8}),Card({1,2}),Card({7}),Card({2,3})});
        main.Add("*CONSTRAINED_JOINT_SPHERICAL_ID", {Card({31}),Card({4,5})});
        main.Add("*END");
        auxiliary.Add("*KEYWORD"); auxiliary.Add("*ELEMENT_DISCRETE", {Card({discrete,12,4,5},8)}); auxiliary.Add("*END");
        wall.Add("*KEYWORD");
        if (wall_spring) wall.Add("*ELEMENT_DISCRETE", {Card({500,12,4,5},8)});
        else wall.Add("*ELEMENT_SHELL", {Card({100000000,12,1,2,3,4},8)});
        wall.Add("*END");
        root.Add("*KEYWORD"); root.Add("*INCLUDE", {main.name}); root.Add("*INCLUDE", {auxiliary.name});
        root.Add("*DEFINE_TRANSFORMATION", {Card({9}),"    TRANSL       1.0       0.0       0.0"});
        root.Add("*INCLUDE_TRANSFORM", {wall.name,Card({10000,10000,10000,10000,10000,10000,10000}),Card({10000}),"",Card({9})});
        root.Add("*END");
        output::Document document, files, materials, sections;
        document.SetObject(); files.SetObject(); materials.SetArray(); sections.SetArray();
        for (const auto* file : {&main,&auxiliary,&wall,&root}) {
            const auto metadata = file->Metadata(); output::array_json::Child(files,file->name.c_str(),metadata);
            storage.emplace_back(file->name,file->bytes);
        }
        output::Document material; material.SetObject(); output::Integer(material,"source_material_id",10); output::String(material,"keyword","*MAT_SPOTWELD");
        output::Value copy; copy.CopyFrom(material,materials.GetAllocator()); materials.PushBack(copy,materials.GetAllocator());
        output::Document section; section.SetObject();
        output::Integer(section,"source_section_id",10); output::String(section,"keyword","*SECTION_BEAM");
        output::String(section,"formulation_field_raw","9");
        copy.CopyFrom(section,sections.GetAllocator()); sections.PushBack(copy,sections.GetAllocator());
        output::array_json::Child(document,"source_files",files); output::array_json::Child(document,"materials",materials);
        output::array_json::Child(document,"sections",sections);
        rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer); document.Accept(writer);
        canonical.canonical_bytes.assign(buffer.GetString(),buffer.GetSize());
        canonical.inputs.source_member={main.name,output::Sha256(main.bytes),main.bytes.size()};
        canonical.inputs.canonical_manifest={"manifest.json",output::Sha256(canonical.canonical_bytes),canonical.canonical_bytes.size()};
        // The real canonical catalog intentionally stores ELFORM only for shells.
        canonical.parts.push_back({10,10,10,0,false});
        const std::uint64_t beam[]{20,10,1,2,3,0,0,0,0,2}; Add<std::uint64_t>("beams_records",beam,10,10);
        const auto line=std::uint32_t(beam_line); Add<std::uint32_t>("beams_source_lines",&line,1,1);
    }
    template<class T> void Add(const char* name, const T* data, std::size_t count, std::size_t columns) {
        source::NamedArray array; array.name=name; array.descriptor.file=std::string(name)+".bin";
        array.descriptor.layout={output::arrays::detail::Type<T>::value,count/columns,columns,{}};
        array.bytes=output::arrays::Encode<T>(array.descriptor.layout,data,count);
        array.descriptor.bytes=array.bytes.size(); array.descriptor.sha256=output::Sha256(array.bytes);
        if (next_array >= canonical.arrays.size()) throw std::runtime_error("Fixture array inventory exceeded");
        canonical.arrays[next_array++] = std::move(array);
    }
    ImportMembers Input() const {
        ImportMembers result; result.profile=Profile::DirectKeywordR14FreshRadiossPoSortById; result.entry_member="combine.key";
        for (const auto& member : storage) result.members.push_back({member.first,member.second});
        return result;
    }
    ContextData Context() const { return detail::BuildContext(canonical,Input(),{}); }
};
inline std::vector<SourceRow> Retained(const ContextData& context) {
    std::vector<SourceRow> result;
    const auto add = [&](const std::vector<SourceRow>& rows) {
        std::size_t index=0;
        for (auto row : rows) if (row.kind != SourceKind::DiscreteNamespaceOnly) {
            row.source_index=index++; row.physical_participant=true; result.push_back(std::move(row));
        }
    };
    add(context.precursors); add(context.welds); add(context.joints); return result;
}
} // namespace crash::modelio::native_spring_ids::test
