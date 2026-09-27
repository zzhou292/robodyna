#pragma once
#include "modelio/native_spring_ids/tests/Fixture.h"
#include "../Internal.h"
namespace crash::modelio::solid_control::test {
using native_spring_ids::test::FileBuilder;
using native_spring_ids::test::Card;
struct Fixture {
    native_spring_ids::test::Fixture arrays;
    std::vector<std::pair<std::string,std::string>> storage;
    explicit Fixture(bool nested = false, bool bad_option = false) {
        arrays.canonical.parts = {{19,401,702,0,false},{91,401,701,0,false}};
        const std::uint64_t solids[]{501,91,1,2,3,4,5,6,7,8,502,19,1,2,3,4,5,6,7,8};
        arrays.Add<std::uint64_t>("solids_records",solids,20,10);
        FileBuilder combine("combine.key"), auxiliary("set-yaris-coarse-v1l.key");
        combine.Add("*KEYWORD");
        combine.Add("*CONTACT_INTERIOR",{bad_option ? Card({5,1}) : Card({5})});
        combine.Add("*END");
        auxiliary.Add("*KEYWORD");
        if (nested) {
            auxiliary.Add("*SET_PART_ADD",{Card({5}),Card({6})});
            auxiliary.Add("*SET_PART_ADD",{Card({6}),Card({7})});
            auxiliary.Add("*SET_PART_LIST_TITLE",{"requested",Card({7}),Card({91})});
        } else auxiliary.Add("*SET_PART_LIST_TITLE",{"requested",Card({5}),Card({91})});
        auxiliary.Add("*END");
        output::Document document,files;document.SetObject();files.SetObject();
        for (const auto* file : {&combine,&auxiliary}) {
            output::array_json::Child(files,file->name.c_str(),file->Metadata());
            storage.emplace_back(file->name,file->bytes);
        }
        output::array_json::Child(document,"source_files",files);
        rapidjson::StringBuffer text;rapidjson::Writer<rapidjson::StringBuffer> writer(text);document.Accept(writer);
        arrays.canonical.canonical_bytes.assign(text.GetString(),text.GetSize());
    }
    ids::ImportMembers Members() const {
        ids::ImportMembers out;
        for (const auto& [name,bytes] : storage) out.members.push_back({name,bytes});
        return out;
    }
    DirectData Read() const { return detail::ReadDirect(arrays.canonical,Members(),1u<<20); }
};
} // namespace crash::modelio::solid_control::test
