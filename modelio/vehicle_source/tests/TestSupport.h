#pragma once
#include "../VehicleSourcePlan.h"
#include "modelio/source_assembly/SourceAssembly.h"
#include "modelio/source_assembly/tests/SectionTestSupport.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include <gtest/gtest.h>
#include <functional>

namespace crash::modelio::vehicle::test {
inline const source::CanonicalSource& Canonical() {
    static const auto source=[] {
        auto in=output::full_shell::source::test::ActualInputs();
        in.scope_report.bytes=13212691;
        in.scope_report.sha256="fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
        return source::CanonicalSource::Read(in);
    }();return source;
}
inline std::filesystem::path Path() {
    const auto* p=std::getenv("ROBO_VEHICLE_DECLARATIONS");output::Require(p&&*p,"Missing explicit vehicle declarations fixture");return p;
}
inline const std::string& Bytes() {
    static const auto bytes=[] {
        auto value=output::ReadBounded(Path(),3648589);
        output::Require(value.size()==3648589&&output::Sha256(value)==
            "a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d","Frozen vehicle declaration fixture changed");
        return value;
    }();return bytes;
}
inline assembly::ArtifactIdentity Identity(const std::string& bytes) {return {bytes.size(),output::Sha256(bytes)};}
inline const VehicleSourcePlan& Plan() {
    static const auto plan=VehicleSourcePlan::ReadBytes(Canonical(),Bytes(),Identity(Bytes()));return plan;
}
inline std::string Alter(const std::function<void(output::Document&)>& edit) {
    output::Document doc;doc.Parse<rapidjson::kParseFullPrecisionFlag>(Bytes().data(),Bytes().size());
    output::Require(!doc.HasParseError(),"Invalid frozen vehicle test fixture");edit(doc);
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(doc.Accept(writer),"Invalid vehicle mutation");return {buffer.GetString(),buffer.GetSize()};
}
inline const output::Value& Typed(const output::Document& d){return d["supported_declarations"]["declarations"];}
} // namespace crash::modelio::vehicle::test
