#pragma once
#include "TestSupport.h"

namespace crash::modelio::vehicle::test {
inline const std::string& GlassBytes() {
    static const auto bytes = [] {
        const auto* path = std::getenv("ROBO_VEHICLE_GLASS_RESOLUTION");
        const auto* sha = std::getenv("ROBO_VEHICLE_GLASS_SHA256");
        output::Require(path && *path && sha && std::string(sha).size() == 64,
                        "Missing explicit original glass resolution identity");
        auto value = output::ReadBounded(path,4*1024*1024);
        output::Require(output::Sha256(value) == sha,"Frozen original glass resolution changed");
        return value;
    }();
    return bytes;
}
inline const VehicleSectionResolution& GlassResolution() {
    static const auto result = VehicleSectionResolution::ReadBytes(Plan(),GlassBytes(),Identity(GlassBytes()));
    return result;
}
inline std::string AlterGlass(const std::function<void(output::Document&)>& edit) {
    output::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag>(GlassBytes().data(),GlassBytes().size());
    output::Require(!doc.HasParseError(),"Invalid frozen glass fixture");
    edit(doc);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(doc.Accept(writer),"Invalid glass mutation");
    return {buffer.GetString(),buffer.GetSize()};
}
} // namespace crash::modelio::vehicle::test
