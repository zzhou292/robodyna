#pragma once
#include "../VehicleSectionResolution.h"
#include "modelio/vehicle_source/tests/TestSupport.h"

namespace crash::modelio::vehicle::test {
inline const std::string& ResolutionBytes() {
    static const auto bytes = [] {
        const auto* path = std::getenv("ROBO_VEHICLE_RESOLUTION");
        output::Require(path && *path, "Missing explicit vehicle resolution fixture");
        auto value = output::ReadBounded(path, 4 * 1024 * 1024);
        output::Require(value.size() == 150581 && output::Sha256(value) ==
            "49952c2f2ecb982ea7e8e53c6af2f1188a09c7ef389bc114ae4dfa4e9ef50006",
            "Frozen original vehicle resolution fixture changed");
        return value;
    }();
    return bytes;
}
inline const VehicleSectionResolution& Resolution() {
    static const auto result = VehicleSectionResolution::ReadBytes(Plan(), ResolutionBytes(), Identity(ResolutionBytes()));
    return result;
}
inline std::string AlterResolution(const std::function<void(output::Document&)>& edit) {
    output::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag>(ResolutionBytes().data(), ResolutionBytes().size());
    output::Require(!doc.HasParseError(), "Invalid frozen resolution fixture");
    edit(doc);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(doc.Accept(writer), "Invalid resolution mutation");
    return {buffer.GetString(), buffer.GetSize()};
}
inline std::size_t PartIndex(std::uint64_t id) {
    const auto& parts = Plan().parts();
    const auto found = std::find_if(parts.begin(), parts.end(), [&](const auto& part) { return part.part_id == id; });
    output::Require(found != parts.end(), "Missing expected original source part");
    return found - parts.begin();
}
} // namespace crash::modelio::vehicle::test
