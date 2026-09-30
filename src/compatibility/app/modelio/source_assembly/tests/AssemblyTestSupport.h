#pragma once
#include "modelio/source_assembly/SourceAssemblyShellInput.h"
#include "modelio/source_assembly/SourceAssemblyMaterialInput.h"
#include "output/ArtifactIO.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace crash::modelio::assembly::test {
inline std::filesystem::path FixturePath() {
    const auto* path = std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY");
    output::Require(path && *path, "Explicit assembly fixture path is required");
    return path;
}
inline SourceAssembly Load() { return SourceAssembly::Read(FixturePath(), PinnedYarisSixPartInventory()); }
inline std::string FixtureBytes() { return output::ReadBounded(FixturePath(), PinnedYarisSixPartInventory().bytes); }
class Scratch {
  public:
    Scratch() {
        std::string pattern = (std::filesystem::temp_directory_path() / "robo-dyna-assembly-test-XXXXXX").string();
        char* result = ::mkdtemp(pattern.data());
        output::Require(result, "Could not create assembly test directory"); path_ = result;
    }
    ~Scratch() { std::error_code ignored; std::filesystem::remove_all(path_, ignored); }
    std::filesystem::path Write(const std::string& bytes) {
        const auto path = path_ / (std::to_string(next_++) + ".json"); output::WriteBytes(path, bytes); return path;
    }
  private:
    std::filesystem::path path_;
    unsigned next_ = 0;
};
inline std::string Alter(const std::function<void(output::Document&)>& edit) {
    const auto bytes = FixtureBytes(); output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag>(bytes.data(), bytes.size());
    output::Require(!document.HasParseError(), "Bad pinned test inventory"); edit(document);
    rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(document.Accept(writer), "Could not serialize assembly corruption fixture");
    return {buffer.GetString(), buffer.GetSize()};
}
inline ArtifactIdentity ExplicitTestIdentity(const std::string& bytes) { return {bytes.size(), output::Sha256(bytes)}; }
inline void SameBits(double a, double b) { EXPECT_EQ(output::Bits(a), output::Bits(b)); }
}  // namespace crash::modelio::assembly::test
