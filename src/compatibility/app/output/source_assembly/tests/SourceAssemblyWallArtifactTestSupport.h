#pragma once
#include "SourceAssemblyWallFieldTestSupport.h"
#include <cstdlib>

namespace crash::output::assembly::test {
struct Directory {
    std::filesystem::path base,path;
    Directory() {std::string pattern=(std::filesystem::temp_directory_path()/"assembly-wall-live-output-XXXXXX").string();
        std::vector<char> chars(pattern.begin(),pattern.end());chars.push_back(0);auto* made=::mkdtemp(chars.data());Require(made,"Output test directory failed");base=made;path=base/"archive";}
    ~Directory() {std::error_code e;std::filesystem::remove_all(base,e);}
};
inline Document ReadJson(const std::filesystem::path& path) {const auto bytes=ReadBounded(path,kArtifactFileCap);Document d;d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());Require(!d.HasParseError(),"Invalid test artifact JSON");return d;}
} // namespace crash::output::assembly::test
