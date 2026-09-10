#include "WallArtifactFileIO.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"

namespace crash::output::assembly::wall_files {
namespace {
struct CountStream {
    using Ch=char;
    std::size_t count=0,cap=0;
    void Put(char) {Require(count<cap,"Assembly JSON exceeds its explicit file capacity");++count;}
    void Flush() {}
};
}
std::size_t JsonBytes(const Document& d,std::size_t cap) {
    Require(cap&&cap<=kArtifactFileCap,"Invalid assembly JSON capacity");CountStream stream{0,cap};
    rapidjson::PrettyWriter<CountStream> writer(stream);Require(d.Accept(writer),"Assembly JSON serialization failed");
    stream.Put('\n');return stream.count;
}
void WriteJsonBounded(const std::filesystem::path& path,const Document& d,std::size_t cap) {
    JsonBytes(d,cap);WriteJson(path,d);
}
void PublishManifest(const std::filesystem::path& directory,const Document& d,std::size_t cap) {
    Require(d.IsObject()&&d.HasMember("schema")&&d["schema"].IsString()&&d["schema"].GetString()==std::string(WallArtifactSchema)&&
        d.HasMember("kind")&&d["kind"].IsString()&&d["kind"].GetString()==std::string(WallArtifactKind)&&
        d.HasMember("status")&&d["status"].IsString()&&d["status"].GetString()==std::string("completed")&&
        d.HasMember("artifacts")&&d["artifacts"].IsArray()&&cap&&cap<=kArtifactExtendedTotalCap,
        "Invalid assembly manifest scope");
    std::size_t total=JsonBytes(d,WallManifestBytes);Require(total<=cap,"Assembly manifest exceeds aggregate capacity");
    for(const auto& e:d["artifacts"].GetArray()) {
        Require(e.IsObject()&&e.HasMember("file")&&e["file"].IsString()&&e.HasMember("bytes")&&e["bytes"].IsUint64()&&
            e.HasMember("sha256")&&e["sha256"].IsString(),"Invalid closed assembly artifact entry");
        const std::string name=e["file"].GetString();Require(!name.empty()&&name.size()<=255&&name!="."&&name!=".."&&
            name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")==std::string::npos,
            "Invalid assembly artifact basename");
        Require(std::filesystem::symlink_status(directory/name).type()==std::filesystem::file_type::regular,
            "Assembly manifest requires closed regular artifacts");
        const auto bytes=ReadBounded(directory/name,kArtifactFileCap);
        Require(bytes.size()==e["bytes"].GetUint64()&&Sha256(bytes)==e["sha256"].GetString()&&bytes.size()<=cap-total,
            "Assembly artifact changed or exceeded capacity before completion");total+=bytes.size();
    }
    Require(!std::filesystem::exists(directory/"manifest.json"),"Assembly completion marker already exists");
    WriteJsonBounded(directory/"manifest.pending.json",d,WallManifestBytes);
    std::filesystem::rename(directory/"manifest.pending.json",directory/"manifest.json");
}
} // namespace crash::output::assembly::wall_files
