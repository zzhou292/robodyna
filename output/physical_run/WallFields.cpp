#include "RunArchive.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
records::source::BundleRequest MakeWallRequest(const records::Context& context,std::uint64_t intervals,
    double duration,std::size_t samples,std::size_t total_cap) {
    auto request=MakeRequest(context,intervals,duration,samples,total_cap);
    for(const auto* name:WallFiles)request.archive.static_files.push_back({name,WallFileCap});
    records::activity::PlanWithActivity(context,request.archive,"parent-activity.json");
    return request;
}
Document WallDocument(const WallReceipt& receipt) {
    Require(receipt.source_instance_id && receipt.wall_binding_id,"Missing physical wall source identity");
    arrays::CheckHash(receipt.source_mapping_sha256);
    Document d;d.SetObject();String(d,"profile",WallProfile);
    Integer(d,"source_instance_id",receipt.source_instance_id);Integer(d,"wall_binding_id",receipt.wall_binding_id);
    String(d,"source_mapping_sha256",receipt.source_mapping_sha256);
    Value files(rapidjson::kArrayType);
    for(std::size_t i=0;i<WallFiles.size();++i) {
        const auto& file=receipt.files[i];
        Require(file.file==WallFiles[i] && file.bytes && file.bytes<=WallFileCap,"Physical wall artifact extent/name differs");
        Value row;row.CopyFrom(FileDocument(file),d.GetAllocator());files.PushBack(row,d.GetAllocator());
    }
    d.AddMember("files",files,d.GetAllocator());return d;
}
WallReceipt ReadWallDocument(const Value& value) {
    using namespace array_json;
    Keys(value,{"profile","source_instance_id","wall_binding_id","source_mapping_sha256","files"});
    Require(Text(value["profile"])==WallProfile && value["files"].IsArray() &&
        value["files"].Size()==WallFiles.size(),"Unsupported physical wall receipt");
    WallReceipt out;
    out.source_instance_id=UInt(value["source_instance_id"]);out.wall_binding_id=UInt(value["wall_binding_id"]);
    out.source_mapping_sha256=Text(value["source_mapping_sha256"]);
    for(std::size_t i=0;i<out.files.size();++i)out.files[i]=ReadFileRecord(value["files"][i]);
    WallDocument(out);return out;
}
} // namespace crash::output::physical_run
