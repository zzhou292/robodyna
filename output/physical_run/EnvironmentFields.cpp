#include "EnvironmentArtifacts.h"
#include "RunArchive.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
records::source::BundleRequest MakeEnvironmentRequest(const records::Context& context,std::uint64_t intervals,
    double duration,std::size_t samples,std::size_t total_cap) {
    auto request=MakeRequest(context,intervals,duration,samples,total_cap);
    for(const auto* name:EnvironmentFiles)request.archive.static_files.push_back({name,EnvironmentFileCap});
    records::activity::PlanWithActivity(context,request.archive,"parent-activity.json");return request;
}
Document EnvironmentDocument(const EnvironmentReceipt& receipt) {
    Require(receipt.source_instance_id && receipt.wall_binding_id && receipt.part_id,"Missing declared environment identity");
    arrays::CheckHash(receipt.source_mapping_sha256);
    Document doc;doc.SetObject();String(doc,"profile",EnvironmentProfile);
    Integer(doc,"source_instance_id",receipt.source_instance_id);Integer(doc,"wall_binding_id",receipt.wall_binding_id);
    Integer(doc,"part_id",receipt.part_id);String(doc,"source_mapping_sha256",receipt.source_mapping_sha256);
    Value files(rapidjson::kArrayType);
    for(unsigned i=0;i<3;++i) {
        const auto& file=receipt.files[i];Require(file.file==EnvironmentFiles[i] && file.bytes && file.bytes<=EnvironmentFileCap,
            "Declared environment file name/extent differs");
        Value row;row.CopyFrom(FileDocument(file),doc.GetAllocator());files.PushBack(row,doc.GetAllocator());
    }
    doc.AddMember("files",files,doc.GetAllocator());return doc;
}
EnvironmentReceipt ReadEnvironmentDocument(const Value& value) {
    using namespace array_json;
    Keys(value,{"profile","source_instance_id","wall_binding_id","part_id","source_mapping_sha256","files"});
    Require(Text(value["profile"])==EnvironmentProfile && value["files"].IsArray() && value["files"].Size()==3,
        "Unknown declared environment receipt");
    EnvironmentReceipt out;out.source_instance_id=UInt(value["source_instance_id"]);out.wall_binding_id=UInt(value["wall_binding_id"]);
    out.part_id=UInt(value["part_id"]);out.source_mapping_sha256=Text(value["source_mapping_sha256"]);
    for(unsigned i=0;i<3;++i)out.files[i]=ReadFileRecord(value["files"][i]);
    EnvironmentDocument(out);return out;
}
}
