#include "Metadata.h"
#include "output/full_shell/FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"
#include <set>
namespace crash::output::physical_run {
static Document LegacyManifestDocument(const Manifest& m) {
    Document d;d.SetObject();String(d,"schema",Schema);
    String(d,"publication","closed_accepted_prefix_or_horizon");
    if(m.wall)array_json::Child(d,"wall_case",WallDocument(*m.wall));
    array_json::Child(d,"identity",records::IdentityDocument(m.identity));
    array_json::Child(d,"configuration",FileDocument(m.configuration));array_json::Child(d,"index",FileDocument(m.index));
    array_json::Child(d,"source_bundle",FileDocument(m.source));array_json::Child(d,"parent_activity",FileDocument(m.activity_declaration));
    Integer(d,"whole_run_forecast_bytes",m.forecast_bytes);
    Value files(rapidjson::kArrayType);
    for(const auto& f:m.inventory) {Value value;value.CopyFrom(FileDocument(f),d.GetAllocator());files.PushBack(value,d.GetAllocator());}
    d.AddMember("files",files,d.GetAllocator());return d;
}
static Manifest ReadLegacyManifest(const Value& v) {
    using namespace array_json;
    const bool wall=v.IsObject() && v.HasMember("wall_case");
    if(wall)Keys(v,{"schema","publication","identity","configuration","index","source_bundle","parent_activity","whole_run_forecast_bytes","files","wall_case"});
    else Keys(v,{"schema","publication","identity","configuration","index","source_bundle","parent_activity","whole_run_forecast_bytes","files"});
    Require(Text(v["schema"])==Schema && Text(v["publication"])=="closed_accepted_prefix_or_horizon","Unsupported physical run archive");
    Manifest m;
    if(wall)m.wall=ReadWallDocument(v["wall_case"]);
    m.identity=records::ParseIdentity(v["identity"]);m.configuration=ReadFileRecord(v["configuration"]);
    m.index=ReadFileRecord(v["index"]);m.source=ReadFileRecord(v["source_bundle"]);m.activity_declaration=ReadFileRecord(v["parent_activity"]);
    m.forecast_bytes=UInt(v["whole_run_forecast_bytes"]);
    Require(m.forecast_bytes && m.forecast_bytes<=records::FullRunByteCap && v["files"].IsArray() &&
        v["files"].Size()<=kArtifactInventoryCap,"Physical run forecast/inventory exceeds cap");
    std::set<std::string> names;
    for(const auto& row:v["files"].GetArray()) {
        auto f=ReadFileRecord(row);Require(f.file!="manifest.json" && names.insert(f.file).second,"Physical inventory aliases");
        m.inventory.push_back(std::move(f));
    }
    for(const auto* f:{&m.configuration,&m.index,&m.source,&m.activity_declaration}) {
        bool found=false;
        for(const auto& entry:m.inventory)if(entry.file==f->file)found=entry.sha256==f->sha256 && entry.bytes==f->bytes;
        Require(found,"Physical static record is absent from exact inventory");
    }
    Require(m.configuration.file=="configuration.json" && m.index.file=="frame-index.json" &&
        m.activity_declaration.file=="parent-activity.json","Physical run static names differ");return m;
}
Document ManifestDocument(const Manifest& m) {
    auto doc=LegacyManifestDocument(m);
    if(m.environment) {
        Require(!m.wall,"Declared and legacy wall receipts cannot occupy the same static role");
        doc["schema"].SetString(EnvironmentRunSchema,doc.GetAllocator());
        array_json::Child(doc,"environment_wall",EnvironmentDocument(*m.environment));
    }
    return doc;
}
Manifest ReadManifest(const Value& value) {
    if(!value.IsObject() || !value.HasMember("environment_wall"))return ReadLegacyManifest(value);
    using namespace array_json;
    Require(value.HasMember("schema") && !value.HasMember("wall_case") && Text(value["schema"])==EnvironmentRunSchema,
        "Unknown declared environment manifest");
    const auto environment=ReadEnvironmentDocument(value["environment_wall"]);
    Document legacy;legacy.CopyFrom(value,legacy.GetAllocator());legacy.RemoveMember("environment_wall");
    legacy["schema"].SetString(Schema,legacy.GetAllocator());
    auto out=ReadLegacyManifest(legacy);out.environment=environment;
    ManifestDocument(out);return out;
}
} // namespace crash::output::physical_run
