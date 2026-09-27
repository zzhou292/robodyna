#include "Internal.h"
#include "output/physical_run/SourceInputFields.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <set>

namespace crash::output::recovered_frames {
static Document LegacyEncode(const Description& value) {
    Require(!value.reason.empty() && value.reason.size()<=4096,"Recovery stop reason is missing or too long");
    arrays::CheckHash(value.mapping_sha256);
    Require(!value.frames.empty() && value.frames.size()<=1001 && value.files.size()<=kArtifactInventoryCap,
        "Recovery sample/inventory capacity exceeded");
    Document d;d.SetObject();
    String(d,"schema",Schema);
    String(d,"purpose","recovered_recorded_samples_not_restart_or_continuous_trajectory");
    String(d,"interval_ledger","unavailable_after_interruption");
    Boolean(d,"horizon_complete",false);
    String(d,"stop_reason",value.reason);
    run::AppendSourceInputs(d,value.source);
    String(d,"mapping_sha256",value.mapping_sha256);
    array_json::Child(d,"configuration",run::FileDocument(value.configuration));
    array_json::Child(d,"source_bundle",run::FileDocument(value.source_bundle));
    array_json::Child(d,"parent_activity",run::FileDocument(value.activity_declaration));
    if(value.wall)array_json::Child(d,"wall_case",run::WallDocument(*value.wall));
    Value frames(rapidjson::kArrayType);
    for(const auto& frame:value.frames) {
        Document row;row.SetObject();
        array_json::Child(row,"stamp",run::StampDocument(frame.stamp));
        array_json::Child(row,"frame",run::FileDocument(frame.frame));
        array_json::Child(row,"activity",run::FileDocument(frame.activity));
        Value item;item.CopyFrom(row,d.GetAllocator());frames.PushBack(item,d.GetAllocator());
    }
    d.AddMember("frames",frames,d.GetAllocator());
    Value files(rapidjson::kArrayType);
    std::set<std::string> names;
    for(const auto& file:value.files) {
        Require(file.file!=DescriptorFilename && names.insert(file.file).second,"Recovery file aliases");
        Value row;row.CopyFrom(run::FileDocument(file),d.GetAllocator());files.PushBack(row,d.GetAllocator());
    }
    d.AddMember("files",files,d.GetAllocator());
    rapidjson::StringBuffer bytes;
    rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
    Require(d.Accept(writer) && bytes.GetSize()<=run::MetadataCap,"Recovery descriptor exceeds metadata cap");
    return d;
}
static Description LegacyDecode(const Value& d) {
    using namespace array_json;
    const bool wall=d.IsObject() && d.HasMember("wall_case");
    if(wall)Keys(d,{"schema","purpose","interval_ledger","horizon_complete","stop_reason","canonical_manifest",
        "scope_report","source_member","source_authority","mapping_sha256","configuration","source_bundle",
        "parent_activity","wall_case","frames","files"});
    else Keys(d,{"schema","purpose","interval_ledger","horizon_complete","stop_reason","canonical_manifest",
        "scope_report","source_member","source_authority","mapping_sha256","configuration","source_bundle",
        "parent_activity","frames","files"});
    Require(Text(d["schema"])==Schema &&
        Text(d["purpose"])=="recovered_recorded_samples_not_restart_or_continuous_trajectory" &&
        Text(d["interval_ledger"])=="unavailable_after_interruption" && d["horizon_complete"].IsBool() &&
        !d["horizon_complete"].GetBool(),"Unsupported recovered sample/completion claim");
    Description value;
    value.source=run::ParseSourceInputs(d);
    value.mapping_sha256=Text(d["mapping_sha256"]);value.reason=Text(d["stop_reason"]);
    value.configuration=run::ReadFileRecord(d["configuration"]);
    value.source_bundle=run::ReadFileRecord(d["source_bundle"]);
    value.activity_declaration=run::ReadFileRecord(d["parent_activity"]);
    if(wall)value.wall=run::ReadWallDocument(d["wall_case"]);
    Require(d["frames"].IsArray() && d["frames"].Size()<=1001 &&
        d["files"].IsArray() && d["files"].Size()<=kArtifactInventoryCap,"Recovery inventory exceeds cap");
    for(const auto& row:d["frames"].GetArray()) {
        Keys(row,{"stamp","frame","activity"});
        value.frames.push_back({run::ReadStamp(row["stamp"]),run::ReadFileRecord(row["frame"]),
            run::ReadFileRecord(row["activity"])});
    }
    for(const auto& row:d["files"].GetArray())value.files.push_back(run::ReadFileRecord(row));
    Encode(value);
    Require(value.configuration.file=="configuration.json" && value.source_bundle.file=="source.bundle.json" &&
        value.activity_declaration.file=="parent-activity.json","Recovery static names differ");
    return value;
}
Document Encode(const Description& value) {
    Require(!(value.wall && value.environment),"Recovered static wall profiles are mutually exclusive");
    auto document=LegacyEncode(value);
    if(value.environment) {
        document["schema"].SetString(EnvironmentSchema,document.GetAllocator());
        array_json::Child(document,"environment_wall",run::EnvironmentDocument(*value.environment));
        rapidjson::StringBuffer bytes;rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
        Require(document.Accept(writer) && bytes.GetSize()<=run::MetadataCap,
            "Environment recovery descriptor exceeds metadata cap");
    }
    return document;
}
Description Decode(const Value& value) {
    if(!value.IsObject() || !value.HasMember("environment_wall"))return LegacyDecode(value);
    using namespace array_json;
    Require(value.HasMember("schema") && !value.HasMember("wall_case") &&
        Text(value["schema"])==EnvironmentSchema,"Unknown recovered environment schema or mixed wall profiles");
    const auto environment=run::ReadEnvironmentDocument(value["environment_wall"]);
    // Schema dispatch reuses the strict v1 field/claim checks in memory only.
    // No original configuration, run manifest or summary is rewritten.
    Document legacy;legacy.CopyFrom(value,legacy.GetAllocator());legacy.RemoveMember("environment_wall");
    legacy["schema"].SetString(Schema,legacy.GetAllocator());
    auto result=LegacyDecode(legacy);result.environment=environment;
    Encode(result);
    return result;
}
} // namespace crash::output::recovered_frames
