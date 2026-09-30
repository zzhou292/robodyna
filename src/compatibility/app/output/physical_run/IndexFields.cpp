#include "Metadata.h"
#include "output/full_shell/FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
Document IndexDocument(const Configuration& c,const Index& index) {
    Document d;d.SetObject();String(d,"schema",IndexSchema);
    array_json::Child(d,"identity",records::IdentityDocument(c.identity));
    Integer(d,"planned_intervals",index.planned_intervals);Integer(d,"accepted_intervals",index.accepted_intervals);
    Boolean(d,"horizon_complete",index.horizon_complete);String(d,"stop_reason",index.stop_reason);
    array_json::Child(d,"final",StampDocument(index.final));
    Value segments(rapidjson::kArrayType),frames(rapidjson::kArrayType);
    const arrays::Limits limits{c.request.file_byte_cap,UINT32_MAX,64};
    for(const auto& s:index.segments) {
        Document row;row.SetObject();Integer(row,"first_epoch",s.first_epoch);Integer(row,"rows",s.rows);
        array_json::Child(row,"integers",arrays::DescriptorDocument(s.integers,limits));
        array_json::Child(row,"reals",arrays::DescriptorDocument(s.reals,limits));
        Value value;value.CopyFrom(row,d.GetAllocator());segments.PushBack(value,d.GetAllocator());
    }
    for(const auto& f:index.frames) {
        Document row;row.SetObject();array_json::Child(row,"stamp",StampDocument(f.stamp));
        array_json::Child(row,"frame",FileDocument(f.frame));array_json::Child(row,"activity",FileDocument(f.activity));
        Value value;value.CopyFrom(row,d.GetAllocator());frames.PushBack(value,d.GetAllocator());
    }
    d.AddMember("segments",segments,d.GetAllocator());d.AddMember("frames",frames,d.GetAllocator());return d;
}
Index ReadIndex(const records::Context& c,const Configuration& config,const Value& v) {
    using namespace array_json;
    Keys(v,{"schema","identity","planned_intervals","accepted_intervals","horizon_complete","stop_reason","final","segments","frames"});
    Require(Text(v["schema"])==IndexSchema && records::SameIdentity(records::ParseIdentity(v["identity"]),config.identity) &&
        v["horizon_complete"].IsBool(),"Physical frame index identity differs");
    Index index;
    index.planned_intervals=UInt(v["planned_intervals"]);index.accepted_intervals=UInt(v["accepted_intervals"]);
    index.horizon_complete=v["horizon_complete"].GetBool();index.stop_reason=Text(v["stop_reason"]);index.final=ReadStamp(v["final"]);
    Require(v["segments"].IsArray() && v["segments"].Size()<=64 && v["frames"].IsArray() &&
        v["frames"].Size()<=kArtifactFrameCap,"Physical index collection exceeds cap");
    const arrays::Limits limits{config.request.file_byte_cap,UINT32_MAX,64};
    for(const auto& row:v["segments"].GetArray()) {
        Keys(row,{"first_epoch","rows","integers","reals"});
        index.segments.push_back({UInt(row["first_epoch"]),UInt(row["rows"]),arrays::ParseDescriptor(row["integers"],limits),
            arrays::ParseDescriptor(row["reals"],limits)});
    }
    for(const auto& row:v["frames"].GetArray()) {
        Keys(row,{"stamp","frame","activity"});
        index.frames.push_back({ReadStamp(row["stamp"]),ReadFileRecord(row["frame"]),ReadFileRecord(row["activity"])});
    }
    CheckIndex(c,config,index);return index;
}
} // namespace crash::output::physical_run
