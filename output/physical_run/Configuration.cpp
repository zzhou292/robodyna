#include "Metadata.h"
#include "output/full_shell/FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
Document ConfigurationDocument(const Configuration& c) {
    const auto& r=c.request;
    records::CheckIdentity(c.identity);arrays::CheckHash(c.point_layout_sha256);
    Require(r.extra_interval_bytes==ExtraIntervalBytes(c.profile) && !r.extra_frame_bytes,
        "Physical configuration interval storage differs from its profile");
    records::PlanArchive(r);
    Document d;d.SetObject();String(d,"schema",c.profile.native_contact?"robo_dyna.physical_run_configuration.v3":c.profile.self_contact?
        "robo_dyna.physical_run_configuration.v2":"robo_dyna.physical_run_configuration.v1");
    if(c.wall)String(d,"wall_case",WallProfile);
    array_json::Child(d,"identity",records::IdentityDocument(c.identity));
    array_json::Child(d,"profile",ProfileDocument(c.profile));
    String(d,"point_layout_sha256",c.point_layout_sha256);
    Integer(d,"nodes",r.nodes);Integer(d,"parents",r.parents);Integer(d,"points",r.plastic_points);
    Integer(d,"samples",r.frames);Integer(d,"intervals",r.intervals);
    Number(d,"fixed_dt_s",r.fixed_dt);Number(d,"requested_duration_s",r.requested_duration);
    Integer(d,"static_reserve_bytes",r.static_byte_reserve);
    Integer(d,"total_byte_cap",r.total_byte_cap);Integer(d,"file_byte_cap",r.file_byte_cap);
    if(c.profile.self_contact||c.profile.native_contact)Integer(d,"extra_interval_bytes",r.extra_interval_bytes);
    Value files(rapidjson::kArrayType);
    for(const auto& f:r.static_files) {
        Document row;row.SetObject();String(row,"file",f.file);Integer(row,"bytes",f.bytes);
        Value value;value.CopyFrom(row,d.GetAllocator());files.PushBack(value,d.GetAllocator());
    }
    d.AddMember("static_reservations",files,d.GetAllocator());return d;
}
Configuration ReadConfiguration(const Value& v) {
    using namespace array_json;
    const bool wall=v.IsObject() && v.HasMember("wall_case");
    const bool self=v.IsObject() && v.HasMember("extra_interval_bytes");
    if(wall && self)Keys(v,{"schema","identity","profile","point_layout_sha256","nodes","parents","points","samples","intervals",
        "fixed_dt_s","requested_duration_s","static_reserve_bytes","total_byte_cap","file_byte_cap","static_reservations","wall_case","extra_interval_bytes"});
    else if(self)Keys(v,{"schema","identity","profile","point_layout_sha256","nodes","parents","points","samples","intervals",
        "fixed_dt_s","requested_duration_s","static_reserve_bytes","total_byte_cap","file_byte_cap","static_reservations","extra_interval_bytes"});
    else if(wall)Keys(v,{"schema","identity","profile","point_layout_sha256","nodes","parents","points","samples","intervals",
        "fixed_dt_s","requested_duration_s","static_reserve_bytes","total_byte_cap","file_byte_cap","static_reservations","wall_case"});
    else Keys(v,{"schema","identity","profile","point_layout_sha256","nodes","parents","points","samples","intervals",
        "fixed_dt_s","requested_duration_s","static_reserve_bytes","total_byte_cap","file_byte_cap","static_reservations"});
    Configuration c;auto& r=c.request;
    if(wall)Require(Text(v["wall_case"])==WallProfile,"Unknown physical wall profile");
    c.wall=wall;
    c.identity=records::ParseIdentity(v["identity"]);c.profile=ReadProfile(v["profile"]);
    Require((c.profile.self_contact||c.profile.native_contact)==self,"Physical configuration/profile contact storage differs");
    Require(Text(v["schema"])==(c.profile.native_contact?"robo_dyna.physical_run_configuration.v3":
        c.profile.self_contact?"robo_dyna.physical_run_configuration.v2":"robo_dyna.physical_run_configuration.v1"),
        "Unsupported physical configuration");
    if(self)r.extra_interval_bytes=UInt(v["extra_interval_bytes"]);
    c.point_layout_sha256=Text(v["point_layout_sha256"]);
    r.nodes=UInt(v["nodes"]);r.parents=UInt(v["parents"]);r.plastic_points=UInt(v["points"]);
    r.frames=UInt(v["samples"]);r.intervals=UInt(v["intervals"]);
    r.fixed_dt=Real(v["fixed_dt_s"]);r.requested_duration=Real(v["requested_duration_s"]);
    r.static_byte_reserve=UInt(v["static_reserve_bytes"]);r.total_byte_cap=UInt(v["total_byte_cap"]);
    r.file_byte_cap=UInt(v["file_byte_cap"]);
    const auto& rows=v["static_reservations"];
    Require(rows.IsArray() && rows.Size()<=kArtifactInventoryCap,"Physical static reservation count exceeds cap");
    for(const auto& row:rows.GetArray()) {
        Keys(row,{"file","bytes"});r.static_files.push_back({Text(row["file"]),static_cast<std::size_t>(UInt(row["bytes"]))});
    }
    ConfigurationDocument(c);return c;
}
} // namespace crash::output::physical_run
