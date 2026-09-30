#include "Internal.h"
#include "output/full_shell/FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"

namespace crash::output::full_shell::activity::detail {
namespace {
void Common(Document& d,const Context& c) {
    String(d,"activity_scope","parent_mechanical_activity");String(d,"encoding","uint64_lsb0");
    String(d,"padding","unused_high_bits_zero");String(d,"phase","accepted_endpoint");
    array_json::Child(d,"identity",IdentityDocument(c.identity()));
    String(d,"point_layout_sha256",c.point_layout_sha256());Number(d,"fixed_dt_s",c.fixed_dt());
    Integer(d,"nodes",c.nodes());Integer(d,"parents",c.parents().size());Integer(d,"points",c.points());
}
void CheckCommon(const Context& c,const Value& v) {
    using namespace array_json;
    Require(Text(v["activity_scope"])=="parent_mechanical_activity"&&Text(v["encoding"])=="uint64_lsb0"&&
        Text(v["padding"])=="unused_high_bits_zero"&&Text(v["phase"])=="accepted_endpoint", "Unexpected activity encoding or phase");
    Require(SameIdentity(c.identity(),ParseIdentity(v["identity"]))&&
        Text(v["point_layout_sha256"])==c.point_layout_sha256()&&Bits(Real(v["fixed_dt_s"]))==Bits(c.fixed_dt())&&
        UInt(v["nodes"])==c.nodes()&&UInt(v["parents"])==c.parents().size()&&UInt(v["points"])==c.points(),
        "Activity source/owner/layout identity mismatch");
}
}
Document Declaration(const Context& c) {
    Document d;d.SetObject();String(d,"schema",DeclarationSchema);Common(d,c);
    Integer(d,"inactive_value",0);Integer(d,"active_value",1);return d;
}
void CheckDeclaration(const Context& c,const Value& v) {
    using namespace array_json;
    Keys(v,{"schema","activity_scope","encoding","padding","phase","identity","point_layout_sha256","fixed_dt_s",
        "nodes","parents","points","inactive_value","active_value"});
    Require(Text(v["schema"])==DeclarationSchema&&UInt(v["inactive_value"])==0&&UInt(v["active_value"])==1,
        "Unexpected activity declaration");CheckCommon(c,v);
}
Document Frame(const ActivityRecord& r,const arrays::Descriptor& array) {
    Document d;d.SetObject();String(d,"schema",Schema);Common(d,r.context());
    const auto& s=r.stamp();Integer(d,"epoch",s.epoch);Integer(d,"base_epoch",s.base_epoch);Integer(d,"attempt",s.attempt);
    Number(d,"time_s",s.time);Number(d,"base_time_s",s.base_time);Number(d,"velocity_time_s",s.velocity_time);Number(d,"kick_dt_s",s.kick_dt);
    array_json::Child(d,"activity",arrays::DescriptorDocument(array,r.context().limits().arrays));return d;
}
arrays::Descriptor ParseFrame(const Context& c,const Value& v,const FrameStamp& expected) {
    using namespace array_json;
    Keys(v,{"schema","activity_scope","encoding","padding","phase","identity","point_layout_sha256","fixed_dt_s","nodes",
        "parents","points","epoch","base_epoch","attempt","time_s","base_time_s","velocity_time_s","kick_dt_s","activity"});
    Require(Text(v["schema"])==Schema,"Unexpected activity frame schema");CheckCommon(c,v);
    FrameStamp s{UInt(v["epoch"]),UInt(v["base_epoch"]),UInt(v["attempt"]),Real(v["time_s"]),Real(v["base_time_s"]),
        Real(v["velocity_time_s"]),Real(v["kick_dt_s"])};
    CheckStamp(c,s);Require(SameStamp(s,expected),"Activity does not match expected accepted phase");
    auto d=arrays::ParseDescriptor(v["activity"],c.limits().arrays);const auto wanted=Layout(c.parents().size());
    Require(d.layout.scalar==wanted.scalar&&d.layout.rows==wanted.rows&&d.layout.columns==wanted.columns&&d.layout.fields==wanted.fields,
        "Activity array does not match original parent layout");return d;
}
} // namespace crash::output::full_shell::activity::detail
