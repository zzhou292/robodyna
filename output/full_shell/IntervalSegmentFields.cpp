#include "IntervalSegments.h"
#include "FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"

namespace crash::output::full_shell {
namespace {
Document CursorDocument(const IntervalCursor& p) {
    Document d;d.SetObject();
    Integer(d,"epoch",p.epoch);
    Integer(d,"attempt",p.attempt);
    Integer(d,"first_contact_epoch",p.first_contact);
    Integer(d,"last_contact_epoch",p.last_contact);
    Integer(d,"contact_intervals",p.contact_intervals);
    Number(d,"time_s",p.time);
    Number(d,"velocity_time_s",p.velocity_time);
    Number(d,"base_time_s",p.base_time);
    Number(d,"maximum_plastic_strain",p.maximum_plastic);
    Number(d,"cumulative_plastic_work_J",p.plastic_work);
    Number(d,"wall_potential_J",p.wall_potential);
    return d;
}
IntervalCursor ParseCursor(const Value& v) {
    using namespace array_json;
    Keys(v,{"epoch","attempt","first_contact_epoch","last_contact_epoch","contact_intervals","time_s","velocity_time_s",
        "base_time_s","maximum_plastic_strain","cumulative_plastic_work_J","wall_potential_J"});
    return {UInt(v["epoch"]),UInt(v["attempt"]),UInt(v["first_contact_epoch"]),UInt(v["last_contact_epoch"]),
        UInt(v["contact_intervals"]),Real(v["time_s"]),Real(v["velocity_time_s"]),Real(v["base_time_s"]),
        Real(v["maximum_plastic_strain"]),Real(v["cumulative_plastic_work_J"]),Real(v["wall_potential_J"])};
}
}
Document SegmentDocument(const IntervalContext& c,const IntervalSegment& s) {
    CheckSegment(c,s);
    Document d;d.SetObject();
    String(d,"schema",IntervalSegmentSchema);
    array_json::Child(d,"identity",IdentityDocument(s.identity));
    Number(d,"fixed_dt_s",s.fixed_dt);
    Integer(d,"index",s.index);
    Integer(d,"first_epoch",s.first_epoch);
    Integer(d,"row_count",s.row_count);
    array_json::Child(d,"before",CursorDocument(s.before));
    array_json::Child(d,"after",CursorDocument(s.after));
    const arrays::Limits limits{c.limits.file_bytes,UINT32_MAX,64};
    array_json::Child(d,"integers",arrays::DescriptorDocument(s.integers,limits));
    array_json::Child(d,"reals",arrays::DescriptorDocument(s.reals,limits));
    return d;
}
IntervalSegment ParseSegmentDocument(const IntervalContext& c,const Value& v) {
    using namespace array_json;
    CheckIntervalContext(c);
    Keys(v,{"schema","identity","fixed_dt_s","index","first_epoch","row_count","before","after","integers","reals"});
    Require(Text(v["schema"])==IntervalSegmentSchema,"Unexpected interval segment schema");
    IntervalSegment s;
    s.identity=ParseIdentity(v["identity"]);
    s.fixed_dt=Real(v["fixed_dt_s"]);
    const auto index=UInt(v["index"]);
    Require(index<64,"Interval segment index exceeds capacity");
    s.index=static_cast<std::size_t>(index);
    s.first_epoch=UInt(v["first_epoch"]);
    s.row_count=UInt(v["row_count"]);
    s.before=ParseCursor(v["before"]);
    s.after=ParseCursor(v["after"]);
    const arrays::Limits limits{c.limits.file_bytes,UINT32_MAX,64};
    s.integers=arrays::ParseDescriptor(v["integers"],limits);
    s.reals=arrays::ParseDescriptor(v["reals"],limits);
    CheckSegment(c,s);
    return s;
}
} // namespace crash::output::full_shell
