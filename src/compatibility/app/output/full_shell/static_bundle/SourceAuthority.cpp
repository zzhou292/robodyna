#include "SourceAuthority.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::full_shell::source::detail {
using namespace array_json;
Document AuthorityDocument(const SourceInputs& in) {
    Document d; d.SetObject();
    String(d, "canonical_manifest_sha256", in.canonical_manifest.sha256);
    Integer(d, "canonical_manifest_bytes", in.canonical_manifest.bytes);
    String(d, "scope_report_sha256", in.scope_report.sha256);
    Integer(d, "scope_report_bytes", in.scope_report.bytes);
    String(d, "source_member_sha256", in.source_member.sha256);
    Integer(d, "source_member_bytes", in.source_member.bytes);
    String(d, "tire_policy", in.tire_policy);
    String(d, "source_mass_unit", in.units.mass);
    String(d, "source_length_unit", in.units.length);
    String(d, "source_time_unit", in.units.time);
    Number(d, "mass_to_kg", in.units.mass_to_kg);
    Number(d, "length_to_m", in.units.length_to_m);
    Number(d, "time_to_s", in.units.time_to_s);
    return d;
}
void CheckAuthority(const SourceInputs& in, const Value& v) {
    Keys(v, {"canonical_manifest_sha256", "canonical_manifest_bytes", "scope_report_sha256",
        "scope_report_bytes", "source_member_sha256", "source_member_bytes", "tire_policy",
        "source_mass_unit", "source_length_unit", "source_time_unit", "mass_to_kg", "length_to_m", "time_to_s"});
    const Units units{Text(v["source_mass_unit"]), Text(v["source_length_unit"]), Text(v["source_time_unit"]),
        Real(v["mass_to_kg"]), Real(v["length_to_m"]), Real(v["time_to_s"])};
    Require(Text(v["canonical_manifest_sha256"]) == in.canonical_manifest.sha256 &&
        UInt(v["canonical_manifest_bytes"]) == in.canonical_manifest.bytes &&
        Text(v["scope_report_sha256"]) == in.scope_report.sha256 && UInt(v["scope_report_bytes"]) == in.scope_report.bytes &&
        Text(v["source_member_sha256"]) == in.source_member.sha256 && UInt(v["source_member_bytes"]) == in.source_member.bytes &&
        Text(v["tire_policy"]) == in.tire_policy && SameUnits(units, in.units),
        "Mapping record does not match caller source authority");
}
} // namespace crash::output::full_shell::source::detail
