#include "FullShellVisualizationRecords.h"
#include "FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"
#include <cmath>

namespace crash::output::full_shell {
namespace {
void CheckDescription(const Context& c, const FrameDescription& f) {
    CheckStamp(c, f.stamp);
    Require(SameIdentity(c.identity(), f.identity) && Bits(c.fixed_dt()) == Bits(f.fixed_dt) &&
        f.nodes == c.nodes() && f.parents == c.parents().size() && f.points == c.points() &&
        f.point_layout_sha256 == c.point_layout_sha256(), "Frame source/owner/layout identity mismatch");
    arrays::CheckDescriptor(f.positions, c.limits().arrays);
    arrays::CheckDescriptor(f.plastic, c.limits().arrays);
    Require(f.positions.layout.scalar == arrays::Scalar::Float64 && f.positions.layout.rows == c.nodes() &&
        f.positions.layout.columns == 3 && f.positions.layout.fields == std::vector<std::string>{"x_m", "y_m", "z_m"} &&
        f.plastic.layout.scalar == arrays::Scalar::Float64 && f.plastic.layout.rows == c.points() &&
        f.plastic.layout.columns == 1 &&
        f.plastic.layout.fields == std::vector<std::string>{"native_equivalent_plastic_strain"} &&
        f.positions.file != f.plastic.file, "Frame array type/source mapping mismatch");
}
} // namespace

Document FrameDocument(const Context& c, const FrameDescription& f) {
    CheckDescription(c, f);
    Document d;
    d.SetObject();
    String(d, "schema", FrameSchema);
    String(d, "purpose", "accepted_visualization_not_restart");
    String(d, "position_phase", "accepted_endpoint");
    String(d, "plastic_phase", "accepted_native_history");
    String(d, "velocity_phase", f.stamp.epoch ? "previous_midpoint" : "collocated");
    array_json::Child(d, "identity", IdentityDocument(f.identity));
    Number(d, "fixed_dt_s", f.fixed_dt);
    Integer(d, "epoch", f.stamp.epoch);
    Integer(d, "base_epoch", f.stamp.base_epoch);
    Integer(d, "attempt", f.stamp.attempt);
    Number(d, "time_s", f.stamp.time);
    Number(d, "base_time_s", f.stamp.base_time);
    Number(d, "velocity_time_s", f.stamp.velocity_time);
    Number(d, "kick_dt_s", f.stamp.kick_dt);
    Integer(d, "nodes", f.nodes);
    Integer(d, "parents", f.parents);
    Integer(d, "points", f.points);
    String(d, "point_layout_sha256", f.point_layout_sha256);
    array_json::Child(d, "positions", arrays::DescriptorDocument(f.positions, c.limits().arrays));
    array_json::Child(d, "plastic", arrays::DescriptorDocument(f.plastic, c.limits().arrays));
    return d;
}

FrameDescription ParseFrameDocument(const Context& c, const Value& v) {
    using namespace array_json;
    Keys(v,{"schema","purpose","position_phase","plastic_phase","velocity_phase","identity","fixed_dt_s","epoch","base_epoch",
        "attempt","time_s","base_time_s","velocity_time_s","kick_dt_s","nodes","parents","points","point_layout_sha256","positions","plastic"});
    Require(Text(v["schema"])==FrameSchema&&Text(v["purpose"])=="accepted_visualization_not_restart"&&
        Text(v["position_phase"])=="accepted_endpoint"&&Text(v["plastic_phase"])=="accepted_native_history",
        "Unexpected frame schema/field phase");
    FrameDescription f;
    f.identity = ParseIdentity(v["identity"]);
    f.fixed_dt = Real(v["fixed_dt_s"]);
    f.stamp = {UInt(v["epoch"]), UInt(v["base_epoch"]), UInt(v["attempt"]), Real(v["time_s"]), Real(v["base_time_s"]),
        Real(v["velocity_time_s"]), Real(v["kick_dt_s"])};
    Require(Text(v["velocity_phase"]) == (f.stamp.epoch ? "previous_midpoint" : "collocated"),
        "Unexpected frame velocity phase");
    const auto nodes = UInt(v["nodes"]), parents = UInt(v["parents"]), points = UInt(v["points"]);
    Require(nodes == c.nodes() && parents == c.parents().size() && points == c.points(), "Frame count mismatch");
    f.nodes = static_cast<std::size_t>(nodes);
    f.parents = static_cast<std::size_t>(parents);
    f.points = static_cast<std::size_t>(points);
    f.point_layout_sha256 = Text(v["point_layout_sha256"]);
    f.positions = arrays::ParseDescriptor(v["positions"], c.limits().arrays);
    f.plastic = arrays::ParseDescriptor(v["plastic"], c.limits().arrays);
    CheckDescription(c, f);
    return f;
}
} // namespace crash::output::full_shell
