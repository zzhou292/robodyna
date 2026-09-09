#include "AcceptedReplayData.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

namespace crash::output::replay_detail {
namespace {
void UniqueMembers(const Value& value, unsigned depth = 0) {
    Require(depth <= 16, "Replay JSON nesting exceeds the preview cap");
    if (value.IsObject()) {
        std::set<std::string> names;
        for (const auto& member : value.GetObject()) {
            Require(names.emplace(member.name.GetString(), member.name.GetStringLength()).second,
                    "Duplicate replay JSON member");
            UniqueMembers(member.value, depth + 1);
        }
    } else if (value.IsArray()) {
        for (const auto& child : value.GetArray()) UniqueMembers(child, depth + 1);
    }
}
bool Basename(const std::string& name) {
    if (name.empty() || name.size() > 255 || name == "." || name == "..") return false;
    for (unsigned char c : name)
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
              c == '-' || c == '_' || c == '.')) return false;
    return true;
}
template <class T> T CsvNumber(const std::string& text) {
    T result{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
    Require(parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size(), "Invalid replay CSV number");
    return result;
}
void CheckTime(double actual, double expected, double dt, std::uint64_t epoch) {
    // Repeated accepted t += h accumulates rounding; archived endpoints need
    // not equal epoch*h bitwise. Bound that accumulation, not an arbitrary
    // display tolerance. Refuse epochs for which this budget is uninformative.
    const double roundoff = (static_cast<double>(epoch)+1)*std::numeric_limits<double>::epsilon();
    const double budget = 8*roundoff*std::max({std::abs(actual),std::abs(expected),dt});
    Require(roundoff < 1e-3 && std::isfinite(expected) && std::isfinite(budget) &&
            std::abs(actual-expected) <= budget, "Replay accepted time disagrees with configured fixed step/horizon");
}
void ReadInventory(Bundle& bundle, const Document& manifest) {
    const auto& inventory = Member(manifest, "artifacts");
    Require(inventory.IsArray() && inventory.Size() && inventory.Size() <= 3*kFrameCap + 16,
            "Invalid replay inventory size");
    std::size_t total = 0;
    for (const auto& item : inventory.GetArray()) {
        const auto name = Text(item, "file"), hash = Text(item, "sha256");
        const auto bytes = Unsigned(item, "bytes");
        Require(Basename(name) && name != "manifest.json" && name != "manifest.pending.json" && name != "failure.json",
                "Unsafe replay inventory filename");
        Require(hash.size() == 64 && hash.find_first_not_of("0123456789abcdef") == std::string::npos,
                "Invalid replay SHA256");
        Require(bytes <= kFileCap && bytes <= kTotalCap-total, "Replay inventory exceeds byte caps");
        total += static_cast<std::size_t>(bytes);
        Require(bundle.inventory.emplace(name, Artifact{hash, static_cast<std::size_t>(bytes)}).second,
                "Duplicate replay inventory filename");
    }
    for (const auto& artifact : bundle.inventory) VerifiedBytes(bundle, artifact.first);
}
void ReadFrames(Bundle& bundle) {
    std::istringstream input(VerifiedBytes(bundle, "accepted-frames.csv"));
    std::string line;
    Require(bool(std::getline(input, line)) &&
        line == "owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization", "Unsupported accepted frame index");
    std::set<std::string> meshes;
    while (std::getline(input, line)) {
        Require(bundle.entries.size() < kFrameCap, "Replay frame cap exceeded");
        std::array<std::string,5> columns;
        std::istringstream row(line);
        for (auto& column : columns) Require(bool(std::getline(row, column, ',')) && !column.empty(), "Incomplete frame row");
        Require(row.eof(), "Extra frame index column");
        Entry entry{CsvNumber<std::uint64_t>(columns[0]), CsvNumber<std::uint64_t>(columns[1]),
                    CsvNumber<double>(columns[2]), columns[3], columns[4]};
        Require(entry.owner && std::isfinite(entry.time) && entry.time >= 0, "Invalid accepted owner/time");
        std::ostringstream stem; stem << "accepted-" << std::setw(6) << std::setfill('0') << entry.epoch;
        Require(entry.mesh == stem.str()+".mesh.json" && entry.obj == stem.str()+".obj" &&
                bundle.inventory.count(entry.mesh) && bundle.inventory.count(entry.obj) && meshes.insert(entry.mesh).second,
                "Frame filenames are missing, duplicate or do not identify their epoch");
        if (bundle.entries.empty()) {
            Require(entry.epoch == 0 && entry.time == 0, "Replay must include the initial accepted frame");
            bundle.info.owner_id = entry.owner;
        } else {
            const auto& previous = bundle.entries.back();
            Require(entry.owner == previous.owner && entry.epoch > previous.epoch && entry.time > previous.time,
                    "Replay owner or frame order changed");
        }
        bundle.entries.push_back(std::move(entry));
    }
    Require(!bundle.entries.empty() && bundle.entries.back().epoch == bundle.info.final_epoch &&
            Bits(bundle.entries.back().time) == Bits(bundle.info.final_time), "Replay final accepted frame is incomplete");
    bundle.info.frame_count = bundle.entries.size();
}
}  // namespace

Document Json(const std::string& bytes) {
    Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag>(bytes.data(), bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Invalid replay JSON");
    UniqueMembers(document);
    return document;
}
const Value& Member(const Value& value, const char* name) {
    Require(value.IsObject() && value.HasMember(name), "Required replay JSON member is missing");
    return value[name];
}
std::uint64_t Unsigned(const Value& value, const char* name) {
    const auto& field = Member(value, name);
    Require(field.IsUint64(), "Replay integer has invalid type");
    return field.GetUint64();
}
double Real(const Value& value, const char* name) {
    const auto& field = Member(value, name);
    Require(field.IsNumber() && std::isfinite(field.GetDouble()), "Replay scalar is nonfinite or has invalid type");
    return field.GetDouble();
}
std::string Text(const Value& value, const char* name) {
    const auto& field = Member(value, name);
    Require(field.IsString(), "Replay string has invalid type");
    return {field.GetString(), field.GetStringLength()};
}
std::string VerifiedBytes(const Bundle& bundle, const std::string& name) {
    const auto found = bundle.inventory.find(name);
    Require(found != bundle.inventory.end(), "Required replay artifact is not inventoried");
    const auto path = bundle.directory/name;
    Require(std::filesystem::symlink_status(path).type() == std::filesystem::file_type::regular,
            "Replay artifact must be a regular file, not a symlink");
    const auto bytes = ReadBounded(path, kFileCap);
    Require(bytes.size() == found->second.bytes && Sha256(bytes) == found->second.hash, "Replay artifact size/hash mismatch");
    return bytes;
}

Bundle ReadIndex(const std::filesystem::path& directory) {
    Require(!std::filesystem::exists(directory/"failure.json") && !std::filesystem::exists(directory/"manifest.pending.json"),
            "Replay bundle reports incomplete output");
    Require(std::filesystem::symlink_status(directory/"manifest.json").type() == std::filesystem::file_type::regular,
            "Replay requires a regular completed manifest");
    const auto manifest = Json(ReadBounded(directory/"manifest.json", 1024*1024));
    Require(Text(manifest, "status") == "completed", "Replay manifest is not completed");
    Bundle bundle; bundle.directory = directory;
    bundle.info.schema = Text(manifest, "schema");
    const bool coupon = bundle.info.schema == "robo_dyna.elastic_coupon_artifacts.v1";
    Require(coupon || bundle.info.schema == "tlfea.normal_impact_artifacts.v1", "Unsupported replay artifact schema");
    bundle.info.kind = coupon ? ReplayKind::ElasticCoupon : ReplayKind::NormalImpact;
    const auto& shell_model = Member(manifest, "shell_model"); const auto& vehicle_model = Member(manifest, "vehicle_model");
    Require(shell_model.IsBool() && shell_model.GetBool() == coupon && vehicle_model.IsBool() && !vehicle_model.GetBool(),
            "Replay schema/model scope flags disagree");
    bundle.info.scope = coupon ? "Synthetic elastic shell coupon" : "Translational mass patch against the canonical wall";
    bundle.info.final_epoch = Unsigned(manifest, "accepted_epoch");
    bundle.info.final_time = Real(manifest, "accepted_time_s");
    Require(bundle.info.final_epoch && bundle.info.final_time > 0, "Replay completed horizon is invalid");
    ReadInventory(bundle, manifest); ReadFrames(bundle);
    Require(bundle.inventory.count("accepted-intervals.csv"), "Completed replay lacks its interval record");
    const auto final = Json(VerifiedBytes(bundle, "final-metrics.json"));
    Require(Unsigned(final, "accepted_epoch") == bundle.info.final_epoch &&
            Bits(Real(final, "accepted_time_s")) == Bits(bundle.info.final_time), "Final metrics disagree with replay manifest");
    const auto configuration = Json(VerifiedBytes(bundle, "configuration.json"));
    const double dt = Real(configuration,coupon ? "fixed_dt_s" : "dt_s");
    const double horizon = Real(configuration,coupon ? "half_period_horizon_s" : "requested_horizon_s");
    Require(dt > 0 && horizon > 0, "Replay configured step/horizon must be positive");
    for (const auto& entry : bundle.entries) CheckTime(entry.time,entry.epoch*dt,dt,entry.epoch);
    CheckTime(bundle.info.final_time,horizon,dt,bundle.info.final_epoch);
    if (coupon) {
        Require(Text(configuration, "schema") == "robo_dyna.elastic_coupon_configuration.v1" &&
                Unsigned(configuration, "owner_id") == bundle.info.owner_id &&
                Unsigned(configuration, "required_steps") == bundle.info.final_epoch &&
                Unsigned(final, "saved_frames") == bundle.info.frame_count, "Coupon replay configuration association mismatch");
        bundle.info.run_id = Unsigned(configuration, "run_id"); bundle.info.topology_id = Unsigned(configuration, "topology_id");
        Require(bundle.info.run_id && bundle.info.topology_id && !bundle.inventory.count("canonical-wall.mesh.json"),
                "Coupon replay has invalid source IDs or an undeclared wall");
        const auto& vertices = Member(configuration, "vertex_binding");
        const auto& triangles = Member(configuration, "triangle_binding");
        Require(vertices.IsArray() && vertices.Size() && vertices.Size() <= kVertexCap &&
                triangles.IsArray() && triangles.Size() && triangles.Size() <= kTriangleCap, "Invalid coupon mesh binding size");
        bundle.info.node_count = vertices.Size();
        for (unsigned i = 0; i < vertices.Size(); ++i) {
            const auto& row = vertices[i];
            Require(row.IsArray() && row.Size() == 4, "Invalid coupon vertex binding");
            for (const auto& number : row.GetArray()) Require(number.IsUint64(), "Invalid coupon source identifier");
            Require(row[0].GetUint64() == i && row[1].GetUint64() && row[2].GetUint64() && row[3].GetUint64(),
                    "Unsupported coupon vertex mapping or absent source identifiers");
        }
        for (const auto& row : triangles.GetArray()) {
            Require(row.IsArray() && row.Size() == 9, "Invalid coupon triangle binding");
            for (const auto& number : row.GetArray()) Require(number.IsUint64(), "Invalid coupon triangle identifier");
            Require(row[3].GetUint64() && row[4].GetUint64() && row[5].GetUint64() && row[6].GetUint64(),
                    "Missing coupon triangle source identifiers");
            std::array<int,3> face{};
            for (unsigned axis = 0; axis < 3; ++axis) {
                Require(row[axis].GetUint64() < vertices.Size(), "Coupon binding index is out of range");
                face[axis] = static_cast<int>(row[axis].GetUint64());
            }
            bundle.topology.push_back(face);
        }
    } else {
        Require(Text(configuration, "schema") == "tlfea.normal_impact_configuration.v1" &&
                Unsigned(final, "owner_id") == bundle.info.owner_id && bundle.inventory.count("canonical-wall.mesh.json"),
                "Normal-impact replay owner association or canonical wall is missing");
        bundle.info.node_count = Unsigned(final, "node_count");
        Require(bundle.info.node_count >= 3 && bundle.info.node_count <= kVertexCap, "Invalid normal-impact node count");
    }
    return bundle;
}
}  // namespace crash::output::replay_detail
