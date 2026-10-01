#include "Request.h"
#include "output/ArtifactIO.h"
#include <cmath>
#include <initializer_list>
#include <limits>
#include <set>

namespace robodyna::driver {
namespace {
namespace io = crash::output;
using Value = io::Value;
void Fields(const Value& value, std::initializer_list<const char*> names) {
    io::Require(value.IsObject() && value.MemberCount() == names.size(), "Invalid native request object fields");
    std::set<std::string> seen;
    for (const auto& entry : value.GetObject()) {
        io::Require(entry.name.IsString() && seen.emplace(entry.name.GetString(), entry.name.GetStringLength()).second,
                    "Duplicate native request field");
    }
    for (const char* name : names) io::Require(value.HasMember(name), "Missing native request field");
}
std::string Text(const Value& value) {
    io::Require(value.IsString() && value.GetStringLength() && value.GetStringLength() <= 4096,
                "Invalid native request string");
    const std::string text(value.GetString(), value.GetStringLength());
    io::Require(text.find('\0') == std::string::npos, "Embedded NUL in native request string");
    return text;
}
std::size_t Count(const Value& value, std::size_t low = 1, std::size_t high = std::numeric_limits<std::size_t>::max()) {
    io::Require(value.IsUint64() && value.GetUint64() >= low && value.GetUint64() <= high,
                "Invalid native request count");
    return static_cast<std::size_t>(value.GetUint64());
}
double Positive(const Value& value) {
    io::Require(value.IsNumber() && std::isfinite(value.GetDouble()) && value.GetDouble() > 0,
                "Invalid native request positive scalar");
    return value.GetDouble();
}
bool Flag(const Value& value) {
    io::Require(value.IsBool(), "Invalid native request boolean");
    return value.GetBool();
}
std::filesystem::path Path(const Value& value) {
    std::filesystem::path path(Text(value));
    io::Require(path.is_absolute(), "Native request paths must be absolute");
    return path;
}
std::string Digest(const Value& value) {
    const auto digest = Text(value);
    io::Require(digest.size() == 64 && digest.find_first_not_of("0123456789abcdef") == std::string::npos,
                "Invalid source SHA-256");
    return digest;
}
}  // namespace

Request ReadRequest(const std::filesystem::path& path, const std::string& expected_sha256) {
    const auto bytes = io::ReadBounded(path, 1u << 20);
    io::Require(expected_sha256.size() == 64 && io::Sha256(bytes) == expected_sha256,
                "Native request differs from its launch identity");
    io::Document doc;
    doc.Parse(bytes.data(), bytes.size());
    io::Require(!doc.HasParseError(), "Invalid native request JSON");
    Fields(doc, {"schema", "profile", "source_paths", "solid_packets", "run", "resources", "output", "report", "stop_file"});
    io::Require(Text(doc["schema"]) == "robodyna.native_vehicle_request.v1" &&
                Text(doc["profile"]) == "yaris.native_v6.wall_self", "Unsupported native request profile");
    const auto& sources = doc["source_paths"];
    Fields(sources, {"canonical", "scope", "member", "declarations", "glass_resolution", "type13",
                     "auxiliary_member", "original_wall_member", "wall_manifest", "self_contact_combine_member"});
    Request request;
    for (const auto& entry : sources.GetObject()) request.sources.emplace(entry.name.GetString(), Path(entry.value));
    const auto& packet = doc["solid_packets"];
    Fields(packet, {"path", "bytes", "sha256"});
    request.packets = {Path(packet["path"]), Count(packet["bytes"], 1, 16u << 20), Digest(packet["sha256"])};
    const auto& run = doc["run"];
    Fields(run, {"duration_s", "fixed_dt_s", "samples", "contact_activity", "stage_timing",
                 "capture_qeph_rejection", "verify_initial_retry"});
    request.run = {Positive(run["duration_s"]), Positive(run["fixed_dt_s"]), Count(run["samples"], 2, 1000),
                   Text(run["contact_activity"]), Flag(run["stage_timing"]), Flag(run["capture_qeph_rejection"]),
                   Flag(run["verify_initial_retry"])};
    // Exact interval admission belongs to the existing fixed-step planner;
    // do not approximate its boundary with a floating-point division here.
    io::Require(request.run.duration_s <= .1 &&
                (request.run.contact_activity == "shell_removal" || request.run.contact_activity == "all_active_prefix"),
                "Native run profile/horizon exceeds supported scope");
    const auto& resources = doc["resources"];
    Fields(resources, {"schema", "cpu_threads", "rss_bytes", "minimum_available_ram_bytes", "gpu_index",
                       "minimum_gpu_free_bytes", "maximum_gpu_growth_bytes", "timeout_s",
                       "cooperative_maximum_elapsed_s", "stop_grace_s", "archive_bytes", "artifact_file_bytes",
                       "solid_worker_blocks", "workstation_lock"});
    io::Require(Text(resources["schema"]) == "robodyna.resources.v1", "Unsupported resource profile");
    for (const char* name : {"cpu_threads", "minimum_available_ram_bytes", "minimum_gpu_free_bytes", "maximum_gpu_growth_bytes"})
        (void)Count(resources[name]);
    (void)Count(resources["gpu_index"], 0);
    (void)Positive(resources["stop_grace_s"]);
    (void)Path(resources["workstation_lock"]);
    request.resources = {Count(resources["rss_bytes"], (2u << 20) + 1),
                         Count(resources["archive_bytes"], 1, std::size_t{6} << 30),
                         Count(resources["artifact_file_bytes"], 1, 32u << 20),
                         Count(resources["solid_worker_blocks"]), Positive(resources["cooperative_maximum_elapsed_s"])};
    const auto workers = request.resources.solid_worker_blocks;
    io::Require((workers == 4 || workers == 8 || workers == 16 || workers == 32) &&
                request.resources.cooperative_maximum_elapsed_s < Positive(resources["timeout_s"]),
                "Invalid native execution resource request");
    request.output = Path(doc["output"]);
    request.report = Path(doc["report"]);
    request.stop_file = Path(doc["stop_file"]);
    io::Require(request.output.filename() == "accepted" && request.report.filename() == "native-report.json" &&
                request.stop_file.filename() == "stop.requested" &&
                request.output.parent_path() == request.report.parent_path() &&
                request.output.parent_path() == request.stop_file.parent_path(), "Native output locations differ from one launch");
    return request;
}
}  // namespace robodyna::driver
