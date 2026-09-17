#include "RunAnalysis.h"

#include "ParentCatalog.h"
#include "output/BoundedArrayIO.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::analysis::impact_response {
namespace {

struct AuthenticatedFile {
    output::full_shell::RecordFile record;
    std::string bytes;
};

AuthenticatedFile ReadAuthenticated(const std::filesystem::path& input,
    const std::string& expected_sha256, std::size_t cap) {
    output::arrays::CheckHash(expected_sha256);
    const auto path = std::filesystem::absolute(input);
    output::Require(std::filesystem::symlink_status(path).type() ==
            std::filesystem::file_type::regular &&
            std::filesystem::file_size(path) > 0 &&
            std::filesystem::file_size(path) <= cap,
        "Impact analysis input is missing, symbolic, empty, or exceeds its cap");
    auto bytes = output::ReadBounded(path, cap);
    output::Require(output::Sha256(bytes) == expected_sha256,
        "Impact analysis input differs from its explicit external hash");
    return {{path.filename().string(), expected_sha256, bytes.size()},
        std::move(bytes)};
}

bool IsWithin(const std::filesystem::path& parent,
    const std::filesystem::path& child) {
    auto a = parent.begin(), b = child.begin();
    while (a != parent.end() && b != child.end() && *a == *b) {
        ++a;
        ++b;
    }
    return a == parent.end();
}

}  // namespace

output::Document Analyze(const AnalysisInput& input, AnalysisLimits limits) {
    const auto viewer_path = std::filesystem::absolute(input.viewer_input);
    auto viewer_file = ReadAuthenticated(viewer_path,
        input.viewer_input_sha256, output::physical_run::ViewerInputByteCap);
    const auto viewer = output::physical_run::ReadViewerInput(
        viewer_path.parent_path(), viewer_file.record);
    const auto archive = output::physical_run::ViewerArchivePath(
        viewer_path.parent_path(), viewer);
    const auto replay = output::physical_run::Replay::Open(archive,
        viewer.manifest, viewer.source, viewer.mapping_sha256);

    PlasticityAccumulator plasticity(replay.context(),
        BuildParentCatalog(replay.mapping(), replay.context()), limits);
    for (std::size_t sample = 0; sample < replay.index().frames.size(); ++sample)
        plasticity.Observe(replay.ReadSample(sample));
    auto result = plasticity.Finish();
    output::Require(result.samples.size() == replay.index().frames.size() &&
            result.samples.back().stamp.epoch == replay.index().final.epoch &&
            output::Bits(result.samples.back().stamp.time_s) ==
                output::Bits(replay.index().final.time),
        "Impact analysis did not consume the complete authenticated sample index");

    auto summary_file = ReadAuthenticated(input.run_summary,
        input.run_summary_sha256, output::physical_run::MetadataCap);
    auto connectivity_file = ReadAuthenticated(input.connectivity_report,
        input.connectivity_report_sha256, 32u << 20);
    const auto metadata = MakeRunMetadata(replay, viewer,
        viewer_file.record, summary_file.record, connectivity_file.record);
    const auto summary = VerifyRunSummary(summary_file.bytes, metadata, result);
    const auto& source = replay.mapping().source().data().inputs;
    const ConnectivitySourceAuthority connectivity_authority{
        source.canonical_manifest.sha256, source.source_member.sha256,
        source.tire_policy};
    const auto connectivity = AnalyzeConnectivity(connectivity_file.bytes,
        result, connectivity_authority, limits);
    return BuildReport(metadata, summary, result, connectivity);
}

void WriteReport(const std::filesystem::path& viewer_input,
    const std::filesystem::path& destination, const output::Document& document,
    AnalysisLimits limits) {
    output::Require(limits.report_bytes && limits.report_bytes <= 16u << 20 &&
            document.IsObject(),
        "Impact analysis report cap or document is invalid");
    const auto input_root =
        std::filesystem::canonical(std::filesystem::absolute(viewer_input).parent_path());
    const auto output_path = std::filesystem::absolute(destination);
    const auto output_parent = output_path.parent_path();
    output::Require(!output_path.filename().empty() &&
            std::filesystem::symlink_status(output_parent).type() ==
                std::filesystem::file_type::directory &&
            !std::filesystem::is_symlink(output_parent) &&
            !std::filesystem::exists(std::filesystem::symlink_status(output_path)),
        "Impact analysis report destination is not a new file under a real directory");
    const auto canonical_parent = std::filesystem::canonical(output_parent);
    output::Require(!IsWithin(input_root,
            canonical_parent / output_path.filename()),
        "Impact analysis report must remain outside the immutable input run");

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(document.Accept(writer),
        "Impact analysis report serialization failed");
    std::string bytes(buffer.GetString(), buffer.GetSize());
    bytes.push_back('\n');
    output::Require(bytes.size() <= limits.report_bytes,
        "Impact analysis report exceeds its byte cap");
    output::WriteBytes(output_path, bytes);
}

}  // namespace crash::analysis::impact_response
