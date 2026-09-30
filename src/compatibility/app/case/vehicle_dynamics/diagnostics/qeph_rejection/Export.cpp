#include "Export.h"
#include "output/BoundedArrayJson.h"
#include "output/physical_run/RunArchive.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
namespace {
constexpr std::size_t MetadataBytes = 32u << 10;
constexpr std::size_t WordBytes = MaximumWords * sizeof(std::uint64_t);
void Error(ExportResult& result, const char* text) noexcept {
    if (!text) return;
    for (std::size_t i = 0; i + 1 < result.error.size() && text[i]; ++i) result.error[i] = text[i];
}
void Source(output::Document& document, const char* name, const native::RejectedSourceIds& source,
            bool law_available, tl::fea::ShellSectionLaw law) {
    output::Document value; value.SetObject();
    output::Boolean(value, "available", source.available);
    output::Integer(value, "parent_id", source.parent);
    output::Integer(value, "part_id", source.part);
    output::Integer(value, "material_id", source.material);
    output::Integer(value, "section_id", source.section);
    output::Boolean(value, "law_available", law_available);
    if (law_available) output::Integer(value, "law", static_cast<unsigned>(law));
    output::array_json::Child(document, name, value);
}
output::arrays::Descriptor Words(const std::filesystem::path& root, const char* name,
                               const std::vector<std::uint64_t>& words) {
    return output::arrays::Write(root, name,
        {output::arrays::Scalar::UInt64, words.size(), 1, {"value_bits"}},
        words.data(), words.size(), {WordBytes, MaximumWords, 1});
}
}
const char* StatusName(ExportStatus value) noexcept {
    switch (value) {
        case ExportStatus::NoRejectedCandidate: return "no_rejected_candidate";
        case ExportStatus::Captured: return "captured";
        case ExportStatus::CaptureIncomplete: return "capture_incomplete";
        case ExportStatus::ExportFailed: return "export_failed";
    }
    return "unknown";
}
ExportResult Export(const CaptureState& capture, const std::filesystem::path& destination) noexcept {
    ExportResult result;
    if (!capture.seen) return result;
    result.status = ExportStatus::CaptureIncomplete;
    try {
        output::Require(std::filesystem::symlink_status(destination).type() ==
                            std::filesystem::file_type::not_found &&
                            std::filesystem::create_directory(destination),
                        "QEPH rejection companion must be a new directory");
        output::Document doc; doc.SetObject();
        output::String(doc, "schema", "robo_dyna.qeph_rejected_candidate_input.v1");
        output::String(doc, "scope", "failed_element_diagnostic_not_restart");
        output::String(doc, "encoding", "field_order_le_uint64_ieee_binary64_bits_v1");
        output::String(doc, "capture_status", StatusName(capture.status));
        output::Integer(doc, "capture_status_code", static_cast<unsigned>(capture.status));
        output::Integer(doc, "cuda_status", static_cast<unsigned>(capture.cuda_status));
        output::String(doc, "capture_message", capture.message.data());
        output::Boolean(doc, "input_available", capture.complete);
        output::Integer(doc, "batch_status", static_cast<unsigned>(capture.original.status));
        output::Integer(doc, "element_index", capture.original.element);
        output::Integer(doc, "node_index", capture.original.node);
        output::Integer(doc, "element_status", static_cast<unsigned>(capture.original.element_status));
        output::Integer(doc, "nodal_status", static_cast<unsigned>(capture.original.nodal_status));
        Source(doc, "requested_source", capture.requested.source,
            capture.requested.law_available, capture.requested.law);
        if (capture.metadata_available) {
            const auto metadata = Words(destination, "metadata.words.bin", EncodeMetadata(capture.metadata));
            output::array_json::Child(doc, "metadata", output::arrays::DescriptorDocument(metadata));
            output::Integer(doc, "accepted_epoch", capture.metadata.owner.epoch);
            output::Integer(doc, "candidate_attempt", capture.metadata.candidate.attempt);
        }
        if (capture.complete) {
            Source(doc, "captured_source", capture.input.source, true, capture.input.law);
            const auto encoded = Encode(capture.input);
            const auto descriptor = Words(destination, "input.words.bin", encoded);
            const auto read = output::arrays::Read<std::uint64_t>(destination, descriptor,
                {WordBytes, MaximumWords, 1});
            output::Require(read == encoded, "QEPH diagnostic file readback differs");
            const auto decoded = Decode(read);
            output::Require(Encode(decoded) == encoded, "QEPH diagnostic decoded values differ");
            output::array_json::Child(doc, "input", output::arrays::DescriptorDocument(descriptor));
            native::RejectedReplayResult replay;
            const bool replayed = native::ReplayRejectedCandidate(decoded, &replay);
            output::Boolean(doc, "host_replay_available", replayed);
            if (replayed) {
                output::Integer(doc, "host_operator_status", static_cast<unsigned>(replay.operator_status));
                output::Boolean(doc, "host_mapped_result_checked", replay.mapped_result_checked);
                output::Boolean(doc, "host_mapped_result_valid", replay.mapped_result_valid);
                const bool reproduced = capture.original.status == native::BatchStatus::ElementFailure
                    ? replay.operator_status == capture.original.element_status
                    : capture.original.status == native::BatchStatus::NonfiniteResult &&
                        replay.operator_status == native::Status::kSuccess && replay.mapped_result_checked &&
                        !replay.mapped_result_valid;
                output::Boolean(doc, "host_reproduces_original_rejection", reproduced);
            }
            result.status = ExportStatus::Captured;
        }
        result.manifest = output::physical_run::WriteDocument(destination, "failure.json", doc, MetadataBytes);
    } catch (const std::exception& error) {
        result.status = ExportStatus::ExportFailed;
        Error(result, error.what());
    } catch (...) {
        result.status = ExportStatus::ExportFailed;
        Error(result, "Unknown QEPH diagnostic export failure");
    }
    return result;
}
ExportResult ExportForRun(const CaptureState& capture, const std::filesystem::path& run_root) noexcept {
    if (!capture.seen) return {};
    try { return Export(capture, run_root / "qeph-rejection"); }
    catch (const std::exception& error) {
        ExportResult result; result.status = ExportStatus::ExportFailed;
        Error(result, error.what()); return result;
    } catch (...) {
        ExportResult result; result.status = ExportStatus::ExportFailed;
        Error(result, "Unknown QEPH companion path failure"); return result;
    }
}
output::Document ExportDocument(const ExportResult& value) {
    output::Document doc; doc.SetObject();
    output::String(doc, "status", StatusName(value.status));
    output::String(doc, "scope", "diagnostic_not_restart");
    if (value.error[0]) output::String(doc, "error", value.error.data());
    if (value.manifest) {
        output::String(doc, "directory", "qeph-rejection");
        output::array_json::Child(doc, "manifest", output::physical_run::FileDocument(*value.manifest));
    }
    return doc;
}
}
