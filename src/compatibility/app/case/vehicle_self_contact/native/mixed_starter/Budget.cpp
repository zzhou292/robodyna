#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::mixed_starter::detail {
namespace {
void Add(std::size_t& total, std::size_t bytes) {
    if (bytes > SIZE_MAX-total) Reject(Status::ResourceLimit, "Mixed Starter forecast overflows");
    total += bytes;
}
}
Forecast Budget(const MainSource& source, Limits limits, const DomainEmbedding* embedding) {
    const Limits hard;
    if (!limits.host_bytes || limits.host_bytes > hard.host_bytes ||
        !limits.metadata_bytes || limits.metadata_bytes > hard.metadata_bytes)
        Reject(Status::ResourceLimit, "Invalid mixed Starter source limits");
    const auto input = source.startup_input();
    const auto post = source.post_gapm();
    const auto& sides = source.mixed().sides();
    const auto& previous = source.forecast();
    if (!previous.retained_bytes || previous.peak_bytes < previous.retained_bytes ||
        source.coefficients().size() != sides.main_count ||
        source.provenance().source_digest.empty() || source.provenance().output_digest.empty())
        Reject(Status::InvalidInput, "Incomplete immutable post-GAPM source authority");
    if (embedding) CheckEmbedding(source, *embedding);
    const auto exact = embedding ? s::ForecastMixedStarterStorage(embedding->domain().node_count(),
        input.primary_count, input.shell_primary_count, input.raw_origin_count, limits.numerical) :
        s::PreflightMixedStarter(input, sides, post, limits.numerical);
    if (exact.status != s::Status::Ok) {
        Report report;
        report.status = exact.status == s::Status::ResourceLimit ? Status::ResourceLimit : Status::UnsupportedSource;
        report.reason = "Post-GAPM source rejected by mixed Starter preflight";
        report.numerical_stage = NumericalStage::Forecast;
        report.startup_report.status = exact.status;
        throw Failure{std::move(report)};
    }
    Forecast out;
    out.prior_construction_peak = previous.peak_bytes;
    out.upstream_retained = previous.retained_bytes;
    if (embedding) {
        out.embedding_additional = embedding->incremental_backing_bytes();
        Add(out.embedding_additional, embedding->domain().owned_payload_bytes());
        out.embedding_prior_peak = previous.retained_bytes;
        Add(out.embedding_prior_peak, embedding->forecast().peak_bytes);
        out.prior_construction_peak = std::max(out.prior_construction_peak, out.embedding_prior_peak);
        // The two vectors retain exact prefix bits and genuine converted suffix
        // coordinates. Actual capacities are checked after allocation.
        const auto nodes = embedding->domain().node_count();
        if (nodes > SIZE_MAX/(2*(sizeof(std::uint64_t)+3*sizeof(double))))
            Reject(Status::ResourceLimit, "Combined node input forecast overflows");
        out.combined_inputs = 2*nodes*(sizeof(std::uint64_t)+3*sizeof(double));
    }
    out.starter_output = exact.output_bytes;
    out.starter_scratch = exact.scratch_bytes;
    out.normal_source_validation = c::MixedSourceValidationBytes(sides.primary_count);
    if (!out.normal_source_validation || out.normal_source_validation > c::Limits{}.source_validation_bytes)
        Reject(Status::ResourceLimit, "Mixed normal source certificate exceeds its public cap");
    // Three short digest strings, report/control objects and shared backing.
    // Prepare checks Data's bounded object size; no hidden full input copy.
    out.retained_metadata = 64u << 10;
    out.digest_and_report = (4u << 20) + 16*limits.metadata_bytes;
    out.retained_bytes = out.upstream_retained;
    Add(out.retained_bytes, out.embedding_additional);
    Add(out.retained_bytes, out.combined_inputs);
    Add(out.retained_bytes, out.starter_output);
    Add(out.retained_bytes, out.retained_metadata);
    // These temporary phases are sequential. The TL scratch is destroyed
    // before the source counter allocation, and both die before field hashing.
    out.current_phase = out.retained_bytes;
    Add(out.current_phase, std::max({out.starter_scratch, out.normal_source_validation, out.digest_and_report}));
    out.peak_bytes = std::max(out.prior_construction_peak, out.current_phase);
    if (out.peak_bytes > limits.host_bytes)
        Reject(Status::ResourceLimit, "Mixed Starter source exceeds inclusive host cap");
    return out;
}
}
