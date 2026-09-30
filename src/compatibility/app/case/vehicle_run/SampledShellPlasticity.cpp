#include "SampledShellPlasticity.h"
#include "output/full_shell/FullShellVisualizationRecords.h"
#include <algorithm>
#include <cmath>
#include <ostream>

namespace crash::cases::vehicle_run::detail {
namespace records = output::full_shell;

SampledShellPlasticityTotals SummarizeShellSample(const SampledShellPlasticityTotals& before,
    const records::Context& context, const records::FrameRecord& frame) {
    using output::Require;
    records::CheckFrame(context, {frame.stamp, frame.position_xyz.data(), frame.position_xyz.size(),
        frame.plastic_points.data(), frame.plastic_points.size()});
    Require(before.available == (before.saved_samples != 0) && before.saved_samples != UINT64_MAX,
        "Saved shell sample summary count is invalid or exhausted");
    if (before.available) {
        Require(frame.stamp.epoch > before.last_epoch && frame.stamp.attempt > before.last_attempt &&
            frame.stamp.time > before.last_time_s && std::isfinite(before.last_time_s) &&
            before.native_points == context.points() &&
            std::isfinite(before.peak_saved_native_equivalent_plastic_strain) &&
            before.peak_saved_native_equivalent_plastic_strain >= 0,
            "Saved shell sample must advance the same captured point population and accepted phase");
    } else {
        Require(frame.stamp.epoch == 0, "Saved shell summary starts with the actual initial frame");
    }

    auto next = before;
    next.last_positive_points = 0;
    next.last_max_native_equivalent_plastic_strain = 0;
    for (const double value : frame.plastic_points) {
        next.last_positive_points += value > 0;
        next.last_max_native_equivalent_plastic_strain =
            std::max(next.last_max_native_equivalent_plastic_strain, value);
    }
    next.available = true;
    next.native_fields_available = context.points() != 0;
    ++next.saved_samples;
    next.last_epoch = frame.stamp.epoch;
    next.last_attempt = frame.stamp.attempt;
    next.last_time_s = frame.stamp.time;
    next.native_points = context.points();
    next.peak_saved_native_equivalent_plastic_strain = std::max(
        before.available ? before.peak_saved_native_equivalent_plastic_strain : 0.,
        next.last_max_native_equivalent_plastic_strain);
    if (!next.positive_sample_observed && next.last_positive_points) {
        next.positive_sample_observed = true;
        next.first_positive_saved_epoch = frame.stamp.epoch;
        next.first_positive_saved_time_s = frame.stamp.time;
    }
    return next;
}

output::Document SampledShellPlasticityDocument(const SampledShellPlasticityTotals& sample) {
    using namespace output;
    Document document;
    document.SetObject();
    String(document, "schema", "robo_dyna.sampled_shell_plasticity.v1");
    String(document, "scope", "successfully saved accepted shell frames; all stored native points, including inactive-parent histories; not continuous first yield");
    Boolean(document, "available", sample.available);
    if (!sample.available) return document;
    Integer(document, "saved_samples", sample.saved_samples);
    Integer(document, "last_saved_epoch", sample.last_epoch);
    Integer(document, "last_saved_attempt", sample.last_attempt);
    Number(document, "last_saved_time_s", sample.last_time_s);
    Boolean(document, "native_fields_available", sample.native_fields_available);
    Integer(document, "stored_native_points", sample.native_points);
    if (!sample.native_fields_available) return document;
    Integer(document, "last_saved_positive_points", sample.last_positive_points);
    Number(document, "last_saved_max_native_equivalent_plastic_strain", sample.last_max_native_equivalent_plastic_strain);
    Number(document, "peak_saved_native_equivalent_plastic_strain", sample.peak_saved_native_equivalent_plastic_strain);
    Boolean(document, "positive_saved_sample_observed", sample.positive_sample_observed);
    if (sample.positive_sample_observed) {
        Integer(document, "first_positive_saved_epoch", sample.first_positive_saved_epoch);
        Number(document, "first_positive_saved_time_s", sample.first_positive_saved_time_s);
    }
    return document;
}

void WriteSampledShellPlasticityProgress(std::ostream& stream, const SampledShellPlasticityTotals& sample) {
    stream << " shell_plasticity_scope=saved_frames"
           << " sampled_shell_available=" << sample.available;
    if (!sample.available) return;
    stream << " last_saved_shell_epoch=" << sample.last_epoch
           << " last_saved_shell_time_s=" << sample.last_time_s
           << " sampled_shell_native_fields_available=" << sample.native_fields_available;
    if (!sample.native_fields_available) return;
    stream << " last_saved_shell_positive_points=" << sample.last_positive_points
           << " last_saved_shell_max_native_equiv_plastic_strain=" << sample.last_max_native_equivalent_plastic_strain
           << " peak_saved_shell_native_equiv_plastic_strain=" << sample.peak_saved_native_equivalent_plastic_strain
           << " positive_saved_shell_sample_observed=" << sample.positive_sample_observed;
    if (sample.positive_sample_observed) {
        stream << " first_positive_saved_shell_epoch=" << sample.first_positive_saved_epoch
               << " first_positive_saved_shell_time_s=" << sample.first_positive_saved_time_s;
    }
}

} // namespace crash::cases::vehicle_run::detail
