#include "nodal_correction/Internal.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_self_contact::native::nodal_correction {
namespace d = detail;
struct CorrectedNodalSource::Data {
    Data(const seed::PreCorrectionNodalSource& source, const d::ids::ImportContext& imported, Forecast capacity)
        : input(source), import(imported), forecast(capacity) {}
    seed::PreCorrectionNodalSource input;
    d::ids::ImportContext import;
    Forecast forecast;
    Provenance provenance;
    std::vector<PartControl> parts;
    std::vector<double> coefficients;
};
Forecast CorrectedNodalSource::Preflight(const seed::PreCorrectionNodalSource& input,
    const modelio::native_spring_ids::ImportMembers& members, Limits limits) {
    return d::Budget(input, members, limits);
}
Preparation CorrectedNodalSource::Prepare(const seed::PreCorrectionNodalSource& input,
    const modelio::native_spring_ids::ImportMembers& members, Limits limits) {
    Preparation result;
    try {
        const auto forecast = Preflight(input, members, limits);
        const auto& canonical = input.physical().shell_source().references().source().canonical();
        const auto imported = d::ids::ImportContext::Prepare(canonical, members);
        auto context = d::ReadContext(input, members, imported, limits);
        auto next = std::make_shared<Data>(input, imported, forecast);
        next->provenance.source_digest = context.source_digest;
        next->provenance.pre_correction_digest = input.provenance().contributor_digest;
        next->provenance.property_digest = context.property_digest;
        next->provenance.interfaces = context.interfaces;
        if (context.interfaces.disposition != InterfaceDisposition::CompleteNoApplicableType24)
            d::Reject(Status::UnsupportedSource, "Complete applicable TYPE24 source disposition is unavailable");
        std::vector<d::c::Solid> solids;
        std::vector<std::uint64_t> source_ids;
        d::PackControls(input, context, solids, source_ids);
        const d::c::OrderInput order{solids.empty() ? nullptr : solids.data(), solids.size(), input.counts().nodes};
        d::c::Forecast numerical;
        auto admission = d::c::PreflightOrderCertificate(order, {}, numerical);
        if (admission.status != d::c::Status::Ok)
            d::Reject(admission.status == d::c::Status::ResourceLimit ? Status::ResourceLimit :
                admission.status == d::c::Status::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidInput,
                "Native order-certificate admission rejected complete source operands");
        if (numerical.scratch_bytes > forecast.certificate_scratch)
            d::Reject(Status::ResourceLimit, "Order-certificate scratch exceeds its admitted bound");
        tl::util::HostArena proof_scratch;
        if (!proof_scratch.Initialize(numerical.scratch_bytes))
            d::Reject(Status::ResourceLimit, "Order-certificate scratch allocation failed");
        const auto proof = d::c::CertifyOrder(order, {}, proof_scratch.data(), proof_scratch.bytes(),
            &next->provenance.certificate);
        if (proof.status == d::c::OrderStatus::NeedsNativeStorageOrder) {
            result.report.status = Status::NeedsNativeStorageOrder;
            result.report.reason = "Shared node has different native factor bits; genuine IXS storage order is required";
            if (proof.first_solid < source_ids.size()) result.report.first_element = source_ids[proof.first_solid];
            if (proof.conflicting_solid < source_ids.size()) result.report.conflicting_element = source_ids[proof.conflicting_solid];
            if (proof.node < input.counts().nodes)
                result.report.source_node = input.physical().source_domain().domain().nodes()[proof.node].source_id;
            return result;
        }
        if (proof.status != d::c::OrderStatus::Ok)
            d::Reject(Status::InvalidInput, "Native order certificate failed without publication");
        std::vector<double> before;
        before.reserve(input.counts().nodes);
        for (const auto& node : input.fields()) before.push_back(node.stiffness_before_control);
        if (before.size() != input.counts().nodes)
            d::Reject(Status::InvalidInput, "Pre-correction nodal coefficient coverage differs");
        d::c::Input native_input;
        native_input.coefficients = before.data();
        native_input.node_count = before.size();
        native_input.solids = solids.empty() ? nullptr : solids.data();
        native_input.solid_count = solids.size();
        // Empty TYPE24 span is supplied only after the complete source closure
        // above, never inferred from the selected TYPE25 handle alone.
        admission = d::c::Preflight(native_input, {}, numerical);
        if (admission.status != d::c::Status::Ok)
            d::Reject(Status::InvalidInput, "Native nodal correction rejected the complete source input");
        if (numerical.scratch_bytes > forecast.correction_scratch || numerical.output_bytes > forecast.output_bytes)
            d::Reject(Status::ResourceLimit, "Native nodal correction exceeds its admitted reservation");
        tl::util::HostArena scratch;
        if (!scratch.Initialize(numerical.scratch_bytes))
            d::Reject(Status::ResourceLimit, "Native nodal correction scratch allocation failed");
        next->coefficients.resize(before.size());
        const auto applied = d::c::Apply(native_input, {}, scratch.data(), scratch.bytes(),
            {next->coefficients.data(), next->coefficients.size()});
        if (applied.status != d::c::Status::Ok)
            d::Reject(applied.status == d::c::Status::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidInput,
                "Native nodal correction failed before publication");
        next->provenance.material_digest = d::MaterialDigest(solids, context, limits.metadata_bytes);
        next->provenance.certificate_digest = d::CertificateDigest(next->provenance.certificate,
            next->provenance.material_digest, limits.metadata_bytes);
        next->parts = std::move(context.parts);
        result.report.status = Status::Ready;
        result.report.reason = "Complete global nodal coefficients corrected under certified factor equivalence";
        // Report string allocation precedes the final no-throw handle copy.
        result.source.emplace(CorrectedNodalSource(std::move(next)));
    } catch (const d::Failure& failure) {
        result.source.reset();
        result.report = failure.report;
    } catch (const std::bad_alloc&) {
        result.source.reset();
        result.report.status = Status::ResourceLimit;
        result.report.reason = "Corrected nodal source allocation failed";
    } catch (const std::exception& error) {
        result.source.reset();
        result.report.status = Status::InvalidInput;
        result.report.reason = std::string(error.what()).substr(0, 1024);
    }
    return result;
}
const seed::PreCorrectionNodalSource& CorrectedNodalSource::pre_correction() const noexcept { return data_->input; }
const Forecast& CorrectedNodalSource::forecast() const noexcept { return data_->forecast; }
const Provenance& CorrectedNodalSource::provenance() const noexcept { return data_->provenance; }
const std::vector<PartControl>& CorrectedNodalSource::part_controls() const noexcept { return data_->parts; }
tl::util::ConstView<double> CorrectedNodalSource::coefficients() const noexcept {
    return {data_->coefficients.data(), data_->coefficients.size()};
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction
