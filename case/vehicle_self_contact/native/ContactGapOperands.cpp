#include "gap_operands/Internal.h"
namespace crash::cases::vehicle_self_contact::native::gap_operands {
namespace d=detail;
struct ContactGapOperands::Data {
    Data(const source::CorrectedNodalSource& input,Forecast capacity):corrected(input),forecast(capacity){}
    source::CorrectedNodalSource corrected;
    Forecast forecast;
    Provenance provenance;
    d::Packed values;
};
Forecast ContactGapOperands::Preflight(const source::CorrectedNodalSource& input,Limits limits) {
    return d::Budget(input,limits);
}
Preparation ContactGapOperands::Prepare(const source::CorrectedNodalSource& input,Limits limits) {
    Preparation result;
    try {
        const auto capacity=Preflight(input,limits);
        const auto context=d::Context(input);
        const auto& seed=input.pre_correction();const auto& counts=seed.counts();
        const auto springs=counts.type13+counts.type25+counts.type45;
        auto next=std::make_shared<Data>(input,capacity);
        auto& values=next->values;
        values.proof=context;
        values.shells.reserve(counts.shells);values.beams.reserve(counts.beams);values.springs.reserve(springs);
        values.bindings.reserve(counts.shells+counts.beams+springs);
        values.counts.nodes=counts.nodes;
        d::Require(seed.contributors().size()==counts.solids+counts.shells+counts.beams+springs,
            "Complete physical contributor census differs before gap binding");
        std::size_t solids=0;
        for(const auto& contributor:seed.contributors()) {
            switch(contributor.kind) {
            case nodal_seed::ContributorKind::Solid18Law36:
            case nodal_seed::ContributorKind::HephLaw42:
            case nodal_seed::ContributorKind::PentaLaw42:
            case nodal_seed::ContributorKind::Solid18Law44:
            case nodal_seed::ContributorKind::Solid18Law90:++solids;break;
            case nodal_seed::ContributorKind::ShellQ4:
            case nodal_seed::ContributorKind::ShellT3:
            case nodal_seed::ContributorKind::Beam18:
            case nodal_seed::ContributorKind::Type13:
            case nodal_seed::ContributorKind::Type25:
            case nodal_seed::ContributorKind::Type45:break;
            default:d::Reject(Status::UnsupportedSource,"Unknown complete physical gap contributor family");
            }
        }
        d::Require(solids==counts.solids,"Gap physical solid disposition incomplete");
        values.counts.solids_without_direct_gap_term=solids;
        // Established jointly by Context's named V5 source authority and the
        // exhaustive admitted contributor-family census above. No omitted
        // original family is turned into a physical zero-gap contributor.
        values.proof.no_retained_trusses=true;
        d::Shells(input,values);d::Lines(input,values);d::Springs(input,values);
        d::Require(values.bindings.size()==counts.shells+counts.beams+springs &&
            values.shells.capacity()<=2*counts.shells&&values.beams.capacity()<=2*counts.beams&&
            values.springs.capacity()<=2*springs&&values.bindings.capacity()<=2*values.bindings.size(),
            "Complete gap operand allocation or source coverage differs");
        next->provenance.units=seed.provenance().units;
        next->provenance.source_digest=input.provenance().source_digest;
        next->provenance.contributor_digest=seed.provenance().contributor_digest;
        next->provenance.property_digest=input.provenance().property_digest;
        next->provenance.operand_digest=d::Digest(values,next->provenance,limits.metadata_bytes);
        result.report={Status::Ready,"Complete native gap operands; final mixed topology and roster gathering remain separate"};
        result.source.emplace(ContactGapOperands(std::move(next)));
    } catch(const d::Failure& failure) {
        result.source.reset();result.report=failure.report;
    } catch(const std::bad_alloc&) {
        result.source.reset();result.report={Status::ResourceLimit,"Gap operand source allocation failed"};
    } catch(const std::exception& error) {
        result.source.reset();result.report={Status::InvalidInput,std::string(error.what()).substr(0,1024)};
    }
    return result;
}
const source::CorrectedNodalSource& ContactGapOperands::corrected() const noexcept { return data_->corrected; }
tl::util::ConstView<values::PhysicalShell> ContactGapOperands::shells() const noexcept {
    return {data_->values.shells.data(),data_->values.shells.size()};
}
tl::util::ConstView<values::Line> ContactGapOperands::beams() const noexcept {
    return {data_->values.beams.data(),data_->values.beams.size()};
}
tl::util::ConstView<values::Spring> ContactGapOperands::springs() const noexcept {
    return {data_->values.springs.data(),data_->values.springs.size()};
}
tl::util::ConstView<Binding> ContactGapOperands::bindings() const noexcept {
    return {data_->values.bindings.data(),data_->values.bindings.size()};
}
const Counts& ContactGapOperands::counts() const noexcept { return data_->values.counts; }
const SourceProof& ContactGapOperands::proof() const noexcept { return data_->values.proof; }
const Provenance& ContactGapOperands::provenance() const noexcept { return data_->provenance; }
const Forecast& ContactGapOperands::forecast() const noexcept { return data_->forecast; }
}
