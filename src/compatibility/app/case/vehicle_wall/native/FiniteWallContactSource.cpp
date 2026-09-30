#include "contact/Internal.h"
#include "lib_src/math/ScalarBits.h"
namespace crash::cases::vehicle_wall::native::wall_interface {
namespace d=detail;
struct FiniteWallContactSource::Data {
    Data(const EnvelopeOwnerSource& owner,const VehicleSource& input):vehicle(input),
        wall(owner.execution_source().mechanical().wall()),embedding(owner.execution_source().mechanical().embedding()){}
    VehicleSource vehicle;
    WallSource wall;
    modelio::physical_scope::DomainEmbedding embedding;
    Declaration declaration;
    Controls controls;
    Provenance provenance;
    Forecast forecast;
    d::Fields fields;
};
Forecast FiniteWallContactSource::Preflight(const EnvelopeOwnerSource& owner,const VehicleSource& vehicle,
    Declaration declaration,Limits limits) {
    return d::Budget(owner,vehicle,declaration,limits).forecast;
}
Preparation FiniteWallContactSource::Prepare(const EnvelopeOwnerSource& owner,const VehicleSource& vehicle,
    Declaration declaration,Limits limits) {
    Preparation result;
    try {
        const auto plan=d::Budget(owner,vehicle,declaration,limits);
        static_assert(sizeof(Data)+64<(64u<<10));
        auto next=std::make_shared<Data>(owner,vehicle);
        next->declaration=declaration;next->forecast=plan.forecast;
        next->controls=d::ResolveControls(next->wall,vehicle,declaration);
        auto& fields=next->fields;
        d::Allocate(fields,plan);
        d::PackNodes(fields,owner,vehicle,plan);
        // Four-node disjoint component; no vehicle assembly or source sums are
        // rerun. Actual wall masses remain in the sole physical ledger.
        const auto component=d::WallComponent(next->wall.declaration().material,plan.shape.units,
            next->wall.ids().shell,plan.shape.wall_nodes);
        if(!tl::math::SameScalarBits(component.primary_coefficient,next->wall.geometry().component_primary_stiffness_native) ||
            !tl::math::SameScalarBits(component.half_gap,next->wall.geometry().native_half_gap))
            d::Reject(Status::SourceMismatch,"Wall component differs from its qualified declared geometry/material source");
        for(unsigned k=0;k<4;++k)fields.global_k[plan.shape.wall_nodes[k]]=component.global_coefficients[k];
        d::ScaleSecondary(fields.global_k.data(),fields.global_k.size(),fields.nsv.data(),fields.nsv.size(),1.,fields.secondary_k.data());
        fields.main_k.fill(component.primary_coefficient);
        d::BuildTopology(fields,next->wall,declaration,plan);
        d::BuildGaps(fields,vehicle,component,next->controls,plan);
        next->provenance.interface_id=next->wall.ids().interface;
        next->provenance.source_digest=vehicle.provenance().source_digest;
        next->provenance.vehicle_digest=vehicle.provenance().output_digest;
        next->provenance.wall_digest=next->wall.digest();
        next->provenance.all_retained_vehicle_additional_nodes=true;
        next->provenance.output_digest=d::Digest(fields,next->controls,next->provenance,declaration,limits.metadata_bytes);
        result.source.emplace(FiniteWallContactSource(std::move(next)));
        result.report={Status::Prepared,"Source-correct finite wall fields; general initialization and final CSR/history still required"};
    } catch(const d::Failure& failure) {
        result.report=failure.report;result.source.reset();
    } catch(const std::bad_alloc&) {
        result.report={Status::ResourceLimit,"Finite-wall source allocation failed"};result.source.reset();
    } catch(const std::exception& failure) {
        result.report={Status::InvalidInput,std::string(failure.what()).substr(0,1024)};result.source.reset();
    }
    return result;
}
const VehicleSource& FiniteWallContactSource::vehicle_source() const noexcept{return data_->vehicle;}
const WallSource& FiniteWallContactSource::wall() const noexcept{return data_->wall;}
const modelio::physical_scope::DomainEmbedding& FiniteWallContactSource::embedding() const noexcept{return data_->embedding;}
const tl::fea::NodalNodeDomain& FiniteWallContactSource::domain() const noexcept{return data_->embedding.domain();}
const Declaration& FiniteWallContactSource::declaration() const noexcept{return data_->declaration;}
const Controls& FiniteWallContactSource::controls() const noexcept{return data_->controls;}
const Provenance& FiniteWallContactSource::provenance() const noexcept{return data_->provenance;}
const Forecast& FiniteWallContactSource::forecast() const noexcept{return data_->forecast;}
s::Input FiniteWallContactSource::startup_input() const noexcept{return data_->fields.Mesh(data_->declaration,data_->controls.runtime.units);}
const s::Snapshot& FiniteWallContactSource::starter() const noexcept{return data_->fields.starter;}
const s::FixedMainView& FiniteWallContactSource::fixed_ready() const noexcept{return data_->fields.ready;}
tl::util::ConstView<n::lifecycle::Node> FiniteWallContactSource::nodes() const noexcept{return {data_->fields.nodes.data(),data_->fields.nodes.size()};}
tl::util::ConstView<std::uint32_t> FiniteWallContactSource::secondary_nodes() const noexcept{return {data_->fields.nsv.data(),data_->fields.nsv.size()};}
tl::util::ConstView<std::uint32_t> FiniteWallContactSource::main_nodes() const noexcept{return {data_->fields.msr.data(),data_->fields.msr.size()};}
tl::util::ConstView<double> FiniteWallContactSource::global_coefficients() const noexcept{return {data_->fields.global_k.data(),data_->fields.global_k.size()};}
tl::util::ConstView<double> FiniteWallContactSource::secondary_coefficients() const noexcept{return {data_->fields.secondary_k.data(),data_->fields.secondary_k.size()};}
tl::util::ConstView<double> FiniteWallContactSource::secondary_gaps() const noexcept{return {data_->fields.secondary_gap.data(),data_->fields.secondary_gap.size()};}
const n::source_gaps::Report& FiniteWallContactSource::gap_report() const noexcept{return data_->fields.gaps;}
tl::util::ConstView<double> FiniteWallContactSource::main_coefficients() const noexcept{return {data_->fields.main_k.data(),data_->fields.main_k.size()};}
tl::util::ConstView<n::source_gaps::MainGapFields> FiniteWallContactSource::main_gaps() const noexcept{return {data_->fields.main_gaps.data(),data_->fields.main_gaps.size()};}
}
