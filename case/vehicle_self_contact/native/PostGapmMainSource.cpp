#include "mixed_main/Internal.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm {
namespace d = detail;
struct PostGapmMainSource::Data {
    Data(const Mixed& a, const GapOperands& b) : mixed(a), gaps(b) {}
    Mixed mixed;
    GapOperands gaps;
    d::Values values;
    Forecast forecast;
    Provenance provenance;
};
Forecast PostGapmMainSource::Preflight(const Mixed& source, const GapOperands& gaps, Limits limits) {
    return d::Budget(source,gaps,limits);
}
Preparation PostGapmMainSource::Prepare(const Mixed& source, const GapOperands& gaps,
    const std::string& combine, Limits limits) {
    Preparation result;
    try {
        const auto capacity=Preflight(source,gaps,limits);
        const auto& initial=source.initial();
        d::old::CheckContext(initial.context(),initial.selection());
        const bool grouping=d::old::GroupingContext(initial.selection(),combine);
        auto next=std::make_shared<Data>(source,gaps);
        d::Maps(source,next->values);
        {
            main_coefficients::Limits shell_limits;
            shell_limits.parts=limits.parts;shell_limits.metadata_bytes=limits.metadata_bytes;
            const auto packed=d::old::PackShells(initial.context(),initial.geometry(),shell_limits);
            for (const auto& part:packed.parts)
                d::Require(part.material && part.section && d::OrdinaryNativeProperty(*part.material,*part.section),
                    "Gap IGTYP1 source profile requires ordinary material and absent shell IRID");
            d::Resolve(source,packed,grouping,next->values);
        }
        d::Gaps(source,gaps,next->values,limits);
        next->provenance.units=initial.geometry().units;
        next->provenance.source_digest=initial.context().provenance().source_digest;
        next->provenance.mixed_digest=source.provenance().output_digest;
        next->provenance.gap_operand_digest=gaps.provenance().operand_digest;
        next->provenance.output_digest=d::Digest(source,gaps,next->values,limits.metadata_bytes);
        next->forecast=capacity;
        tl::util::BoundedArenaLayout retained(limits.host_bytes);tl::util::ArenaRegion unused;
        static_assert(sizeof(Data)+sizeof(PostGapmMainSource)<16384);
        for(const auto bytes:{capacity.mixed_retained,capacity.gap_additional_retained,
                d::RetainedValues(next->values,limits.host_bytes),std::size_t{16384}})
            d::Require(retained.Append<std::byte>(bytes,unused),"Post-GAPM retained source overflow");
        d::Require(retained.bytes()<=capacity.retained_bytes,"Post-GAPM actual vector capacity exceeds preflight");
        next->forecast.retained_bytes=retained.bytes();
        result.source.emplace(PostGapmMainSource(std::move(next)));
        result.report={Status::Ready,"Complete mixed main support/K and source gap arrays; normals/history/runtime remain separate"};
    } catch(const d::Failure& error) {
        result.source.reset();result.report=error.report;
    } catch(const d::old::Failure& error) {
        const auto status=error.report.status==main_coefficients::Status::ResourceLimit ? Status::ResourceLimit :
            error.report.status==main_coefficients::Status::NonfiniteResult ? Status::NonfiniteResult : Status::UnsupportedSource;
        result.source.reset();result.report={status,error.what(),error.report.primary,
            error.report.support_element,error.report.conflicting_element};
    } catch(const std::bad_alloc&) {
        result.source.reset();result.report={Status::ResourceLimit,"Post-GAPM source allocation failed"};
    } catch(const std::exception& error) {
        result.source.reset();result.report={Status::InvalidInput,std::string(error.what()).substr(0,1024)};
    }
    return result;
}
const Mixed& PostGapmMainSource::mixed() const noexcept {return data_->mixed;}
const GapOperands& PostGapmMainSource::gap_operands() const noexcept {return data_->gaps;}
s::Input PostGapmMainSource::startup_input() const noexcept {
    const auto& side=mixed().sides();const auto& v=data_->values;s::Input out;
    out.profile=s::Profile::MixedSurface;out.topology=s::TopologyPolicy::NativeMixedSurface;
    out.node_source_ids=v.node_ids.data();out.node_count=v.node_ids.size();
    out.positions={v.positions.data(),std::uint32_t(v.node_ids.size()),3,1};
    out.coordinates=s::Coordinates::Native;out.units=provenance().units;
    out.primary=mixed().primary();out.primary_count=side.primary_count;out.source_generation=side.source_generation;
    out.primary_identities=side.primary_identities;out.primary_identity_count=side.primary_count;
    out.shell_primary_count=side.shell_primary_count;out.raw_origins=side.raw_origins;
    out.raw_origin_to_primary=side.raw_origin_to_primary;out.raw_origin_count=side.raw_origin_count;
    return out;
}
s::PostGapmTopology PostGapmMainSource::post_gapm() const noexcept {
    const auto& v=data_->values;s::PostGapmTopology out;
    out.phase=s::PostGapmPhase::FinalizedBeforeNeighbors;out.primary_corners=v.corners.data();out.primary_count=v.corners.size();
    out.before_shell=v.before_shell.data();out.before_shell_count=v.before_shell.size();
    out.final_support=v.final_support.data();out.main_count=v.final_support.size();
    out.pre_shell_internal_count=v.counts.pre_shell_internal;out.incoming_solid_erosion=v.incoming_erosion;
    out.final_solid_erosion=v.final_erosion;out.source_generation=mixed().sides().source_generation;return out;
}
tl::util::ConstView<double> PostGapmMainSource::coefficients() const noexcept {return {data_->values.coefficients.data(),data_->values.coefficients.size()};}
tl::util::ConstView<PhysicalOwner> PostGapmMainSource::primary_owners() const noexcept {return {data_->values.owners.data(),data_->values.owners.size()};}
tl::util::ConstView<Geometry> PostGapmMainSource::primary_geometry() const noexcept {return {data_->values.geometry.data(),data_->values.geometry.size()};}
tl::util::ConstView<std::uint32_t> PostGapmMainSource::secondary_nodes() const noexcept {return {data_->values.secondary_nodes.data(),data_->values.secondary_nodes.size()};}
tl::util::ConstView<std::uint32_t> PostGapmMainSource::main_nodes() const noexcept {return {data_->values.main_nodes.data(),data_->values.main_nodes.size()};}
tl::util::ConstView<double> PostGapmMainSource::secondary_gaps() const noexcept {return {data_->values.secondary_gaps.data(),data_->values.secondary_gaps.size()};}
tl::util::ConstView<double> PostGapmMainSource::main_node_gaps() const noexcept {return {data_->values.main_node_gaps.data(),data_->values.main_node_gaps.size()};}
tl::util::ConstView<n::source_gaps::MainGapFields> PostGapmMainSource::main_gaps() const noexcept {return {data_->values.main_gaps.data(),data_->values.main_gaps.size()};}
const n::source_gaps::Profile& PostGapmMainSource::gap_profile() const noexcept {return data_->values.gap_profile;}
const n::source_gaps::Report& PostGapmMainSource::gap_report() const noexcept {return data_->values.gap_report;}
const Counts& PostGapmMainSource::counts() const noexcept {return data_->values.counts;}
const Provenance& PostGapmMainSource::provenance() const noexcept {return data_->provenance;}
const Forecast& PostGapmMainSource::forecast() const noexcept {return data_->forecast;}
}
