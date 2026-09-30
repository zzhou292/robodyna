#include "Internal.h"
#include "modelio/physical_domain/Policy.h"
#include "case/CanonicalWallArtifacts.h"
#include "lib_src/math/ScalarBits.h"
#include "case/vehicle_self_contact/native/TopologyDigestFields.h"
#include <new>
namespace crash::cases::vehicle_wall::native {
struct WallSource::Data {
    Data(const modelio::physical_domain::VehiclePhysicalDomain& v,detail::DomainValues&& values)
        :vehicle(v),domain(values.domain),fixed(std::move(values.fixed)),rotation(std::move(values.rotation)){}
    modelio::physical_domain::VehiclePhysicalDomain vehicle;
    tl::fea::NodalNodeDomain domain;
    Declaration declaration;
    Geometry geometry;
    AllocatedIds ids;
    NamespaceReport namespace_report;
    VehiclePrefix prefix;
    tl::fea::ShellBatchStartup startup;
    std::vector<std::uint8_t> fixed,rotation;
    std::string wall_manifest,digest;
    Forecast forecast;
};
namespace detail {
void CheckLimits(Limits limits) {
    const Limits hard;
    const std::size_t values[]{limits.host_bytes,limits.namespace_entries,limits.namespace_bytes,
        limits.source_metadata_bytes,limits.domain_bytes,limits.metadata_bytes};
    const std::size_t caps[]{hard.host_bytes,hard.namespace_entries,hard.namespace_bytes,
        hard.source_metadata_bytes,hard.domain_bytes,hard.metadata_bytes};
    for(unsigned i=0;i<std::size(values);++i)
        if(!values[i]||values[i]>caps[i])Reject(Status::ResourceLimit,"Invalid bounded wall source limits");
}
}
namespace {
void Add(std::size_t& total,std::size_t value) {
    if(value>SIZE_MAX-total)detail::Reject(Status::ResourceLimit,"Wall source forecast overflow");
    total+=value;
}
}
Forecast WallSource::Preflight(const modelio::physical_domain::VehiclePhysicalDomain& vehicle,
        const modelio::native_spring_ids::ImportMembers& members,Limits limits) {
    detail::CheckLimits(limits);
    using Policy=modelio::physical_domain::Policy;
    if(!modelio::physical_domain::detail::HasVehicleSupports(vehicle.policy()))
        detail::Reject(Status::UnsupportedSource,"Envelope supplement requires the complete named retained V5 domain");
    const auto count=vehicle.domain().node_count();
    const auto hard=tl::fea::NodalDomainLimits::Vehicle();
    if(!count||count>hard.max_nodes-4)
        detail::Reject(Status::ResourceLimit,"Combined source domain exceeds native capacity");
    const auto& canonical=vehicle.source().tied_source().canonical();
    const auto source=modelio::native_spring_ids::ImportContext::Preflight(canonical,members);
    Forecast f;
    // Inclusive upstream reservations may overlap; no undocumented shared
    // source discount is used. The new domain copy is charged in full.
    f.retained_vehicle=vehicle.forecast().total_bytes;
    f.import_context=source.total_bytes;
    f.namespace_workspace=detail::NamespaceWorkspace(canonical.data(),members,limits);
    f.common_domain=limits.domain_bytes;
    f.input_packing=2*(count+4)*sizeof(tl::fea::NodalDomainNode);
    f.masks=4*(count+4);
    f.geometry=sizeof(Data)+2*(1u<<20)+65536;
    f.digest=2*vehicle_self_contact::native::detail::digest::ChunkWords*sizeof(std::uint64_t)+
        2*limits.metadata_bytes+65536;
    for(const auto bytes:{f.retained_vehicle,f.import_context,f.namespace_workspace,f.common_domain,
        f.input_packing,f.masks,f.geometry,f.digest})Add(f.peak_bytes,bytes);
    if(f.peak_bytes>limits.host_bytes)detail::Reject(Status::ResourceLimit,"Complete wall supplement forecast exceeds cap");
    return f;
}
Preparation WallSource::Prepare(const modelio::physical_domain::VehiclePhysicalDomain& vehicle,
        const modelio::native_spring_ids::ImportMembers& members,const case_data::CanonicalWall& wall,
        const std::string& wall_bytes,const Declaration& declaration,Limits limits) {
    Preparation result;
    try {
        const auto forecast=Preflight(vehicle,members,limits);
        detail::Check(declaration);
        case_data::CheckCanonicalWallBinding(wall,wall_bytes);
        if(wall_bytes.size()>1u<<20)detail::Reject(Status::ResourceLimit,"Original wall provenance exceeds cap");
        const auto& canonical=vehicle.source().tied_source().canonical();
        const auto context=modelio::native_spring_ids::ImportContext::Prepare(canonical,members);
        bool same_combine=false;
        for(const auto& member:context.data().members)
            same_combine=same_combine||member.sha256==wall.provenance().combine_sha256;
        detail::Require(same_combine,"Canonical wall and vehicle import use different combine source");
        auto allocation=detail::Namespace(context,members,wall.provenance().wall_sha256,limits);
        const auto bounds=detail::Bounds(vehicle.domain());
        auto geometry=detail::BuildGeometry(bounds,declaration,allocation.second,canonical.data().inputs.units);
        auto combined=detail::BuildDomain(vehicle.domain(),allocation.second,geometry,limits);
        auto next=std::make_shared<Data>(vehicle,std::move(combined));
        next->declaration=declaration;next->namespace_report=std::move(allocation.first);next->ids=allocation.second;
        next->geometry=std::move(geometry);next->wall_manifest=wall_bytes;next->forecast=forecast;
        const auto count=vehicle.domain().node_count();
        next->startup={tl::fea::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,
            {vehicle_runtime::InitialSpeedMps,0,0}};
        next->prefix.nodes=count;next->prefix.reference_bounds=bounds;
        next->prefix.source_digest=canonical.data().inputs.canonical_manifest.sha256;
        next->digest=detail::Digest(canonical.data(),declaration,next->namespace_report,next->ids,
            next->geometry,next->domain,next->prefix,limits.metadata_bytes);
        result.source=WallSource(std::move(next));
        result.report={Status::Ready,"Declared envelope wall and fresh common domain; no runtime/contact readiness"};
    } catch(const detail::Error& error) {result.report=error.report;}
      catch(const std::bad_alloc&) {result.report={Status::ResourceLimit,"Wall supplement allocation failed"};}
      catch(const std::exception& error) {result.report={Status::InvalidInput,std::string(error.what()).substr(0,1024)};}
    return result;
}
const modelio::physical_domain::VehiclePhysicalDomain& WallSource::vehicle_origin() const noexcept{return data_->vehicle;}
const tl::fea::NodalNodeDomain& WallSource::domain() const noexcept{return data_->domain;}
const VehiclePrefix& WallSource::vehicle_prefix() const noexcept{return data_->prefix;}
const Declaration& WallSource::declaration() const noexcept{return data_->declaration;}
const Geometry& WallSource::geometry() const noexcept{return data_->geometry;}
const AllocatedIds& WallSource::ids() const noexcept{return data_->ids;}
const NamespaceReport& WallSource::namespace_report() const noexcept{return data_->namespace_report;}
const tl::fea::ShellBatchStartup& WallSource::startup() const noexcept{return data_->startup;}
tl::util::ConstView<std::uint8_t> WallSource::translation_fixed_bits() const noexcept{return {data_->fixed.data(),data_->fixed.size()};}
tl::util::ConstView<std::uint8_t> WallSource::rotation_fixed() const noexcept{return {data_->rotation.data(),data_->rotation.size()};}
const std::string& WallSource::digest() const noexcept{return data_->digest;}
const Forecast& WallSource::forecast() const noexcept{return data_->forecast;}
} // namespace crash::cases::vehicle_wall::native
