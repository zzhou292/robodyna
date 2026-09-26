#include "owner_source/Internal.h"
#include <optional>
#include <algorithm>
namespace crash::cases::vehicle_wall::native {
namespace d=owner_source_detail;
struct EnvelopeOwnerSource::Data {
    Data(const EnvelopeExecutionSource& e,const modelio::type45::VehicleType45Source& j)
        : execution(e),joint_source(j) {}
    EnvelopeExecutionSource execution;
    modelio::type45::VehicleType45Source joint_source;
    std::optional<vehicle_startup::TiedCinAttachments> attachments;
    std::optional<vehicle_startup::TiedCinWitnessRoster> witnesses;
    tl::fea::type45::Model joints;
    std::vector<std::uint32_t> joint_rows;
    vehicle_runtime::SourceRoles roles;
    EnvelopeOwnerForecast forecast;
};
EnvelopeOwnerForecast EnvelopeOwnerSource::Preflight(const EnvelopeExecutionSource& execution,
    const vehicle_startup::TiedSearchPostKinChk& post,const modelio::type45::VehicleType45Source& joints,
    EnvelopeOwnerLimits limits) {
    d::Check(execution,post,joints,limits);
    const auto& source=execution.mechanical();
    EnvelopeOwnerForecast f;
    f.execution_source=execution.retained_host_upper_bound(limits.host_bytes);
    f.previous_execution_peak=execution.forecast().total_bytes;
    f.cin=vehicle_startup::TiedCinAttachments::Forecast(post,source.domain(),limits.attachments).total_host_bytes;
    f.witness_reservation=limits.witnesses.host_bytes;
    f.joint_source=joints.forecast().total_bytes;f.joint_model=limits.joints.max_host_bytes;
    f.roles=tl::fea::NodalDomainLimits::Vehicle().max_nodes;
    f.owner_packing=vehicle_runtime::detail::PackingBytes(source.domain().node_count(),limits.host_bytes);
    f.fixed=sizeof(EnvelopeOwnerSource)+sizeof(Data)+4096;
    tl::util::BoundedArenaLayout packing(limits.host_bytes),all(limits.host_bytes);
    tl::util::ArenaRegion unused;
    d::Require(packing.Append<tl::fea::type45::JointInput>(joints.data().required,unused)&&
        packing.Append<std::uint32_t>(joints.data().required,unused),"Combined joint packing exceeds cap");
    f.joint_packing=packing.bytes();
    for(const auto bytes:{f.execution_source,f.cin,f.witness_reservation,f.joint_source,f.joint_model,
            f.roles,f.owner_packing,f.fixed,f.joint_packing})
        d::Require(all.Append<std::byte>(bytes,unused),"Complete combined owner-source forecast exceeds cap");
    f.current_phase=all.bytes();
    f.peak_bytes=std::max(f.previous_execution_peak,f.current_phase);
    d::Require(f.peak_bytes<=limits.host_bytes,"Complete owner-source chain exceeds cap");
    return f;
}
EnvelopeOwnerSource EnvelopeOwnerSource::Prepare(const EnvelopeExecutionSource& execution,
    const vehicle_startup::TiedSearchPostKinChk& post,const modelio::type45::VehicleType45Source& joints,
    EnvelopeOwnerLimits limits) {
    const auto forecast=Preflight(execution,post,joints,limits);
    const auto& source=execution.mechanical();
    auto next=std::make_shared<Data>(execution,joints);
    next->attachments.emplace(vehicle_startup::TiedCinAttachments::Prepare(post,source.domain(),limits.attachments));
    const auto& cin=next->attachments->model();
    d::Require(cin.rows().count==11165&&!cin.explicitly_empty(),"Combined source lost its genuine complete CIN rows");
    for(std::size_t i=0;i<cin.rows().count;++i) {
        const auto& row=cin.rows().data[i];
        d::Require(!source.rigid_assembly().FindMember(row.secondary_domain_node),
            "Combined CIN secondary intersects a rigid assembly");
        for(const auto node:row.master_domain_nodes)
            d::Require(!source.rigid_assembly().FindMember(node),"Combined CIN master intersects a rigid assembly");
    }
    next->witnesses.emplace(vehicle_startup::TiedCinWitnessRoster::PreparePhysical(cin,execution.physical(),limits.witnesses));
    d::Require(next->witnesses->runtime_mappable(),"Combined source lacks complete positive shell witnesses");
    std::vector<tl::fea::type45::JointInput> input;input.reserve(joints.data().required);
    next->joint_rows.reserve(joints.data().required);
    for(std::size_t i=0;i<joints.data().rows.size();++i) {
        const auto& row=joints.data().rows[i];
        if(row.disposition==modelio::type45::Disposition::OmittedAssemblyBoundary)continue;
        input.push_back(vehicle_startup::joints::detail::Pack(row,joints.data()));
        next->joint_rows.push_back(static_cast<std::uint32_t>(i));
    }
    d::Require(input.size()==44&&input.capacity()<=44&&next->joint_rows.capacity()<=44,
        "Combined source joint coverage or packing capacity differs");
    const auto made=next->joints.Initialize(source.rigid_assembly(),
        {source.domain().source_instance_id(),{input.data(),input.size()}},limits.joints);
    d::Require(bool(made),made.message);
    d::Require(next->joints.domain()->SharesStorage(source.domain()),"TYPE45 model retained a foreign domain");
    next->roles=vehicle_runtime::ResolvePhysicalSourceRoles(source.coefficients(),source.rigid_assembly(),cin,
        source.beams(),&source.structural_beams());
    for(const auto node:source.environment_parent().domain_nodes)
        d::Require(next->roles.node[node]==vehicle_runtime::Shell,"Declared wall acquired an original connector/constraint role");
    next->forecast=forecast;
    return EnvelopeOwnerSource(std::move(next));
}
const EnvelopeExecutionSource& EnvelopeOwnerSource::execution_source() const noexcept { return data_->execution; }
const tl::fea::ShellPhysicalBinding& EnvelopeOwnerSource::physical() const noexcept { return data_->execution.physical(); }
const vehicle_startup::TiedCinAttachments& EnvelopeOwnerSource::attachments() const noexcept { return *data_->attachments; }
const vehicle_startup::TiedCinWitnessRoster& EnvelopeOwnerSource::witnesses() const noexcept { return *data_->witnesses; }
const tl::fea::type45::Model& EnvelopeOwnerSource::joints() const noexcept { return data_->joints; }
const modelio::type45::VehicleType45Source& EnvelopeOwnerSource::joint_source() const noexcept { return data_->joint_source; }
const std::vector<std::uint32_t>& EnvelopeOwnerSource::joint_source_rows() const noexcept { return data_->joint_rows; }
const vehicle_runtime::SourceRoles& EnvelopeOwnerSource::roles() const noexcept { return data_->roles; }
const tl::fea::ShellBatchStartup& EnvelopeOwnerSource::startup() const noexcept { return data_->execution.mechanical().wall().startup(); }
const EnvelopeOwnerForecast& EnvelopeOwnerSource::forecast() const noexcept { return data_->forecast; }
tl::fea::NodalCinWitnessSource EnvelopeOwnerSource::witness_source() const noexcept {
    const auto& values=witnesses().data();
    return {&attachments().model(),values.ranges.data(),values.witnesses.data(),values.ranges.size(),values.witnesses.size()};
}
std::size_t EnvelopeOwnerSource::retained_host_upper_bound(std::size_t cap) const {
    const auto& roster=witnesses().data();
    // PreparePhysical copied these exact shared handles. Their source backing
    // is already retained by execution/attachments below; only its owning
    // object and actual roster vectors are additional.
    d::Require(&witnesses().shells()==physical().shells() &&
        witnesses().model().rows().data==attachments().model().rows().data,
        "Retained witness accounting requires exact shared physical/CIN backing");
    tl::util::BoundedArenaLayout bytes(cap);tl::util::ArenaRegion unused;
    for(const auto count:{execution_source().retained_host_upper_bound(cap),
            attachments().forecast().total_host_bytes,witnesses().forecast().fixed_bytes,
            joint_source().forecast().total_bytes,joints().owned_payload_bytes(),
            sizeof(EnvelopeOwnerSource)+sizeof(Data)+std::size_t{128}})
        d::Require(bytes.Append<std::byte>(count,unused),"Retained combined owner source exceeds cap");
    d::Require(bytes.Append<vehicle_startup::cin_stage::WitnessRange>(roster.ranges.capacity(),unused) &&
        bytes.Append<vehicle_startup::cin_stage::ActiveWitness>(roster.witnesses.capacity(),unused) &&
        bytes.Append<vehicle_startup::TiedCinWitnessOrigin>(roster.origins.capacity(),unused) &&
        bytes.Append<std::uint32_t>(joint_source_rows().capacity(),unused) &&
        bytes.Append<std::uint8_t>(roles().node.capacity(),unused),"Retained combined source vector capacity exceeds cap");
    d::Require(bytes.bytes()<=forecast().current_phase,"Retained source exceeds its original complete construction bound");
    return bytes.bytes();
}
vehicle_runtime::detail::OwnerPacking EnvelopeOwnerSource::PackOwner(std::size_t cap) const {
    const auto& source=data_->execution.mechanical();
    auto packed=vehicle_runtime::detail::PackOwner(source.coefficients(),source.rigid_assembly(),data_->roles,
        startup().uniform_velocity,cap);
    vehicle_runtime::detail::ApplyConstrainedStartup(packed,startup(),source.wall().translation_fixed_bits(),
        source.wall().rotation_fixed());
    return packed;
}
}
