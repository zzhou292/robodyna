#include "Mapping.h"
#include "lib_utils/BoundedArena.h"
#include <map>
#include <optional>
#include "case/vehicle_wall/native/EnvelopeOwnerSource.h"
namespace crash::output::physical_frames {
struct Mapping::Data {
    Data(const Execution* old,const cases::vehicle_runtime::Source* current,source::PreparedSourceMapping m)
        :mapping(std::move(m)) {
        if(old)execution.emplace(*old);else runtime_source.emplace(*current);
    }
    std::optional<Execution> execution;
    std::optional<cases::vehicle_runtime::Source> runtime_source;
    source::PreparedSourceMapping mapping;
    std::vector<std::uint32_t> nodes;
    std::vector<ParentField> parents;
    FamilyCounts physical_counts,render_counts;
    std::size_t bytes=0;
};
Mapping Mapping::Prepare(const Execution& execution,std::size_t cap) {
    return PrepareImpl(&execution,nullptr,cap);
}
Mapping Mapping::Prepare(const cases::vehicle_runtime::Source& source,std::size_t cap) {
    return PrepareImpl(nullptr,&source,cap);
}
Mapping Mapping::PrepareImpl(const Execution* old,const cases::vehicle_runtime::Source* current,std::size_t cap) {
    Require(bool(old)!=bool(current),"Accepted mapping needs exactly one genuine source handle");
    const auto* owner=current?current->environment():nullptr;
    const auto* environment=owner?&owner->execution_source().mechanical():nullptr;
    const auto& physical=current?current->physical():old->physical();
    const auto& references=current?current->vehicle_references():old->model().shell_source().references();
    Require(references.resolution(),"Accepted mapping lacks original vehicle resolution");
    const auto& resolution=*references.resolution();
    const auto& native=*physical.execution();
    const auto& canonical=resolution.source().canonical();
    const auto& canonical_nodes=resolution.source().canonical_nodes();
    const auto* map=physical.mapping();
    const auto render_parents=resolution.parents().size();
    Require(cap && cap<=512u<<20 && native.prepared() && map &&
        map->Matches(*physical.shells(),*physical.domain()) &&
        canonical_nodes.size()+(environment?4u:0u)==map->shell_node_count() &&
        native.parents().size()==render_parents+(environment?1u:0u),
        "Physical accepted mapping has incomplete source scope");
    const auto& rc=resolution.native_counts();
    const FamilyCounts actual{physical.shells()->qeph_count(),physical.shells()->t3_count(),physical.shells()->qbat_count()};
    Require(actual.qeph==rc.qeph+(environment?1u:0u) && actual.t3==rc.t3 && actual.qbat==rc.qbat,
        "Physical accepted mapping has an undeclared family suffix");
    if(environment) {
        const auto& embedding=environment->embedding();
        const auto& wall=environment->wall();
        const auto& parent=environment->environment_parent();
        Require(embedding.domain().SharesStorage(*physical.domain()) && embedding.suffix().size()==4 &&
            embedding.original().node_count()+4==physical.domain()->node_count() &&
            current->vehicle_nodes()==embedding.original().node_count() &&
            &embedding.source().tied_source().canonical().data()==&canonical.data() &&
            parent.qeph_index==rc.qeph && parent.catalog_append_ordinal==render_parents,
            "Combined capture lacks the genuine original-domain prefix/wall suffix certificate");
        const auto& role=native.parents()[render_parents];
        Require(role.source.source_parent_id==wall.ids().shell && role.source.source_part_id==wall.ids().part &&
            role.source.material_id==wall.ids().material && role.source.section_id==wall.ids().section &&
            role.source.family==tl::fea::ShellBindingFamily::Qeph && role.source.family_index==rc.qeph &&
            role.law==tl::fea::ShellSectionLaw::GlobalLaw1Npt0 && role.material_points==0 &&
            physical.failure()->parent(render_parents)->policy==tl::fea::ShellFailurePolicy::None,
            "Declared fixed wall execution/failure row differs from its actual source");
        for(unsigned k=0;k<4;++k) {
            Require(parent.domain_nodes[k]==embedding.original().node_count()+k &&
                parent.shell_nodes[k]==canonical_nodes.size()+k && map->owner_index(parent.shell_nodes[k])==parent.domain_nodes[k] &&
                embedding.suffix()[k].source_id==wall.ids().nodes[k] && current->translation_fixed(parent.domain_nodes[k])==7 &&
                current->rotation_fixed(parent.domain_nodes[k])==1,
                "Declared wall source nodes/mapping/fixed roles differ");
        }
    }
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion unused;
    Require(budget.Append<std::byte>(sizeof(Data)+8192,unused) &&
        budget.Append<std::byte>(canonical.data().limits.host_bytes,unused) &&
        budget.Append<std::uint64_t>(canonical.data().canonical_nodes,unused) &&
        budget.Append<std::uint32_t>(canonical_nodes.size(),unused) &&
        budget.Append<ParentField>(render_parents,unused) &&
        budget.Append<source::NativeParent>(render_parents,unused) &&
        budget.Append<std::byte>(resolution.parts().size()*(3*sizeof(source::MappingExecutionPart)+128),unused),
        "Accepted mapping scratch exceeds cap");
    source::MappingExecution provenance;
    std::map<std::uint64_t,source::MappingExecutionPart> resolved;
    const auto& policy=environment?owner->execution_source().law1_policy():
        (old?old->law1_policy():current->original_execution().law1_policy());
    if(policy.requires_ordinary_explicit_defaults()) {
        provenance.profile=source::MappingExecutionProfile::NativeA62OrdinaryLaw1;
        provenance.coefficient_working_length_m=policy.coefficient_working_length_m();
        provenance.projection_working_length_m=references.qeph_metric().working_length_m();
    }
    std::vector<source::NativeParent> rows;
    rows.reserve(render_parents);
    std::vector<ParentField> fields;
    fields.reserve(render_parents);
    for(std::size_t i=0;i<render_parents;++i) {
        const auto& role=native.parents()[i];
        const auto& raw=resolution.parents()[i];
        const auto* mapping=resolution.native_mapping(i);
        Require(mapping && mapping->family==role.source.family && mapping->family_index==role.source.family_index &&
            mapping->family_index<=UINT32_MAX && raw.source_parent_id==role.source.source_parent_id,
            "Accepted parent/source family identity differs");
        const auto family=Family(mapping->family);
        if(role.law==tl::fea::ShellSectionLaw::GlobalLaw1Npt0) {
            const auto driver=resolution.source().law1_driver(raw.part_index);
            Require(policy.requires_ordinary_explicit_defaults() && driver.available() &&
                driver.material_id()==role.source.material_id && driver.section_id()==role.source.section_id &&
                role.material_points==0,"Accepted global LAW1 lacks authenticated execution provenance");
            auto [at,inserted]=resolved.try_emplace(role.source.source_part_id,source::MappingExecutionPart{
                role.source.source_part_id,role.source.material_id,role.source.section_id,0,0});
            Require(at->second.material==role.source.material_id && at->second.section==role.source.section_id,
                "Accepted global LAW1 PID has conflicting source declarations");
            if(family==QephFamily)++at->second.qeph;else if(family==T3Family)++at->second.t3;
            else Require(false,"Global LAW1 cannot replace QBAT execution");
        }
        rows.push_back({raw.canonical_parent,family,static_cast<std::uint32_t>(mapping->family_index),
                        role.material_points,Plasticity(role.law)});
        fields.push_back({family,static_cast<std::uint32_t>(mapping->family_index),role.law});
    }
    for(const auto& part:resolved)provenance.parts.push_back(part.second);
    const auto* metadata=policy.requires_ordinary_explicit_defaults()?&provenance:nullptr;
    auto prepared=source::PreparedSourceMapping::Prepare(canonical,
        {canonical_nodes.data(),canonical_nodes.size(),rows.data(),rows.size(),metadata});
    // The source mapping factory already enforces its complete child reservation
    // charged above; its temporary arrays retire before the node-ID decode.
    auto next=std::make_shared<Data>(old,current,std::move(prepared));
    next->parents=std::move(fields);
    next->physical_counts=actual;next->render_counts={rc.qeph,rc.t3,rc.qbat};
    next->nodes.reserve(canonical_nodes.size());
    const auto& ids=source::FindArray(canonical.data(),"node_ids");
    const auto original=arrays::Decode<std::uint64_t>(ids.descriptor,ids.bytes);
    const auto shell_nodes=physical.shells()->active_nodes();
    for(std::size_t i=0;i<canonical_nodes.size();++i) {
        const auto physical_node=map->owner_index(i);
        Require(canonical_nodes[i]<original.size() && physical_node<map->owner_node_count() && physical_node<=UINT32_MAX &&
            original[canonical_nodes[i]]==shell_nodes[i].source_id &&
            physical.domain()->nodes()[physical_node].source_id==shell_nodes[i].source_id,
            "Original render node and physical source node identity differ");
        if(environment)Require(physical_node<environment->embedding().original().node_count(),
            "Vehicle render mapping selects an environment suffix node");
        next->nodes.push_back(static_cast<std::uint32_t>(physical_node));
    }
    next->bytes=sizeof(Data)+next->nodes.capacity()*sizeof(std::uint32_t)+
        next->parents.capacity()*sizeof(ParentField)+next->mapping.payload_bytes()+
        next->mapping.parents().capacity()*sizeof(records::ParentPoints)+8192;
    Require(next->bytes<=cap,"Retained accepted source mapping exceeds cap");
    return Mapping(std::move(next));
}
const Execution& Mapping::execution() const {
    return data_->execution?*data_->execution:data_->runtime_source->original_execution();
}
const tl::fea::ShellPhysicalBinding& Mapping::physical() const noexcept {
    return data_->execution?data_->execution->physical():data_->runtime_source->physical();
}
const cases::vehicle_startup::VehicleShellReferences& Mapping::vehicle_references() const noexcept {
    return data_->execution?data_->execution->model().shell_source().references():data_->runtime_source->vehicle_references();
}
const tl::fea::beam18::Model* Mapping::structural_beams() const noexcept {
    return data_->execution?data_->execution->model().structural_beams():data_->runtime_source->structural_beams();
}
const cases::vehicle_wall::native::EnvelopePhysicalSource* Mapping::environment() const noexcept {
    const auto* source=data_->runtime_source?data_->runtime_source->environment():nullptr;
    return source?&source->execution_source().mechanical():nullptr;
}
FamilyCounts Mapping::physical_counts() const noexcept {return data_->physical_counts;}
FamilyCounts Mapping::render_counts() const noexcept {return data_->render_counts;}
const source::PreparedSourceMapping& Mapping::source_mapping() const noexcept {return data_->mapping;}
const std::vector<std::uint32_t>& Mapping::physical_nodes() const noexcept {return data_->nodes;}
const std::vector<ParentField>& Mapping::parents() const noexcept {return data_->parents;}
std::size_t Mapping::physical_node_count() const noexcept {return physical().domain()->node_count();}
std::size_t Mapping::payload_bytes() const noexcept {return data_->bytes;}
} // namespace crash::output::physical_frames
