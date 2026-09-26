#include "PhysicalSource.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/constraints/NodalRigidPartTopology.h"
#include "lib_src/constraints/NodalRigidPartAssemblyModel.h"
#include "case/vehicle_startup/TiedCinWitnessRoster.h"
namespace crash::cases::native_scene {
namespace fe=tl::fea;
namespace source=modelio::native_scene;
using output::Require;
struct PhysicalSource::Data {
    explicit Data(const source::DeclaredSource& input):declared(input){}
    source::DeclaredSource declared;
    source::LinearHardeningBridge hardening;
    fe::ShellBatchBinding shells;
    fe::ShellBatchPlasticityBinding catalog;
    fe::ShellBatchFailureBinding failure;
    fe::NodalNodeDomain domain;
    fe::ShellNodeMap mapping;
    fe::NodalCoefficientLedger ledger;
    fe::rigid::NodalRigidPartTopology part_topology;
    fe::rigid::NodalRigidPartAssemblyModel parts;
    fe::NodalRigidAssemblyBinding rigid;
    fe::ShellExecutionBinding execution;
    fe::ShellPhysicalBinding physical;
    tl::constraints::tied_shell::TiedCinAttachmentModel cin;
    std::optional<TiedSource> tied;
    std::optional<vehicle_startup::TiedCinWitnessRoster> witnesses;
    fe::ShellBatchStartup startup;
    std::vector<std::uint8_t> fixed,rotation;
};
PhysicalSource PhysicalSource::Prepare(const source::DeclaredSource& declared,std::uint64_t instance,SourceLimits limits) {
    const auto& d=declared.data();
    Require(instance&&limits.nodes&&limits.nodes<=2048&&limits.parents&&limits.parents<=1024&&
        d.nodes.size()<=limits.nodes&&d.wall.size()<=limits.parents&&d.patch.size()<=limits.parents-d.wall.size(),
        "Declared shell scene exceeds explicit host source counts");
    // Reuse the existing source-unit scalar expressions. This is a native
    // LAW44 source; no fictitious MAT024 keyword/card is constructed.
    const modelio::assembly::SourceUnits units{1000,.001,1};
    const double stress=modelio::assembly::reader::MaterialStressScale(units);
    const double density_scale=modelio::assembly::reader::MaterialDensityScale(units);
    const auto& m=d.material;
    const double young=m.young_n_mm2*stress,density=m.density_tonne_mm3*density_scale;
    auto out=std::make_shared<Data>(declared);
    const tl::material::TabulatedShellPlasticityRate rate{true,m.rate_c_per_s,m.rate_p,m.rate_filter_hz};
    out->hardening=source::PrepareNativeHardening(young,m.poisson,density,m.yield_n_mm2*stress,
        m.plastic_hardening_n_mm2*stress,rate);
    const auto position=[&](std::uint32_t node) {
        const auto x=d.nodes.at(node).xyz_mm;
        return tl::math::Vec3{x.x*units.length_to_m,x.y*units.length_to_m,x.z*units.length_to_m};
    };
    std::vector<fe::ShellQephBindingInput> quads;quads.reserve(d.patch.size());
    std::vector<fe::ShellT3BindingInput> triangles;triangles.reserve(d.wall.size());
    std::vector<fe::ShellPlasticityParentInput> parents;parents.reserve(d.patch.size()+d.wall.size());
    for(const auto& p:d.patch) {
        fe::ShellQephBindingInput q;q.source_parent_id=p.id;
        q.reference.density=density;q.reference.young_modulus=young;q.reference.poisson_ratio=m.poisson;
        q.reference.projection_working_length_m=units.length_to_m;
        q.reference.thickness=d.thickness_mm*units.length_to_m;q.reference.placement=fe::ShellReferencePlacement::Centered;
        for(unsigned k=0;k<4;++k){q.nodes[k]=p.nodes[k];q.reference.node_ids[k]=std::uint32_t(d.nodes[p.nodes[k]].id);q.reference.position[k]=position(p.nodes[k]);}
        parents.push_back({fe::ShellBindingFamily::Qeph,quads.size(),p.id,p.part,d.rigid_patch?2u:1u,d.rigid_patch?2u:1u});quads.push_back(q);
    }
    for(const auto& p:d.wall) {
        fe::ShellT3BindingInput t;t.source_parent_id=p.id;
        t.reference.density=density;t.reference.young_modulus=young;t.reference.poisson_ratio=m.poisson;
        t.reference.thickness=d.thickness_mm*units.length_to_m;t.reference.placement=fe::ShellReferencePlacement::Centered;
        for(unsigned k=0;k<3;++k){t.nodes[k]=p.nodes[k];t.reference.node_ids[k]=d.nodes[p.nodes[k]].id;t.reference.position[k]=position(p.nodes[k]);}
        parents.push_back({fe::ShellBindingFamily::T3,triangles.size(),p.id,p.part,1,1});triangles.push_back(t);
    }
    // Native source census is Q4 then T3; IDs preserve the original exported
    // wall/patch order independently of this qualified family execution order.
    Require(out->shells.Initialize({quads.data(),triangles.data(),quads.size(),triangles.size(),d.nodes.size()}).status==fe::ShellBindingStatus::Success,
        "Declared scene native reference geometry rejected");
    fe::ShellPlasticityMaterialInput material;
    material.material_id=1;material.young_pa=young;material.poisson_ratio=m.poisson;material.density_kg_m3=density;
    material.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    material.linear={m.yield_n_mm2*stress,out->hardening.derived_etan_pa};material.rate=rate;
    material.law=fe::ShellSectionLaw::LayeredLaw44Nip3;
    const fe::ShellPlasticitySectionInput section{1,d.thickness_mm*units.length_to_m,3,fe::ShellSectionFormulation::LayeredNip3};
    std::vector<fe::ShellPlasticityMaterialInput> materials{material};
    std::vector<fe::ShellPlasticitySectionInput> sections{section};
    if(d.rigid_patch) {
        // Internal execution key2 represents authentic covered-shell OFF=-1.
        // The raw source material remains the single LAW44 declaration above.
        // Reference E/rho/nu/thickness still produce physical M/J and contact K.
        fe::ShellPlasticityMaterialInput skin;
        skin.material_id=2;skin.young_pa=young;skin.poisson_ratio=m.poisson;skin.density_kg_m3=density;
        skin.law=fe::ShellSectionLaw::RigidSkin;
        materials.push_back(skin);
        sections.push_back({2,d.thickness_mm*units.length_to_m,0,fe::ShellSectionFormulation::Nonconstitutive});
    }
    const fe::ShellBatchPlasticityBindingInput catalog{nullptr,materials.data(),sections.data(),parents.data(),0,materials.size(),sections.size(),parents.size()};
    Require(out->catalog.InitializeExecutionCatalog(out->shells,catalog).status==fe::ShellPlasticityBindingStatus::Success,
        "Declared native LAW44 execution catalog rejected");
    for(const auto& parent:parents) {
        if(d.rigid_patch&&parent.family==fe::ShellBindingFamily::Qeph) {
            fe::ShellSectionLaw law;unsigned points=99;
            Require(out->catalog.Law(parent.family,parent.family_index,&law)&&law==fe::ShellSectionLaw::RigidSkin&&
                out->catalog.MaterialPointCount(parent.family,parent.family_index,&points)&&points==0,
                "Covered rigid patch has a constitutive execution point");
            continue;
        }
        fe::sections::PointParameters prepared;
        Require(out->catalog.Parameters(parent.family,parent.family_index,&prepared)&&
            output::Bits(prepared.plastic_hardening_pa)==output::Bits(out->hardening.prepared_h_pa),
            "Actual prepared catalog H differs from the checked source bridge");
    }
    std::vector<fe::ShellFailureParentInput> failure;
    for(const auto& parent:parents){fe::ShellFailureParentInput f;f.source=parent;f.policy=fe::ShellFailurePolicy::None;failure.push_back(f);}
    Require(out->failure.InitializeExecution(out->catalog,failure.data(),failure.size()).status==fe::ShellPlasticityBindingStatus::Success,
        "Declared no-failure scope rejected");
    std::vector<fe::NodalDomainNode> nodes;nodes.reserve(d.nodes.size());
    for(std::size_t i=0;i<d.nodes.size();++i)nodes.push_back({d.nodes[i].id,position(std::uint32_t(i))});
    Require(bool(out->domain.Initialize({instance,nodes.data(),nodes.size()})),"Declared scene physical domain rejected");
    Require(bool(out->mapping.Initialize(out->shells,out->domain)),"Declared scene shell mapping rejected");
    Require(bool(out->ledger.Initialize({&out->mapping,nullptr,nullptr}))&&!out->ledger.scope().uncovered_nodes,
        "Declared scene lacks complete physical M/J");
    if(d.rigid_patch) {
        const auto& declared_part=*d.rigid_patch;
        const fe::rigid::PartTopologyPartInput part{declared_part.source_part_id,
            declared_part.member_source_ids.data(),declared_part.member_source_ids.size()};
        fe::rigid::PartTopologyInput topology;
        topology.source_instance_id=instance;topology.parts=&part;topology.part_count=1;
        topology.expected_members=part.nodes;topology.expected_member_count=part.node_count;
        Require(bool(out->part_topology.Initialize(topology)),"Declared rigid patch topology rejected");
        Require(bool(out->parts.Initialize(out->part_topology,out->ledger,{units.mass_to_kg,units.length_to_m})),
            "Source-derived rigid PART aggregate rejected");
        Require(bool(out->rigid.Initialize(out->parts)),"Declared rigid patch binding rejected");
    } else Require(bool(out->rigid.InitializeEmpty(out->ledger)),"Declared empty rigid scope rejected");
    Require(out->execution.Initialize(out->catalog,out->ledger,out->rigid).status==fe::ShellPlasticityBindingStatus::Success,
        "Declared constitutive execution roles rejected");
    Require(bool(out->physical.InitializeExecution({&out->shells,&out->catalog,&out->failure,nullptr},out->ledger,out->execution)),
        "Declared scene physical binding rejected");
    if(d.tied_patch) {
        out->tied=TiedSource::Prepare(declared,out->domain);
        out->cin=out->tied->model();
        out->witnesses.emplace(vehicle_startup::TiedCinWitnessRoster::PreparePhysical(out->cin,out->physical,{8u<<20,4}));
        Require(out->witnesses->runtime_mappable()&&out->witnesses->data().counts.declared_parent_witnesses==4&&
            out->witnesses->data().counts.additional_containing_parents==0,
            "Named source does not have four genuine declared-master witnesses");
    } else Require(bool(tl::constraints::tied_shell::PrepareEmptyCinAttachments(out->domain,&out->cin)),"Declared empty CIN scope rejected");
    out->fixed.assign(nodes.size(),0);out->rotation.assign(nodes.size(),0);
    for(auto node:d.wall_nodes){out->fixed[node]=7;out->rotation[node]=1;}
    out->startup={fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,
        {d.velocity_mm_s.x*units.length_to_m/units.time_to_s,d.velocity_mm_s.y*units.length_to_m/units.time_to_s,d.velocity_mm_s.z*units.length_to_m/units.time_to_s}};
    return PhysicalSource(std::move(out));
}
const source::DeclaredSource& PhysicalSource::declared() const noexcept{return data_->declared;}
const source::LinearHardeningBridge& PhysicalSource::hardening() const noexcept{return data_->hardening;}
const fe::ShellPhysicalBinding& PhysicalSource::physical() const noexcept{return data_->physical;}
const fe::NodalRigidAssemblyBinding& PhysicalSource::rigid() const noexcept{return data_->rigid;}
const tl::constraints::tied_shell::TiedCinAttachmentModel& PhysicalSource::cin() const noexcept{return data_->cin;}
const TiedSource* PhysicalSource::tied_source() const noexcept{return data_->tied?&*data_->tied:nullptr;}
const vehicle_startup::TiedCinWitnessRoster* PhysicalSource::cin_witnesses() const noexcept{return data_->witnesses?&*data_->witnesses:nullptr;}
const fe::ShellBatchStartup& PhysicalSource::startup() const noexcept{return data_->startup;}
const std::vector<std::uint8_t>& PhysicalSource::translation_fixed_bits() const noexcept{return data_->fixed;}
const std::vector<std::uint8_t>& PhysicalSource::rotation_fixed() const noexcept{return data_->rotation;}
std::size_t PhysicalSource::retained_host_upper_bound() const {
    const auto& d=data_->declared.data();tl::util::BoundedArenaLayout b(SIZE_MAX);tl::util::ArenaRegion unused;
    Require(b.Append<std::byte>(sizeof(Data)+sizeof(source::DeclaredData)+8192,unused)&&
        b.Append<std::byte>(data_->physical.owned_payload_bytes(),unused)&&
        b.Append<std::byte>(data_->part_topology.owned_payload_bytes(),unused)&&
        b.Append<std::byte>(data_->cin.forecast().model_payload_bytes,unused)&&
        b.Append<source::SourceNode>(d.nodes.capacity(),unused)&&
        b.Append<source::SourceParent>(d.wall.capacity()+d.patch.capacity(),unused)&&
        b.Append<std::uint32_t>(d.wall_nodes.capacity()+d.patch_nodes.capacity(),unused)&&
        b.Append<std::uint8_t>(data_->fixed.capacity()+data_->rotation.capacity(),unused)&&
        b.Append<char>(d.export_sha256.capacity()+d.definition_sha256.capacity()+d.definition_bytes.capacity()+d.definition_schema.capacity()+4,unused),
        "Physical source retained byte bound overflow");
    if(d.rigid_patch)Require(b.Append<std::uint64_t>(d.rigid_patch->member_source_ids.capacity()+
        d.rigid_patch->centroid_source_order.capacity(),unused),"Rigid declaration byte bound overflow");
    if(data_->tied)Require(b.Append<std::byte>(data_->tied->retained_host_upper_bound(),unused),
        "Complete tied source retained byte bound overflow");
    if(data_->witnesses)Require(b.Append<std::byte>(data_->witnesses->forecast().total_host_bytes,unused),
        "Complete tied witness retained byte bound overflow");
    return b.bytes();
}
} // namespace crash::cases::native_scene
