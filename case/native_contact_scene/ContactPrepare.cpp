#include "ContactStorage.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::native_scene::contact_detail {
using output::Require;
ContactPlan PlanContact(const PhysicalSource& physical,ContactIdentity identity,ContactLimits limits,
    MainMotion motion,std::size_t object_bytes) {
    const auto& d=physical.declared().data();
    const bool fixed=motion==MainMotion::FixedWall;
    Require((fixed&&d.contact_surface==modelio::native_scene::DeclaredContactSurface::FixedWall)||
        (motion==MainMotion::MovingShells&&d.contact_surface==modelio::native_scene::DeclaredContactSurface::AllShells),
        "Declared contact surface differs from the selected fixed/moving factory");
    ContactPlan plan;plan.motion=motion;plan.nodes=d.nodes.size();plan.shells=d.wall.size()+d.patch.size();
    plan.primary=fixed?d.wall.size():plan.shells;
    const auto nodes=plan.nodes,primary=plan.primary,shells=plan.shells;
    Require(identity.source&&identity.topology&&identity.generation&&nodes&&primary&&!d.wall.empty()&&!d.patch.empty()&&
        limits.nodes&&limits.nodes<=2048&&limits.physical_shells&&limits.physical_shells<=1024&&
        nodes<=limits.nodes&&shells<=limits.physical_shells&&limits.host_bytes&&limits.host_bytes<=128u<<20&&
        limits.scratch_bytes&&limits.scratch_bytes<=64u<<20&&
        (limits.preprocessing==n::search_startup::Initialization::SerialNative||
         limits.preprocessing==n::search_startup::Initialization::InvariantNoExpansion),
        "Declared contact source has invalid identity/profile/count or byte limits");
    plan.topology_limits={limits.nodes,limits.physical_shells,limits.host_bytes,limits.scratch_bytes};
    auto& search_limits=plan.search_limits;search_limits.max_nodes=limits.nodes;
    search_limits.max_mains=2*limits.physical_shells;search_limits.max_secondaries=limits.nodes;
    search_limits.max_output_bytes=limits.host_bytes;search_limits.max_scratch_bytes=limits.scratch_bytes;
    plan.topology=n::startup::Preflight(nodes,primary,plan.topology_limits);
    plan.search=n::search_startup::Preflight(nodes,primary,nodes,search_limits);
    Require(plan.topology.status==n::startup::Status::Ok&&plan.search.status==n::search_startup::Status::Ok,
        "Contact topology/search source preflight rejected");
    plan.rows=PlanRows(nodes,shells,primary,plan.topology.maximum_references,limits.host_bytes);
    tl::util::BoundedArenaLayout budget(limits.host_bytes);tl::util::ArenaRegion unused;
    Require(object_bytes<=SIZE_MAX-8192&&budget.Append<std::byte>(object_bytes+8192,unused)&&
        budget.Append<std::byte>(plan.rows.bytes,unused)&&budget.Append<std::byte>(plan.topology.output_bytes,unused)&&
        (!fixed||budget.Append<std::byte>(plan.topology.ready_output_bytes,unused))&&
        budget.Append<std::byte>(plan.search.output_bytes,unused),"Complete retained contact-source buffers exceed cap");
    plan.retained_bytes=budget.bytes();return plan;
}
void BuildContact(ContactStorage& out,n::ContactSourceInput& source,ContactIdentity identity,
    ContactLimits limits,const ContactPlan& plan) {
    const bool fixed=plan.motion==MainMotion::FixedWall;
    const auto nodes=plan.nodes,primary=plan.primary,shells=plan.shells;
    const auto& top=plan.topology;const auto& search=plan.search;
    out.preprocessing=limits.preprocessing;
    Require(out.rows.Initialize(plan.rows.bytes),"Contact-source input allocation failed");
    out.fields=Construct(out.rows,plan.rows);auto& rows=out.fields;
    FillDeclaredInputs(out.physical,rows,plan.motion);
    n::source_shells::Limits shell_limits{limits.nodes,limits.physical_shells,limits.physical_shells,limits.nodes,limits.scratch_bytes};
    const n::source_shells::Input shell_input{ShellProfile(),nodes,rows.shells,shells,rows.primary_shells,primary,rows.secondary_input,nodes};
    n::source_shells::Forecast shell_forecast;
    Require(n::source_shells::Preflight(shell_input,shell_limits,shell_forecast).status==n::source_shells::Status::Ok,
        "Complete native shell coefficient source rejected");
    out.forecast.owned_bytes=plan.retained_bytes;
    out.forecast.startup_scratch_bytes=std::max({shell_forecast.scratch_bytes,top.scratch_bytes,
        fixed?top.ready_scratch_bytes:std::size_t{0},search.scratch_bytes});
    tl::util::BoundedArenaLayout budget(limits.host_bytes);tl::util::ArenaRegion unused;
    Require(out.forecast.startup_scratch_bytes<=limits.scratch_bytes&&
        budget.Append<std::byte>(plan.retained_bytes,unused)&&budget.Append<std::byte>(out.forecast.startup_scratch_bytes,unused),
        "Complete contact-source startup peak exceeds cap");
    out.forecast.peak_bytes=budget.bytes();
    tl::util::HostArena scratch;
    Require(scratch.Initialize(out.forecast.startup_scratch_bytes)&&out.topology_arena.Initialize(top.output_bytes)&&
        (!fixed||out.ready_arena.Initialize(top.ready_output_bytes))&&out.search_arena.Initialize(search.output_bytes),
        "Contact producer startup allocation failed");
    Require(n::source_shells::Build(shell_input,shell_limits,scratch.data(),scratch.bytes(),
        {rows.node_fields,nodes,rows.primary_k,primary,rows.secondary_fields,nodes}).status==n::source_shells::Status::Ok,
        "Native whole-shell coefficient/gap source rejected");
    n::startup::Input mesh;
    mesh.profile=fixed?n::startup::Profile::OrdinaryExteriorFixedMain:n::startup::Profile::OrdinaryExteriorMovingMain;
    mesh.node_source_ids=rows.ids;mesh.node_count=nodes;mesh.positions={rows.positions,std::uint32_t(nodes),3,1};
    mesh.primary=rows.primary;mesh.primary_count=primary;mesh.coordinates=n::startup::Coordinates::Native;
    mesh.source_generation=identity.generation;
    Require(n::startup::BuildStarter(mesh,plan.topology_limits,out.topology_arena,scratch,&out.topology).status==n::startup::Status::Ok,
        "Native main topology/Starter normal source rejected");
    Require(out.topology.main_count==2*primary,"Native topology expanded source is incomplete");
    for(std::size_t i=0;i<out.topology.main_count;++i) {
        const auto parent=out.topology.expanded_to_primary[i];Require(parent<primary,"Native expanded-to-primary source differs");
        rows.expanded_k[i]=rows.primary_k[parent];n::source_shells::MainGapFields gap;
        Require(n::source_shells::MainGaps(rows.node_fields,nodes,out.topology.mains[i].nodes,&gap).status==n::source_shells::Status::Ok,
            "Native expanded main gap source rejected");rows.expanded_gaps[i]=gap.maximum;
    }
    n::startup::NormalView normals=out.topology.starter;
    if(fixed) {
        Require(n::startup::BuildFixedMain(mesh,out.topology,{rows.expanded_k,2*primary},plan.topology_limits,
            out.ready_arena,scratch,&out.ready).status==n::startup::Status::Ok,"Native fixed-main ready normal source rejected");
        normals=out.ready.normals;
    }
    for(std::size_t i=0;i<nodes;++i)rows.search_secondary[i]={rows.secondary_nodes[i],rows.secondary_fields[i].stiffness,rows.secondary_fields[i].gap};
    n::search_startup::Input search_input;search_input.mesh=mesh;search_input.topology=out.topology;
    search_input.contributors={n::search_startup::Census::CompleteDeclaredModel,nodes,shells,0,0,0,0,0};
    search_input.profile=SearchProfile(limits.preprocessing);search_input.secondary=rows.search_secondary;
    search_input.secondary_count=nodes;search_input.main_gaps=rows.expanded_gaps;search_input.main_count=2*primary;
    Require(n::search_startup::Build(search_input,plan.search_limits,out.search_arena,scratch,&out.search).status==n::search_startup::Status::Ok,
        "Native margin/removal/initial-contact source rejected");
    FillRuntimeRows(out.physical,rows,out.topology,normals,out.search);
    source.source_id=identity.source;source.topology_generation=identity.topology;
    source.primary_main_count=primary;source.primary_parent_ids=rows.parent_ids;source.primary_curvature=out.search.primary_extent;
    source.margin=out.search.margin;source.gap_load=0;source.drad=0;source.force_packet_size=128;source.native_workers=1;
    const auto references=normals.reference_count;
    source.selection={rows.nodes,nodes,rows.mains,2*primary,rows.secondary,nodes,rows.normals,references,
        {out.topology.normal_offsets,references+1,out.topology.normal_mains,out.topology.normal_incidence_count},
        {out.search.secondary_offsets,nodes+1,out.search.removal_count?out.search.removed_mains:nullptr,out.search.removal_count},identity.generation};
    out.config=RuntimeProfile();
}
}
