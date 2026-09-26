#include "ContactBuild.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <climits>
namespace crash::cases::native_scene::contact_detail {
using output::Require;
void FillDeclaredInputs(const PhysicalSource& physical,Rows r,MainMotion motion) {
    const auto& d=physical.declared().data();const auto count=d.nodes.size();
    const auto& rigid=physical.rigid();
    const bool rigid_scope=d.rigid_patch?
        (rigid.prepared()&&!rigid.explicitly_empty()&&rigid.parts()&&rigid.groups().size()==1&&
         rigid.members().size()==d.rigid_patch->member_source_ids.size()&&
         rigid.groups()[0].source_kind==tl::fea::RigidBindingSourceKind::Part&&
         rigid.groups()[0].source_id==d.rigid_patch->source_part_id):rigid.explicitly_empty();
    Require(rigid_scope&&physical.cin().explicitly_empty()&&
        physical.translation_fixed_bits().size()==count&&physical.rotation_fixed().size()==count,
        "Contact source differs from its declared rigid/empty-CIN physical scope");
    for(std::size_t i=0;i<count;++i) {
        const auto& node=d.nodes[i];Require(node.id<=INT_MAX,"Native positive source NID exceeds signed reference domain");r.ids[i]=node.id;
        r.positions[3*i]=node.xyz_mm.x;r.positions[3*i+1]=node.xyz_mm.y;r.positions[3*i+2]=node.xyz_mm.z;
        const auto bits=physical.translation_fixed_bits()[i];
        Require((bits==0||bits==7)&&physical.rotation_fixed()[i]==std::uint8_t(bits==7),
            "Declared contact source has another constraint profile");
        // Global BCS input system0 maps to native identity skew slot1;
        // untouched nodes are changed from -1 to0 by PRINTBCS.
        r.nodes[i]={node.id,int(bits),bits?1:0};
    }
    std::size_t ordinal=0;
    for(const auto* family:{&d.patch,&d.wall})for(const auto& p:*family) {
        auto& shell=r.shells[ordinal++];shell.source_element_id=p.id;
        shell.layout=p.corners==4?n::ShellLayout::Quad4:n::ShellLayout::Triangle3;
        for(unsigned k=0;k<4;++k)shell.nodes[k]=p.nodes[k];
        shell.young=d.material.young_n_mm2;shell.structural_thickness=d.thickness_mm;
        shell.property_thickness=d.thickness_mm; // No element/part contact override in this compiler profile.
    }
    std::size_t secondary=0;
    const auto append=[&](std::uint32_t node){Require(node<count,"Secondary source-node index is outside domain");
        if(!r.selected[node]){r.selected[node]=1;r.secondary_nodes[secondary++]=node;}};
    for(std::size_t i=0;i<d.wall.size();++i) {
        const auto& p=d.wall[i];auto& face=r.primary[i];face.source_id=p.id;face.layout=n::ShellLayout::Triangle3;
        for(unsigned j=0;j<4;++j)face.nodes[j]=p.nodes[j];
        r.primary_shells[i]=std::uint32_t(d.patch.size()+i);r.parent_ids[i]=p.id;
        for(unsigned j=0;j<3;++j)append(p.nodes[j]);
    }
    if(motion==MainMotion::MovingShells)for(std::size_t i=0;i<d.patch.size();++i) {
        const auto& p=d.patch[i];const auto target=d.wall.size()+i;auto& face=r.primary[target];
        face.source_id=p.id;face.layout=n::ShellLayout::Quad4;
        for(unsigned j=0;j<4;++j){face.nodes[j]=p.nodes[j];append(p.nodes[j]);}
        r.primary_shells[target]=std::uint32_t(i);r.parent_ids[target]=p.id;
    }
    // ILEV1 I25SURFI union of primary-surface nodes and the additional group,
    // followed by I25SORS/MY_ORDERS ascending positive external NID order.
    for(auto node:d.patch_nodes)append(node);
    Require(secondary==count,"Declared contact roster does not cover its complete source-node union");
    std::sort(r.secondary_nodes,r.secondary_nodes+count,[&](auto a,auto b){return d.nodes[a].id<d.nodes[b].id;});
    for(std::size_t i=0;i<count;++i) {
        // I25STI3 lines494–498 initializes STFN to ONE before I25STSECND.
        // This compiler profile has INACTI5 and no initial removal contributors.
        r.secondary_input[i]={r.secondary_nodes[i],1.};
    }
}
n::source_shells::Profile ShellProfile() {
    n::source_shells::Profile p;p.population=n::source_shells::Population::OrdinaryShellsOnly;
    p.property_type=1;p.input_thickness_mode=0;p.level=1;p.gap_mode=1;
    p.free_edge_gap=0;p.contact_thickness_update=0;p.stiffness_scale=1;p.gap_scale=1;
    p.maximum_secondary_gap=1e30;p.maximum_main_gap=1e30;return p;
}
n::search_startup::Profile SearchProfile(n::search_startup::Initialization initialization) {
    n::search_startup::Profile p;p.level=1;p.gap_mode=1;p.neighbor_removal=2;
    p.initial_penetration=5;p.edge_mode=0;p.thermal_mode=0;p.curvature=0;p.partitions=1;
    p.initialization=initialization;p.gap_load_cards=n::search_startup::LoadCards::Absent;return p;
}
n::TransactionConfig RuntimeProfile() {
    // Resolved interpretation of the one supported declared compiler profile,
    // pinned native scalar controls. These are not sampled numerical arrays.
    n::TransactionConfig c;c.units.length_m=.001;c.units.mass_kg=1000;c.units.time_s=1;
    auto& selection=c.lifecycle.selection;selection.gap_mode=1;selection.initial_penetration=5;
    selection.local_processor=1;selection.foreign_rows=false;selection.thermal=false;selection.gap_loading=false;
    auto& geometry=c.lifecycle.geometry;geometry.gap_mode=1;geometry.sharp=1;geometry.initial_penetration=5;
    geometry.damping_flag=1;geometry.adhesion=false;geometry.thermal=false;geometry.foreign_row=false;
    c.lifecycle.coefficient.stiffness_formulation=4;c.lifecycle.coefficient.mass_timestep_augmentation=0;
    c.lifecycle.minimum_coefficient=0;c.lifecycle.maximum_coefficient=1e30;
    c.lifecycle.neighbor_removal=2;c.lifecycle.optcd_response_precision=0;
    c.normal.stiffness_formulation=4;c.normal.damping_flag=1;c.normal.initial_penetration=5;
    c.normal.arithmetic_precision=8;c.normal.prescribed_contact_force=false;c.normal.adhesion=false;
    c.normal.damping_factor=.05;c.normal.engine.kdtint=0;c.normal.engine.idtmins=0;c.normal.engine.idtmins_int=0;
    c.friction.model=2;c.friction.formulation=10;c.friction.orthotropic=0;c.friction.converged=1;
    c.friction.thermal=0;c.friction.part_coefficients=0;c.friction.alpha=1;
    c.friction_coefficients.base=.1;c.friction_coefficients.c[4]=.1;c.friction_coefficients.c[5]=-.001;
    c.assembly.parallel_assembly=0;c.assembly.pinch=0;c.assembly.thermal=0;
    c.assembly.thermal_formulation=0;c.assembly.thermal_nodal_timestep=0;
    c.assembly.engine.kdtint=0;c.assembly.engine.idtmins=0;c.assembly.engine.idtmins_int=0;return c;
}
void FillRuntimeRows(const PhysicalSource& physical,Rows r,const n::startup::Snapshot& topology,
    const n::startup::NormalView& normals,const n::search_startup::Snapshot& search) {
    const auto& d=physical.declared().data();
    Require(normals.reference_count==topology.starter.reference_count&&
        search.main_count==topology.main_count&&search.secondary_count==d.nodes.size(),
        "Produced contact source domains differ");
    for(std::size_t i=0;i<topology.main_count;++i) {
        const auto& in=topology.mains[i];auto& out=r.mains[i];
        out.global_id=in.global_id;out.segment_type=in.segment_type;out.coefficient=r.expanded_k[i];
        n::source_shells::MainGapFields gap;
        Require(n::source_shells::MainGaps(r.node_fields,d.nodes.size(),in.nodes,&gap).status==n::source_shells::Status::Ok,
            "Native main gap distribution rejected");
        out.maximum_gap=gap.maximum;
        for(unsigned j=0;j<4;++j){out.nodes[j]=in.nodes[j];out.normal_reference[j]=in.normal_reference[j];out.neighbors[j]=in.neighbors[j];
            out.normal_slot[j]=normals.face_normals[4*i+j];out.gap[j]=gap.corner[j];}
    }
    for(std::size_t i=0;i<normals.reference_count;++i) {
        const auto& in=normals.references[i];auto& out=r.normals[i];out.boundary=in.boundary;
        // Boundary0 bisectors are native-unused and carry no numerical claim.
        if(in.boundary){out.bisector[0]=in.bisector[0];out.bisector[1]=in.bisector[1];}
    }
    for(std::size_t i=0;i<d.nodes.size();++i)
        r.secondary[i]={r.secondary_nodes[i],r.secondary_fields[i].stiffness,r.secondary_fields[i].gap,search.initial_contact[i]};
}
}
