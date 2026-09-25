#pragma once
#include "ContactSource.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#include "lib_utils/BoundedArena.h"
namespace crash::cases::native_scene::contact_detail {
namespace n=tlfea::contact::radioss_type25;
namespace l=n::lifecycle;
struct Rows {
    std::uint64_t* ids=nullptr;double* positions=nullptr;
    n::source_shells::PhysicalShell* shells=nullptr;
    std::uint32_t* primary_shells=nullptr;std::uint32_t* secondary_nodes=nullptr;std::uint8_t* selected=nullptr;
    n::source_shells::Secondary* secondary_input=nullptr;
    n::startup::PrimaryFace* primary=nullptr;
    n::source_shells::NodeFields* node_fields=nullptr;
    double* primary_k=nullptr;double* expanded_k=nullptr;double* expanded_gaps=nullptr;
    n::source_shells::SecondaryFields* secondary_fields=nullptr;
    n::search_startup::Secondary* search_secondary=nullptr;
    l::Node* nodes=nullptr;l::Main* mains=nullptr;l::Secondary* secondary=nullptr;l::NormalReference* normals=nullptr;
    std::uint64_t* parent_ids=nullptr;
};
struct Layout {
    tl::util::ArenaRegion ids,positions,shells,primary_shells,secondary_nodes,selected,secondary_input,primary;
    tl::util::ArenaRegion node_fields,primary_k,expanded_k,expanded_gaps,secondary_fields,search_secondary;
    tl::util::ArenaRegion nodes,mains,secondary,normals,parent_ids;
    std::size_t bytes=0;
};
Layout PlanRows(std::size_t nodes,std::size_t shells,std::size_t primary,std::size_t references,std::size_t cap);
Rows Construct(tl::util::HostArena&,const Layout&);
void FillDeclaredInputs(const PhysicalSource&,Rows);
n::source_shells::Profile ShellProfile();
n::TransactionConfig RuntimeProfile();
n::search_startup::Profile SearchProfile(n::search_startup::Initialization);
void FillRuntimeRows(const PhysicalSource&,Rows,const n::startup::Snapshot&,const n::startup::FixedMainView&,
    const n::search_startup::Snapshot&);
}
