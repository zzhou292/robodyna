#pragma once
#include "ContactBuild.h"
namespace crash::cases::native_scene::contact_detail {
// Immutable retained source and bounded producer buffers shared by the two
// explicit factory profiles. No runtime nodal fields or clock are owned here.
struct ContactStorage {
    explicit ContactStorage(const PhysicalSource& p):physical(p){}
    PhysicalSource physical;
    tl::util::HostArena rows,topology_arena,ready_arena,search_arena;
    Rows fields;
    n::startup::Snapshot topology;
    n::startup::FixedMainView ready;
    n::search_startup::Snapshot search;
    n::TransactionConfig config;
    ContactForecast forecast;
    n::search_startup::Initialization preprocessing=n::search_startup::Initialization::Unspecified;
};
struct ContactPlan {
    MainMotion motion=MainMotion::FixedWall;
    std::size_t nodes=0,primary=0,shells=0,retained_bytes=0;
    n::startup::Limits topology_limits;
    n::search_startup::Limits search_limits;
    n::startup::Forecast topology;
    n::search_startup::Forecast search;
    Layout rows;
};
ContactPlan PlanContact(const PhysicalSource&,ContactIdentity,ContactLimits,MainMotion,std::size_t object_bytes);
void BuildContact(ContactStorage&,n::ContactSourceInput&,ContactIdentity,ContactLimits,const ContactPlan&);
}
