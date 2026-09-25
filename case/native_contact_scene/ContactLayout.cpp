#include "ContactBuild.h"
#include "output/ArtifactIO.h"
namespace crash::cases::native_scene::contact_detail {
Layout PlanRows(std::size_t nodes,std::size_t shells,std::size_t primary,std::size_t references,std::size_t cap) {
    Layout r;tl::util::BoundedArenaLayout b(cap);
    output::Require(b.Append<std::uint64_t>(nodes,r.ids)&&b.Append<double>(3*nodes,r.positions)&&
        b.Append<n::source_shells::PhysicalShell>(shells,r.shells)&&b.Append<std::uint32_t>(primary,r.primary_shells)&&
        b.Append<std::uint32_t>(nodes,r.secondary_nodes)&&b.Append<std::uint8_t>(nodes,r.selected)&&
        b.Append<n::source_shells::Secondary>(nodes,r.secondary_input)&&b.Append<n::startup::PrimaryFace>(primary,r.primary)&&
        b.Append<n::source_shells::NodeFields>(nodes,r.node_fields)&&b.Append<double>(primary,r.primary_k)&&
        b.Append<double>(2*primary,r.expanded_k)&&b.Append<double>(2*primary,r.expanded_gaps)&&
        b.Append<n::source_shells::SecondaryFields>(nodes,r.secondary_fields)&&b.Append<n::search_startup::Secondary>(nodes,r.search_secondary)&&
        b.Append<l::Node>(nodes,r.nodes)&&b.Append<l::Main>(2*primary,r.mains)&&b.Append<l::Secondary>(nodes,r.secondary)&&
        b.Append<l::NormalReference>(references,r.normals)&&b.Append<std::uint64_t>(primary,r.parent_ids),"Complete contact-source rows exceed host cap");
    r.bytes=b.bytes();return r;
}
Rows Construct(tl::util::HostArena& a,const Layout& p) {
    Rows r;
    r.ids=a.Construct<std::uint64_t>(p.ids);r.positions=a.Construct<double>(p.positions);
    r.shells=a.Construct<n::source_shells::PhysicalShell>(p.shells);r.primary_shells=a.Construct<std::uint32_t>(p.primary_shells);
    r.secondary_nodes=a.Construct<std::uint32_t>(p.secondary_nodes);r.selected=a.Construct<std::uint8_t>(p.selected);
    r.secondary_input=a.Construct<n::source_shells::Secondary>(p.secondary_input);r.primary=a.Construct<n::startup::PrimaryFace>(p.primary);
    r.node_fields=a.Construct<n::source_shells::NodeFields>(p.node_fields);r.primary_k=a.Construct<double>(p.primary_k);
    r.expanded_k=a.Construct<double>(p.expanded_k);r.expanded_gaps=a.Construct<double>(p.expanded_gaps);
    r.secondary_fields=a.Construct<n::source_shells::SecondaryFields>(p.secondary_fields);
    r.search_secondary=a.Construct<n::search_startup::Secondary>(p.search_secondary);
    r.nodes=a.Construct<l::Node>(p.nodes);r.mains=a.Construct<l::Main>(p.mains);r.secondary=a.Construct<l::Secondary>(p.secondary);
    r.normals=a.Construct<l::NormalReference>(p.normals);r.parent_ids=a.Construct<std::uint64_t>(p.parent_ids);
    output::Require(r.ids&&r.positions&&r.shells&&r.primary_shells&&r.secondary_nodes&&r.selected&&r.secondary_input&&r.primary&&
        r.node_fields&&r.primary_k&&r.expanded_k&&r.expanded_gaps&&r.secondary_fields&&r.search_secondary&&r.nodes&&r.mains&&
        r.secondary&&r.normals&&r.parent_ids,"Contact-source arena construction failed");
    return r;
}
}
