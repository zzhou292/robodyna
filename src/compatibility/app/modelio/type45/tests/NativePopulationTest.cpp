#include "../VehicleType45Source.h"
#include "../SourcePolicy.h"
#include "modelio/physical_scope/tests/NativeSupport.h"
#include "modelio/physical_domain/Policy.h"
#include "modelio/point_mass/VehiclePointMassSource.h"
#include <gtest/gtest.h>
namespace crash::modelio::type45 {
namespace {
const physical_domain::VehiclePhysicalDomain& V5(){static const auto d=physical_domain::VehiclePhysicalDomain::Prepare(
 physical_scope::test::V5Supports().scope,physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5);return d;}
const physical_domain::VehiclePhysicalDomain& V6(){static const auto d=physical_domain::VehiclePhysicalDomain::Prepare(
 physical_scope::test::V6Supports().scope,physical_domain::Policy::RetainedShellAssembliesNativeSupportsV6);return d;}
void GroupEqual(const physical_scope::Group& a,const physical_scope::Group& b){
 EXPECT_EQ(a.id,b.id);EXPECT_EQ(a.node_set_id,b.node_set_id);EXPECT_EQ(a.child_part_id,b.child_part_id);EXPECT_EQ(a.source_index,b.source_index);
 EXPECT_EQ(a.before,b.before);EXPECT_EQ(a.after,b.after);EXPECT_EQ(a.covered_before,b.covered_before);EXPECT_EQ(a.covered_after,b.covered_after);EXPECT_EQ(a.tire_members,b.tire_members);
 ASSERT_EQ(a.members.size(),b.members.size());for(std::size_t n=0;n<a.members.size();++n){EXPECT_EQ(a.members[n].node,b.members[n].node);EXPECT_EQ(a.members[n].roles,b.members[n].roles);}
}
template<class V>void PositionBits(const V& a,const V& b){EXPECT_EQ(output::Bits(a.x),output::Bits(b.x));EXPECT_EQ(output::Bits(a.y),output::Bits(b.y));EXPECT_EQ(output::Bits(a.z),output::Bits(b.z));}
}
TEST(VehicleNativeV6Population, SamePhysicalNodeRolesAndGroupsWithRaw8NativeSolidAuthority){
 const auto& a=V5().source();const auto& b=V6().source();const auto& x=a.data();const auto& y=b.data();
 EXPECT_EQ(a.solid_source().data().policy,solid_source::Policy::OriginalVehicleSupportsV5);EXPECT_EQ(b.solid_source().data().policy,solid_source::Policy::NativeConvertedSupportsV6);
 EXPECT_EQ(a.solid_source().data().rows.size(),4980u);EXPECT_EQ(b.solid_source().data().rows.size(),4980u);
 EXPECT_EQ(a.solid_source().data().solid24.size(),1991u);EXPECT_EQ(a.solid_source().data().solid6z.size(),350u);
 EXPECT_EQ(b.solid_source().data().solid24.size(),2341u);EXPECT_TRUE(b.solid_source().data().solid6z.empty());
 ASSERT_NE(a.structural_beam_source(),nullptr);ASSERT_NE(b.structural_beam_source(),nullptr);
 EXPECT_EQ(a.structural_beam_source()->data().rows.size(),142u);EXPECT_EQ(b.structural_beam_source()->data().rows.size(),142u);
 EXPECT_EQ(a.structural_beam_source()->data().canonical_endpoints,b.structural_beam_source()->data().canonical_endpoints);
 EXPECT_EQ(x.node_roles,y.node_roles);ASSERT_EQ(x.plain_groups.size(),y.plain_groups.size());ASSERT_EQ(x.part_roots.size(),y.part_roots.size());
 for(std::size_t i=0;i<x.plain_groups.size();++i)GroupEqual(x.plain_groups[i],y.plain_groups[i]);for(std::size_t i=0;i<x.part_roots.size();++i)GroupEqual(x.part_roots[i],y.part_roots[i]);
 EXPECT_EQ(x.counts.baseline_nodes,y.counts.baseline_nodes);EXPECT_EQ(x.counts.with_type25_nodes,y.counts.with_type25_nodes);EXPECT_EQ(x.counts.type25_added_nodes,y.counts.type25_added_nodes);
 for(unsigned i=0;i<8;++i)EXPECT_EQ(x.counts.role_nodes[i],y.counts.role_nodes[i]);
 ASSERT_EQ(x.point_masses.size(),155u);ASSERT_EQ(x.point_masses.size(),y.point_masses.size());
 for(std::size_t i=0;i<x.point_masses.size();++i){const auto& p=x.point_masses[i];const auto& q=y.point_masses[i];EXPECT_EQ(p.element,q.element);EXPECT_EQ(p.node,q.node);EXPECT_EQ(p.source_record,q.source_record);EXPECT_EQ(p.root_index,q.root_index);EXPECT_EQ(p.roles,q.roles);}
 ASSERT_EQ(x.rigid_skin.size(),y.rigid_skin.size());for(std::size_t i=0;i<x.rigid_skin.size();++i){const auto& p=x.rigid_skin[i];const auto& q=y.rigid_skin[i];EXPECT_EQ(p.element,q.element);EXPECT_EQ(p.part,q.part);EXPECT_EQ(p.canonical_row,q.canonical_row);EXPECT_EQ(p.root_index,q.root_index);}
 ASSERT_EQ(x.spotwelds.size(),y.spotwelds.size());for(std::size_t i=0;i<x.spotwelds.size();++i){const auto& p=x.spotwelds[i];const auto& q=y.spotwelds[i];EXPECT_EQ(p.id,q.id);EXPECT_EQ(p.nodes,q.nodes);EXPECT_EQ(p.source_index,q.source_index);EXPECT_EQ(p.first_card,q.first_card);EXPECT_EQ(p.default_only,q.default_only);}
 ASSERT_EQ(x.evidence.size(),y.evidence.size());for(std::size_t i=0;i<x.evidence.size();++i){const auto& p=x.evidence[i];const auto& q=y.evidence[i];EXPECT_EQ(p.node,q.node);EXPECT_EQ(p.roles,q.roles);ASSERT_EQ(p.incidence.size(),q.incidence.size());for(std::size_t j=0;j<p.incidence.size();++j){const auto& u=p.incidence[j];const auto& v=q.incidence[j];EXPECT_EQ(u.element,v.element);EXPECT_EQ(u.part,v.part);EXPECT_EQ(u.canonical_row,v.canonical_row);EXPECT_EQ(u.local_slots,v.local_slots);EXPECT_EQ(u.family,v.family);EXPECT_EQ(u.selected,v.selected);EXPECT_EQ(u.orientation_only,v.orientation_only);}}
 RecordProperty("selected_raw_solid_rows",4980);RecordProperty("native_heph_rows",2341);RecordProperty("collapsed_heph_rows",350);RecordProperty("structural_beams",142);
}
TEST(VehicleNativeV6Population, SameDomainMembershipPointMassesAndLiteralRigidTopology){
 const auto& a=V5();const auto& b=V6();EXPECT_TRUE(a.domain().Matches(b.domain()));ASSERT_EQ(a.domain().node_count(),b.domain().node_count());
 for(std::size_t n=0;n<a.domain().node_count();++n){EXPECT_EQ(a.domain().nodes()[n].source_id,b.domain().nodes()[n].source_id);PositionBits(a.domain().nodes()[n].position,b.domain().nodes()[n].position);}
 ASSERT_EQ(a.plain_groups().size(),b.plain_groups().size());for(std::size_t i=0;i<a.plain_groups().size();++i){const auto& x=a.plain_groups()[i];const auto& y=b.plain_groups()[i];EXPECT_EQ(x.source_group,y.source_group);EXPECT_EQ(x.disposition,y.disposition);EXPECT_EQ(x.case_node_set_id,y.case_node_set_id);EXPECT_EQ(x.members,y.members);EXPECT_EQ(x.excluded_members,y.excluded_members);}
 const auto ac=a.counts(),bc=b.counts();EXPECT_EQ(ac.complete_groups,bc.complete_groups);EXPECT_EQ(ac.restricted_groups,bc.restricted_groups);EXPECT_EQ(ac.omitted_groups,bc.omitted_groups);EXPECT_EQ(ac.plain_members,bc.plain_members);EXPECT_EQ(ac.retained_point_masses,154u);EXPECT_EQ(bc.retained_point_masses,154u);
 const auto& x=a.topology();const auto& y=b.topology();ASSERT_EQ(x.part_count(),y.part_count());ASSERT_EQ(x.extra_count(),y.extra_count());ASSERT_EQ(x.root_count(),y.root_count());ASSERT_EQ(x.merge_count(),y.merge_count());ASSERT_EQ(x.member_count(),y.member_count());ASSERT_EQ(x.other_rigid_member_count(),y.other_rigid_member_count());
 for(std::size_t i=0;i<x.part_count();++i){const auto& p=x.parts()[i];const auto& q=y.parts()[i];EXPECT_EQ(p.source_part_id,q.source_part_id);EXPECT_EQ(p.member_offset,q.member_offset);EXPECT_EQ(p.member_count,q.member_count);EXPECT_EQ(p.extra_row,q.extra_row);EXPECT_EQ(p.root_index,q.root_index);}
 for(std::size_t i=0;i<x.extra_count();++i){const auto& p=x.extras()[i];const auto& q=y.extras()[i];EXPECT_EQ(p.source_node_set_id,q.source_node_set_id);EXPECT_EQ(p.part_index,q.part_index);EXPECT_EQ(p.member_offset,q.member_offset);EXPECT_EQ(p.member_count,q.member_count);}
 for(std::size_t i=0;i<x.root_count();++i){const auto& p=x.roots()[i];const auto& q=y.roots()[i];EXPECT_EQ(p.part_index,q.part_index);EXPECT_EQ(p.child_part_index,q.child_part_index);EXPECT_EQ(p.member_offset,q.member_offset);EXPECT_EQ(p.member_count,q.member_count);EXPECT_EQ(p.original_primary_count,q.original_primary_count);}
 for(std::size_t i=0;i<x.merge_count();++i){EXPECT_EQ(x.merges()[i].parent_part_id,y.merges()[i].parent_part_id);EXPECT_EQ(x.merges()[i].child_part_id,y.merges()[i].child_part_id);}
 for(std::size_t i=0;i<x.member_count();++i){EXPECT_EQ(x.original_members()[i],y.original_members()[i]);EXPECT_EQ(x.root_members()[i],y.root_members()[i]);EXPECT_EQ(x.expected_members()[i],y.expected_members()[i]);}
 for(std::size_t i=0;i<x.other_rigid_member_count();++i)EXPECT_EQ(x.other_rigid_members()[i],y.other_rigid_members()[i]);
 const auto p=point_mass::VehiclePointMassSource::Prepare(a.source(),a.domain());const auto q=point_mass::VehiclePointMassSource::Prepare(b.source(),b.domain());
 EXPECT_TRUE(p.contributions().Matches(q.contributions()));ASSERT_EQ(p.dispositions().size(),q.dispositions().size());for(std::size_t i=0;i<p.dispositions().size();++i){EXPECT_EQ(p.dispositions()[i].source_record,q.dispositions()[i].source_record);EXPECT_EQ(p.dispositions()[i].domain_node,q.dispositions()[i].domain_node);}
 EXPECT_EQ(q.contributions().records().size(),154u);RecordProperty("physical_nodes",std::to_string(b.domain().node_count()));RecordProperty("plain_rigid_groups",std::to_string(b.plain_groups().size()));RecordProperty("retained_point_mass_cards",154);
}
TEST(VehicleNativeV6Population, Same44JointCardsEndpointsAxesAndBodies){
 const auto old=VehicleType45Source::Prepare(V5(),Policy::OriginalDirectSdiType45VehicleSupportsV5);const auto current=VehicleType45Source::Prepare(V6(),Policy::OriginalDirectSdiType45NativeSupportsV6);
 const auto& a=old.data();const auto& b=current.data();ASSERT_EQ(a.rows.size(),44u);ASSERT_EQ(a.rows.size(),b.rows.size());EXPECT_EQ(a.required,b.required);EXPECT_EQ(b.required,44u);EXPECT_EQ(a.boundaries,b.boundaries);EXPECT_EQ(b.boundaries,0u);EXPECT_EQ(current.readiness(),RuntimeReadiness::RequiresOwnerTt0Context);
 for(unsigned i=0;i<3;++i){EXPECT_EQ(a.properties[i].origin_joint_id,b.properties[i].origin_joint_id);const auto& p=a.properties[i].value;const auto& q=b.properties[i].value;EXPECT_EQ(p.kind,q.kind);EXPECT_EQ(p.working_units,q.working_units);EXPECT_EQ(output::Bits(p.automatic_stiffness_scale),output::Bits(q.automatic_stiffness_scale));EXPECT_EQ(output::Bits(p.critical_damping_ratio),output::Bits(q.critical_damping_ratio));PositionBits(p.free_stiffness.translation,q.free_stiffness.translation);PositionBits(p.free_stiffness.rotation,q.free_stiffness.rotation);PositionBits(p.free_viscosity.translation,q.free_viscosity.translation);PositionBits(p.free_viscosity.rotation,q.free_viscosity.rotation);}
 for(std::size_t i=0;i<a.rows.size();++i){const auto& p=a.rows[i];const auto& q=b.rows[i];EXPECT_EQ(p.source_id,q.source_id);EXPECT_EQ(p.source_index,q.source_index);EXPECT_EQ(p.header_line,q.header_line);EXPECT_EQ(p.card_line,q.card_line);EXPECT_EQ(p.property_index,q.property_index);EXPECT_EQ(p.source_node_count,q.source_node_count);EXPECT_EQ(p.blank_mask,q.blank_mask);EXPECT_EQ(p.unused_columns,q.unused_columns);EXPECT_EQ(p.disposition,q.disposition);
  for(unsigned n=0;n<4;++n){const auto& x=p.nodes[n];const auto& y=q.nodes[n];EXPECT_EQ(x.source_id,y.source_id);EXPECT_EQ(x.canonical_index,y.canonical_index);EXPECT_EQ(x.domain_index,y.domain_index);EXPECT_EQ(x.use,y.use);PositionBits(x.position_m,y.position_m);EXPECT_EQ(x.body.kind,y.body.kind);EXPECT_EQ(x.body.source_id,y.body.source_id);EXPECT_EQ(x.body.source_index,y.body.source_index);EXPECT_EQ(x.body.retained,y.body.retained);}}
 RecordProperty("required_joint_cards",44);RecordProperty("omitted_joint_cards",0);
}
TEST(VehicleNativeV6Population, PolicyMismatchMissingSupportAndOneByteShortCapsRejectWithoutReplacement){
 auto domain=V6();const auto* nodes=domain.domain().nodes().data();
 EXPECT_THROW(domain=physical_domain::VehiclePhysicalDomain::Prepare(V6().source(),physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5),std::runtime_error);EXPECT_EQ(domain.domain().nodes().data(),nodes);
 EXPECT_THROW(physical_domain::VehiclePhysicalDomain::Prepare(V5().source(),physical_domain::Policy::RetainedShellAssembliesNativeSupportsV6),std::runtime_error);
 const auto& in=physical_scope::test::V6Supports().sources;EXPECT_THROW(physical_scope::PhysicalScope::Prepare(in.masses,in.tied,in.beams,in.solids),std::runtime_error);
 physical_domain::Limits limits;limits.host_bytes=domain.forecast().total_bytes;EXPECT_EQ(physical_domain::VehiclePhysicalDomain::Preflight(domain.source(),domain.policy(),limits).total_bytes,limits.host_bytes);--limits.host_bytes;
 EXPECT_THROW(domain=physical_domain::VehiclePhysicalDomain::Prepare(domain.source(),domain.policy(),limits),std::runtime_error);EXPECT_EQ(domain.domain().nodes().data(),nodes);
 auto joints=VehicleType45Source::Prepare(domain,Policy::OriginalDirectSdiType45NativeSupportsV6);const auto* rows=joints.data().rows.data();EXPECT_THROW(joints=VehicleType45Source::Prepare(domain,Policy::OriginalDirectSdiType45VehicleSupportsV5),std::runtime_error);EXPECT_EQ(joints.data().rows.data(),rows);
}
} // namespace crash::modelio::type45
