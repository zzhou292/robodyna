// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Source.h"
#include "lib_utest/qualification/beam18_force/TestSupport.h"
namespace beam18_publication_test {
bool Source::Initialize(existing::Fixture& f) {
  const std::uint64_t ends[3][2]{{778,9305},{9302,9306},{14,9305}};
  std::array<b::ParentInput,3> rows;
  std::vector<double> x(std::begin(law44_solid_test::X),std::end(law44_solid_test::X));
  std::vector<double> y(std::begin(law44_solid_test::Y),std::end(law44_solid_test::Y));
  for (unsigned p=0;p<rows.size();++p) {
    auto input=beam18_force_test::Reference().input();
    input.source_element_id=32000+p; input.source_part_id=32100;
    input.source_section_id=32200; input.source_material_id=32300;
    for (unsigned n=0;n<2;++n) {
      input.source_node_id[n]=ends[p][n];
      input.position[n]=f.domain.nodes()[f.domain.Find(ends[p][n])].position;
    }
    input.source_node_id[2]=0; input.position[2]={};
    if (b::InitializeReference(input,rows[p].reference)!=b::Status::Success) return false;
    rows[p].material=beam18_force_test::Material(rows[p].reference);
    rows[p].material.curve={x.data(),y.data(),static_cast<std::uint32_t>(x.size())};
  }
  const auto model_report=model.Initialize(f.domain,{f.domain.source_instance_id(),
      {rows.data(),rows.size()},b::ModelProfile::CircularFourPointLaw44V1});
  if (!model_report || !contribution.Initialize(model) ||
      !ledger.InitializeWithBeams({{{&f.shells,&f.welds,&f.beam_coefficients},&f.point,
        f.solids.contributions()},&contribution}) || !parts.Initialize(f.topology,ledger,{1000,.001})) return false;
  std::vector<fe::NodalRigidGroupMember> members(f.plain.member_count());
  for (std::size_t n=0;n<members.size();++n) {
    members[n]=f.plain.members()[n];
    const auto& c=ledger.nodes()[members[n].global_node].coefficients;
    members[n].mass_kg=c.mass;
    members[n].total_inertia_kg_m2=c.isotropic_inertia;
    members[n].physical_inertia_kg_m2=c.shell.physical_inertia;
    members[n].added_inertia_kg_m2=c.shell.added_inertia;
    members[n].unpartitioned_native_inertia_kg_m2=c.beam18.isotropic_inertia;
  }
  const auto& prior=f.plain.groups()[0];
  const fe::NodalRigidGroupInput group{prior.source_group_id,prior.source_node_set_id,members.data(),members.size()};
  if (!plain.InitializeNativeTotal({f.plain.source_instance_id(),f.domain.node_count(),&group,1,{1000,.001}}) ||
      !rigid.Initialize(parts,&plain) ||
      !physical.Initialize({&f.source.shells,&f.catalog,&f.failure,nullptr},ledger)) return false;
  for (std::size_t n=0;n<f.domain.node_count();++n) {
    f.m[n]=ledger.nodes()[n].coefficients.mass;
    f.j[n]=ledger.nodes()[n].coefficients.isotropic_inertia;
    f.present[n]=f.j[n]>0 || rigid.FindMember(n);
    f.im[n]=f.fixed[n]==7 ? 0 : (f.m[n]>0 ? 1/f.m[n] : 0);
    f.ij[n]=f.rotation_fixed[n] ? 0 : (f.j[n]>0 ? 1/f.j[n] : 0);
  }
  f.im[f.domain.Find(901)]=0; f.ij[f.domain.Find(901)]=0;
  return true; // Caller curve arrays and parent declarations die here.
}
} // namespace beam18_publication_test
