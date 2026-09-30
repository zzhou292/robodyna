// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "../rigid_assembly_owner/Fixture.h"

namespace solid_resident_test {
// The existing owner fixture owns its full ledger, PART/plain groups and CIN.
// Its source contributors are prescribed test inputs, not a vehicle publisher.
struct Fixture {
  rigid_assembly_owner_test::Fixture mechanics{false,true,.002,true};
  double strain[3]{0,.2,.4},stress[3]{1e6,2e6,3e6};
  s::Input18 input18;
  s::Input24 input24;
  s::Input6z input6z;
  s::Model model;
  Fixture() {
    const auto& source=mechanics.source;
    EXPECT_EQ(fe::solid18::InitializeReference(source.a,input18.reference),fe::solid18::Status::Success);
    EXPECT_EQ(fe::solid24::InitializeReference(source.b,input24.reference),fe::solid24::Status::Success);
    EXPECT_EQ(fe::solid6z::InitializeReference(source.c,input6z.reference),fe::solid6z::Status::Success);
    EXPECT_EQ(tl::material::law36::Prepare(100e6,.3,source.a.density_kg_m3,
        {strain,stress,3},input18.material),tl::material::law36::Status::Ok);
    EXPECT_EQ(tl::material::law42::Prepare(24e6,.463,source.b.density_kg_m3,1e26,input24.material),
        tl::material::law42::Status::Ok);
    input6z.material=input24.material;
    const auto report=model.Initialize(mechanics.domain,{1,{&input18,1},{&input24,1},{&input6z,1}});
    EXPECT_TRUE(report)<<report.message;
  }
  fe::NodalCinWitnessSource Witnesses() const {
    return {&mechanics.cin_model,mechanics.ranges.data(),mechanics.witnesses.data(),
        mechanics.ranges.size(),mechanics.witnesses.size()};
  }
  s::BatchConfig Configuration() const {
    auto config=Config(mechanics.domain.node_count());
    // The recurrence validates the actual CIN owner interval. Its receipt and
    // contributor must use that same qualification identity, not Config's
    // standalone constructor-test identity.
    config.qualification_id=mechanics.Cin().qualification_id;
    config.owner.fixed_dt=mechanics.Config().fixed_dt;
    config.owner.has_rotation_presence=true;
    config.owner.rigid_groups={1,mechanics.binding.groups().size(),mechanics.binding.members().size(),
        mechanics.parts.roots().size(),mechanics.binding.plain_source_instance_id()};
    config.cin_attachment_count=mechanics.ranges.size();
    config.cin_witness_count=mechanics.witnesses.size();
    return config;
  }
};
} // namespace solid_resident_test
