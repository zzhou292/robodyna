// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/nodal_coefficients/SolidFixture.h"
#include "lib_utest/qualification/solid18_law44_reference/TestSupport.h"
#include "lib_utest/qualification/law90_solid18_reference/TestSupport.h"
namespace extended_solid_test {
namespace fe = tl::fea;
using Family = fe::SolidCoefficientFamily;
using Profile = fe::SolidCoefficientProfile;
using Status = fe::NodalDomainStatus;
struct Fixture : coefficient_test::SolidFixture {
  fe::solid18::ReferenceInput rear_input=rear18_test::Collapsed();
  fe::solid18::ReferenceInput foam_input=law90_reference_test::Cube();
  fe::solid18::Reference old;
  fe::solid24::Reference brick;
  fe::solid6z::Reference wedge;
  fe::solid18::law44::Reference rear;
  fe::solid18::total_strain::Reference foam;
  Fixture() {
    auto add=[&](auto& input, std::uint64_t offset) {
      input.source_element_id=offset;
      input.source_part_id+=offset;
      for(unsigned k=0;k<8;++k) {
        input.source_node_id[k]+=offset;
        bool seen=false;
        for(unsigned j=0;j<k;++j) seen |= input.source_node_id[k]==input.source_node_id[j];
        if(!seen) nodes.push_back({input.source_node_id[k],input.position_m[k]});
      }
    };
    add(rear_input,100000);add(foam_input,200000);
    EXPECT_EQ(fe::solid18::InitializeReference(a,old),fe::solid18::Status::Success);
    EXPECT_EQ(fe::solid24::InitializeReference(b,brick),fe::solid24::Status::Success);
    EXPECT_EQ(fe::solid6z::InitializeReference(c,wedge),fe::solid6z::Status::Success);
    EXPECT_EQ(fe::solid18::law44::InitializeReference(rear_input,rear),fe::solid18::Status::Success);
    EXPECT_EQ(fe::solid18::total_strain::InitializeReference90(foam_input,foam),fe::solid18::Status::Success);
  }
  fe::SolidCoefficientInput Input() const {
    return {1,&old,1,&brick,1,&wedge,1,&rear,1,&foam,1,Profile::ExtendedLaw44Law90};
  }
};
} // namespace extended_solid_test
