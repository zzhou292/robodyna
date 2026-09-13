// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "lib_src/elements/solid18/Solid18Reference.h"
#include "lib_src/elements/solid24/Solid24Reference.h"
#include "lib_src/elements/solid6z/Solid6zReference.h"

namespace coefficient_test {
struct SolidFixture : Fixture {
  fe::solid18::ReferenceInput a;
  fe::solid24::ReferenceInput b;
  fe::solid6z::ReferenceInput c;
  explicit SolidFixture(bool contact_geometry = false)
      : Fixture(contact_geometry) {
    a.source_element_id=9100; a.source_part_id=9200;
    a.source_section_id=9201; a.source_material_id=9202;
    a.density_kg_m3=1070;
    const tl::math::Vec3 x[8]{{0,0,0},{.04,0,0},{.04,.03,0},{0,.03,0},
        {0,0,.02},{.04,0,.02},{.04,.03,.02},{0,.03,.02}};
    for(unsigned k=0;k<8;++k) {
      a.source_node_id[k]=k<2?10+k:9300+k;
      a.position_m[k]=x[k];
      if(k>=2) nodes.push_back({a.source_node_id[k],x[k]});
    }
    b.source_element_id=9101; b.source_part_id=9203;
    b.source_section_id=9204; b.source_material_id=9205;
    b.density_kg_m3=1980;
    // Distinct physical elements may share their full topology.
    for(unsigned k=0;k<8;++k) {
      b.source_node_id[k]=a.source_node_id[k]; b.position_m[k]=x[k];
    }
    b.profile.reference_strain=fe::solid24::ReferenceStrain::TotalLagrangian10;
    c.source_element_id=9102; c.source_part_id=9206;
    c.source_section_id=9207; c.source_material_id=9208;
    c.density_kg_m3=1980;
    const unsigned wedge[6]{0,1,3,4,5,7};
    for(unsigned k=0;k<6;++k) {
      c.source_node_id[k]=a.source_node_id[wedge[k]]; c.position_m[k]=x[wedge[k]];
    }
  }
  fe::SolidNodeContributions Solids(const fe::NodalNodeDomain& domain) const {
    fe::solid18::Reference r18; fe::solid24::Reference r24; fe::solid6z::Reference r6;
    EXPECT_EQ(fe::solid18::InitializeReference(a,r18),fe::solid18::Status::Success);
    EXPECT_EQ(fe::solid24::InitializeReference(b,r24),fe::solid24::Status::Success);
    EXPECT_EQ(fe::solid6z::InitializeReference(c,r6),fe::solid6z::Status::Success);
    fe::SolidNodeContributions output;
    const auto report=output.Initialize(domain,{1,&r18,1,&r24,1,&r6,1});
    EXPECT_TRUE(report)<<report.message;
    return output; // All borrowed reference objects die here.
  }
};
}  // namespace coefficient_test
