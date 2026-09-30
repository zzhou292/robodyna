#include "Fixture.h"
// Existing independently authenticated HM_READ_ADMAS TYPE5 extract. Values are
// by value in the unchanged BIND(C) ABI; output is {nodal M, nodal J, added M}.
extern "C" void rigid_part_native_point_mass(double,double,double,double*);

namespace coefficient_test {
TEST(NodalCoefficientNative, AdditiveType5KeepsScalarInertiaAndUsesEverySourceRow) {
  const Fixture f;
  const auto d=f.Domain(); const auto map=f.Map(d);
  const auto beams=f.Contributions(d); const auto springs=f.Springs();
  const fe::NodalCoefficientSources sources{&map,&springs,&beams};
  const fe::ElementMassSource rows[]={{9100,d.nodes()[f.map[1]].source_id,f.map[1],.0011},
      {9101,d.nodes()[2].source_id,2,.000001},
      {9102,d.nodes()[f.map[1]].source_id,f.map[1],.0023},
      {9103,d.nodes()[2].source_id,2,0.0}};
  fe::ElementMassContributions masses;
  ASSERT_TRUE(masses.Initialize(d,{1,1000,rows,4}));
  fe::NodalCoefficientLedger before,after;
  ASSERT_TRUE(before.Initialize(sources));
  ASSERT_TRUE(after.InitializeWithElementMass({sources,&masses}));
  for(std::size_t n=0;n<d.node_count();++n) {
    double mass=before.nodes()[n].coefficients.mass;
    double inertia=before.nodes()[n].coefficients.isotropic_inertia;
    double point_mass=0;
    for(const auto& row:rows) if(row.domain_node==n) {
      const double source_si=row.mass_source*1000;
      double native[3]{};
      rigid_part_native_point_mass(mass,inertia,source_si,native);
      mass=native[0]; inertia=native[1]; point_mass+=native[2];
    }
    EXPECT_EQ(Bits(after.nodes()[n].coefficients.mass),Bits(mass));
    EXPECT_EQ(Bits(after.nodes()[n].coefficients.isotropic_inertia),Bits(inertia));
    EXPECT_EQ(Bits(after.nodes()[n].coefficients.element_mass),Bits(point_mass));
  }
  // Real SI input order is intentional; this does not assert native global
  // element/ADMAS startup ordering or use a rigid aggregate as a mass oracle.
}
} // namespace coefficient_test
