#include "Fixture.h"
#include "../type13/NativeOracle.h"
extern "C" void type25_native_mass(const double*,const double*,double*);

namespace coefficient_test {
TEST(NodalCoefficientNative, IndependentSpringCoefficientsAtSharedOwnerNodes) {
  const Fixture f; const auto d=f.Domain(); const auto map=f.Map(d);
  const auto springs=f.Springs(); const auto beams=f.Contributions(d);
  fe::NodalCoefficientLedger ledger; ASSERT_TRUE(ledger.Initialize({&map,&springs,&beams}));
  std::vector<long double> mass(d.node_count()),inertia(d.node_count());
  for(std::size_t local=0;local<f.shells.node_count();++local) {
    mass[f.map[local]]=f.shells.nodes()[local].native.mass;
    inertia[f.map[local]]=f.shells.nodes()[local].native.isotropic_inertia;
  }
  double native25[2]{};
  type25_native_mass(&f.spring_property.property.mass_kg,
                    &f.spring_property.property.isotropic_inertia_kg_m2,native25);
  for(std::size_t e=0;e<4;++e) {
    const auto& endpoint=springs.endpoint_mass()[e];
    EXPECT_EQ(Bits(endpoint.mass_kg),Bits(native25[0]));
    EXPECT_EQ(Bits(endpoint.isotropic_inertia_kg_m2),Bits(native25[1]));
    mass[endpoint.global_node]+=native25[0]; inertia[endpoint.global_node]+=native25[1];
  }
  const auto& p=f.beam_input.declaration.input;
  const auto units=beams.model()->units();
  const auto jscale=units.mass_to_kg*units.length_to_m*units.length_to_m;
  for(std::size_t e=0;e<2;++e) {
    beam::ReferenceInput geometry;
    for(unsigned n=0;n<3;++n) geometry.position[n]=f.beam_input.nodes[f.beam_input.connections[e].node[n]].position_native;
    const auto native_frame=type13_test::NativeReference(geometry);
    const auto native=type13_test::NativeMass(p.mass_per_length,p.inertia_per_length,native_frame.length);
    for(unsigned local=0;local<2;++local) {
      const auto& r=beams.records()[2*e+local].value;
      EXPECT_EQ(Bits(r.coefficients.mass_kg),Bits(native[0]*units.mass_to_kg));
      EXPECT_EQ(Bits(r.coefficients.isotropic_inertia_kg_m2),Bits(native[1]*jscale));
      EXPECT_EQ(Bits(r.coefficients.added_inertia_kg_m2),Bits(native[1]*jscale));
      mass[r.global_node]+=native[0]*units.mass_to_kg;
      inertia[r.global_node]+=native[1]*jscale;
    }
  }
  for(std::size_t n=0;n<d.node_count();++n) {
    const auto& actual=ledger.nodes()[n].coefficients;
    EXPECT_LE(std::abs(static_cast<long double>(actual.mass)-mass[n]),
      16*std::numeric_limits<double>::epsilon()*mass[n]);
    EXPECT_LE(std::abs(static_cast<long double>(actual.isotropic_inertia)-inertia[n]),
      16*std::numeric_limits<double>::epsilon()*inertia[n]);
  }
}
} // namespace coefficient_test
