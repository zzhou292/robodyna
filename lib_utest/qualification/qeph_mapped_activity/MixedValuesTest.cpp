// SPDX-License-Identifier: MIT
#include "SerialMixedReadback.h"
#include "Oracle.h"
#include "lib_src/elements/qeph/mapped/MixedActivityValues.h"

namespace qeph_activity_test {
TEST(QephMixedActivityHost,EverySelectedScalarMatchesCompleteFrozenReader) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  for (auto law : {fe::ShellSectionLaw::LayeredLaw44Nip3,fe::ShellSectionLaw::LayeredLaw1Nip3,
      fe::ShellSectionLaw::RigidSkin,fe::ShellSectionLaw::Law44Nip1,fe::ShellSectionLaw::Unspecified}) {
    for (unsigned field = 0; field < 46; ++field) {
      std::vector<fe::ShellBatchSectionState> plastic(1);
      std::vector<fe::sections::ShellLayeredLaw1History> elastic(1);
      auto& p = plastic[0];
      if (field < 21) {
        const auto point = field / 7, component = field % 7;
        if (component < 5) p.history.point[point].stress[component] = nan;
        else if (component == 5) p.history.point[point].plastic_strain = nan;
        else p.history.point[point].filtered_rate_per_s = nan;
      } else if (field < 29) {
        double* diagnostics[]{&p.diagnostics.plastic_work_density_increment,&p.diagnostics.maximum_plastic_strain,
          &p.diagnostics.mean_plastic_strain,&p.diagnostics.minimum_tangent_ratio,&p.diagnostics.mean_tangent_ratio,
          &p.diagnostics.mean_yield_before_pa,&p.diagnostics.last_point_yield_before_pa,&p.cumulative_plastic_work_J};
        *diagnostics[field-21] = nan;
      } else if (field < 44) elastic[0].point[(field-29)/5].stress[(field-29)%5] = nan;
      else if (field == 44) p.history.point[0].plastic_strain = -0.;
      frozen_mixed::MixedHostStorage serial;
      serial.Bind(plastic,elastic);
      frozen_mixed::ShellBatchPlasticityBinding source{{law}};
      const auto expected = serial.Read(0,1,nullptr,source);
      const auto actual = m::CheckMixedActivity(law,plastic[0],elastic[0]);
      EXPECT_EQ(actual == m::MixedActivityError::None,
          expected.status == fe::shell_batch_plasticity_detail::SetupStatus::Success);
      if (std::strcmp(expected.message,"Nonfinite elastic section history") == 0)
        EXPECT_EQ(actual,m::MixedActivityError::Elastic);
      if (std::strcmp(expected.message,"Nonfinite plastic section history") == 0)
        EXPECT_EQ(actual,m::MixedActivityError::Plastic);
      if (std::strcmp(expected.message,"One-point history is unavailable") == 0)
        EXPECT_EQ(actual,m::MixedActivityError::OnePoint);
      if (std::strcmp(expected.message,"Unsupported section readback law") == 0)
        EXPECT_EQ(actual,m::MixedActivityError::Unsupported);
      if (actual == m::MixedActivityError::None)
        EXPECT_EQ(serial.output_[0].law(),law);
    }
  }
}

TEST(QephMixedActivityHost,ParentPriorityAndRawRolesPreserveAvailability) {
  using E = m::MixedActivityError;
  EXPECT_LT(m::MixedActivityKey(2,E::Unsupported),m::MixedActivityKey(3,E::Elastic));
  EXPECT_LT(m::MixedActivityKey(524287,E::Unsupported),UINT32_MAX);
  for (unsigned raw = 0; raw < 256; ++raw) {
    const bool valid = raw == unsigned(fe::ShellSectionLaw::GlobalLaw1Npt0) ||
        raw == unsigned(fe::ShellSectionLaw::LayeredLaw44Nip3) ||
        raw == unsigned(fe::ShellSectionLaw::LayeredLaw1Nip3) || raw == unsigned(fe::ShellSectionLaw::RigidSkin);
    EXPECT_EQ(m::ValidMixedActivityRole(static_cast<std::uint8_t>(raw)),valid);
  }
  // Neither an unselected NaN nor a negative finite scalar is newly rejected.
  fe::ShellBatchSectionState plastic;
  fe::sections::ShellLayeredLaw1History elastic;
  plastic.cumulative_plastic_work_J = -1;
  EXPECT_EQ(m::CheckMixedActivity(fe::ShellSectionLaw::LayeredLaw44Nip3,plastic,elastic),E::None);
  plastic.history.point[2].stress[4] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(m::CheckMixedActivity(fe::ShellSectionLaw::LayeredLaw1Nip3,plastic,elastic),E::None);
  elastic.point[1].stress[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(m::CheckMixedActivity(fe::ShellSectionLaw::RigidSkin,plastic,elastic),E::None);
  // Baseline global LAW1 has no layered material-point state to consume.
  EXPECT_EQ(m::CheckMixedActivity(fe::ShellSectionLaw::GlobalLaw1Npt0,plastic,elastic),E::None);
}

TEST(QephMixedActivityHost,FreshRoleBytesMatchTypedAgreementAndKeepErrorOrder) {
  qt_mapped_test::Fixture fixture;
  qt_mapped_test::MakeSkinFixture(fixture);
  const auto& catalog = *fixture.Physical().catalog();
  const std::vector<fe::ShellBatchLayeredSection> sections{
      fe::ShellBatchLayeredSection::RigidSkin(),fe::ShellBatchLayeredSection::Plastic({})};
  std::vector<std::uint8_t> roles;
  for (const auto& section : sections) roles.push_back(static_cast<std::uint8_t>(section.law()));
  const auto valid_roles = roles;
  std::uint8_t force[]{1,1},failure[]{1,1};
  for (unsigned fault = 0; fault < 4; ++fault) {
    auto typed = sections;
    roles = valid_roles;
    force[0] = fault >= 2 ? 0 : 1;
    if (fault == 1 || fault == 3) {
      typed[1] = fe::ShellBatchLayeredSection::RigidSkin();
      roles[1] = static_cast<std::uint8_t>(typed[1].law());
    }
    const auto active = [&](std::size_t i) { return force[i]; };
    const auto failed = [&](std::size_t i) { return failure[i]; };
    SameReport(m::ValidateRoleActivity(catalog,roles.data(),roles.size(),active,failed),
        m::ValidateSectionActivity(catalog,typed.data(),typed.size(),active,failed));
  }
}
} // namespace qeph_activity_test
