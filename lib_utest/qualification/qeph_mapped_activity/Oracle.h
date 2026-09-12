// SPDX-License-Identifier: MIT
#pragma once
#include "../qt_mapped/OwnerFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/qeph/mapped/ActivityValues.h"
#include "SerialValidation.h"

namespace qeph_activity_test {
namespace fe = tl::fea;
namespace q = fe::qeph;
namespace m = q::mapped;
struct Histories {
  std::vector<fe::ShellBatchLayeredSection> sections;
  std::vector<fe::ShellBatchFailureState> failures;
  const auto* section_staging() const { return sections.data(); }
  const auto* failure_staging() const { return failures.data(); }
};
// A host oracle, not an owner. Its complete two functions are frozen from the
// old readback; the actual public readback supplies these copied observations.
struct Oracle {
  const fe::ShellPhysicalBinding* physical = nullptr;
  fe::NodalStamp accepted_stamp;
  q::QephBatchConfig config;
  q::batch_detail::Storage header;
  q::batch_detail::Storage* storage = &header;
  std::vector<q::ForceTrial> staging;
  Histories histories;
  Histories* plasticity = &histories;
  unsigned AcceptedSlabIndex() const { return 0; }
  q::BatchReport ReadResults(const q::batch_detail::Slab* slab) const {
    return serial::ValidateMappedResults(*this,slab == &header.slab[0] ? 0 : 1);
  }
  q::BatchReport Compact(unsigned slab) const {
    const auto epoch = accepted_stamp.epoch + (slab ? 1 : 0);
    const auto time = accepted_stamp.time + (slab ? config.owner.fixed_dt : 0);
    std::uint32_t first_invalid = m::NoActivityFailure;
    std::vector<std::uint8_t> active(staging.size(),19);
    for (std::size_t parent = 0; parent < staging.size(); ++parent) {
      fe::ShellSectionLaw law;
      if (!physical->catalog()->Law(fe::ShellBindingFamily::Qeph,parent,&law) ||
          !m::ValidResult(physical->shells()->qeph_reference(parent),staging[parent],time,epoch,
              law == fe::ShellSectionLaw::RigidSkin)) {
        first_invalid = std::min(first_invalid,static_cast<std::uint32_t>(parent));
      } else active[parent] = static_cast<std::uint8_t>(staging[parent].proposed_history.data().active);
    }
    return m::FinishActivity(first_invalid,*physical->catalog(),histories.sections.data(),
        histories.failures.data(),staging.size(),[&](std::size_t parent) { return active[parent]; });
  }
};
inline void SameReport(const q::BatchReport& actual, const q::BatchReport& expected) {
  EXPECT_EQ(actual.status,expected.status);
  EXPECT_EQ(actual.element,expected.element);
  EXPECT_EQ(actual.node,expected.node);
  EXPECT_EQ(actual.element_status,expected.element_status);
  EXPECT_STREQ(actual.message,expected.message);
}
} // namespace qeph_activity_test
