#include "Support.h"

namespace crash::cases::vehicle_startup::physical_attachments::test {
TEST(VehiclePhysicalAttachmentsOriginal, LateWitnessCapFailurePreservesAllAcceptedSourceValuesAndRetries) {
    const auto& saved = Actual();
    const auto* before = saved.witnesses().data().witnesses.data();
    Limits limits;
    limits.witnesses.witnesses = saved.witnesses().data().counts.witnesses - 1;
    EXPECT_THROW(VehiclePhysicalAttachments::Prepare(saved.physical(),Post(),limits),std::exception);
    EXPECT_EQ(saved.witnesses().data().witnesses.data(),before);
    EXPECT_EQ(saved.attachments().model().rows().count,11165u);
    ++limits.witnesses.witnesses;
    const auto retry = VehiclePhysicalAttachments::Prepare(saved.physical(),Post(),limits);
    EXPECT_EQ(retry.witnesses().data().witnesses.back().source_element_id,
              saved.witnesses().data().witnesses.back().source_element_id);
    EXPECT_TRUE(retry.attachments().model().domain()->SharesStorage(saved.physical().source_domain().domain()));
    auto copy = saved;
    EXPECT_EQ(copy.witnesses().data().witnesses.data(),before);
    limits.host_bytes = saved.forecast().total_bytes - 1;
    EXPECT_THROW(VehiclePhysicalAttachments::Preflight(saved.physical(),Post(),limits),std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(VehiclePhysicalAttachments::Preflight(saved.physical(),Post(),limits).total_bytes,limits.host_bytes);
}
} // namespace crash::cases::vehicle_startup::physical_attachments::test
