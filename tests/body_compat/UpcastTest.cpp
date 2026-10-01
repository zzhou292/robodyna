#include "chrono/physics/ChBody.h"
#include "robodyna/mbd/RbBody.h"

#include <memory>
#include <typeindex>
#include <type_traits>

#include <gtest/gtest.h>

namespace robodyna::body_compat {
namespace {
static_assert(std::is_same_v<chrono::ChBody, mbd::RbBody>);

template <class Base>
void CheckBase(const char* legacy_base_tag) {
    auto body = std::make_shared<mbd::RbBody>();
    void* complete = body.get();
    auto* expected = static_cast<Base*>(body.get());
    const auto derived_type = std::type_index(typeid(mbd::RbBody*));
    const auto base_type = std::type_index(typeid(Base*));
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(*body)), "ChBody");
    EXPECT_EQ(chrono::ChCastingMap::GetClassnameFromPtrTypeindex(derived_type), "ChBody");
    EXPECT_EQ(chrono::ChCastingMap::GetClassnameFromPtrTypeindex(base_type), legacy_base_tag);
    if constexpr (!std::is_same_v<Base, chrono::ChPhysicsItem>) {
        // Check that the secondary-base cases genuinely require adjustment.
        ASSERT_NE(static_cast<void*>(expected), complete);
    }
    EXPECT_EQ(chrono::ChCastingMap::Convert("ChBody", legacy_base_tag, complete), expected);
    EXPECT_EQ(chrono::ChCastingMap::Convert(derived_type, base_type, complete), expected);
    EXPECT_EQ(chrono::ChCastingMap::Convert("ChBody", base_type, complete), expected);
    EXPECT_EQ(chrono::ChCastingMap::Convert(derived_type, legacy_base_tag, complete), expected);

    const auto erased = std::static_pointer_cast<void>(body);
    const auto check_shared = [&](const std::shared_ptr<void>& converted) {
        ASSERT_NE(converted, nullptr);
        ASSERT_EQ(converted.get(), expected);
        EXPECT_FALSE(body.owner_before(converted));
        EXPECT_FALSE(converted.owner_before(body));
        // The adjusted base pointer must also retain the dynamic body identity.
        const auto recovered = std::dynamic_pointer_cast<mbd::RbBody>(std::static_pointer_cast<Base>(converted));
        EXPECT_EQ(recovered, body);
        EXPECT_FALSE(body.owner_before(recovered));
        EXPECT_FALSE(recovered.owner_before(body));
    };
    check_shared(chrono::ChCastingMap::Convert("ChBody", legacy_base_tag, erased));
    check_shared(chrono::ChCastingMap::Convert(derived_type, base_type, erased));
    check_shared(chrono::ChCastingMap::Convert("ChBody", base_type, erased));
    check_shared(chrono::ChCastingMap::Convert(derived_type, legacy_base_tag, erased));
}

TEST(BodyBaseCasts, PhysicsItemRawAndShared) { CheckBase<chrono::ChPhysicsItem>("ChPhysicsItem"); }
TEST(BodyBaseCasts, MovingFrameRawAndShared) { CheckBase<chrono::ChBodyFrame>("ChBodyFrame"); }
TEST(BodyBaseCasts, ContactableRawAndShared) { CheckBase<chrono::ChContactable>("ChContactable"); }
TEST(BodyBaseCasts, LoadableRawAndShared) { CheckBase<chrono::ChLoadableUVW>("ChLoadableUVW"); }
}  // namespace
}  // namespace robodyna::body_compat
