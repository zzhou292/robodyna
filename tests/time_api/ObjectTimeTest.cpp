#include "chrono/physics/ChObject.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>

namespace {

class Object final : public chrono::ChObj {
  public:
    Object* Clone() const override { return new Object(*this); }
    void Update(double time, chrono::UpdateFlags flags) override {
        ++updates;
        ChObj::Update(time, flags);
    }
    unsigned updates = 0;
};

static_assert(std::is_same_v<decltype(&chrono::ChObj::GetTime), double (chrono::ChObj::*)() const>);
static_assert(std::is_same_v<decltype(&chrono::ChObj::SetTime), void (chrono::ChObj::*)(double)>);

TEST(ObjectTime, BothNamesShareOnlyTheSelectedObjectTimestamp) {
    Object first;
    Object second;
    const chrono::ChObj& read_only = first;
    second.SetChTime(9.0);
    first.SetChTime(1.25);
    EXPECT_DOUBLE_EQ(read_only.GetTime(), 1.25);
    first.SetTime(-2.5);
    EXPECT_DOUBLE_EQ(read_only.GetChTime(), -2.5);
    EXPECT_DOUBLE_EQ(second.GetTime(), 9.0);
    EXPECT_EQ(first.updates, 0u);
    EXPECT_EQ(first.GetVisualModel(), nullptr);

    std::unique_ptr<Object> clone(first.Clone());
    EXPECT_DOUBLE_EQ(clone->GetTime(), -2.5);
    clone->SetTime(4.0);
    EXPECT_DOUBLE_EQ(first.GetTime(), -2.5);

    first.Update(3.5, chrono::UpdateFlags::DYNAMICS);
    EXPECT_EQ(first.updates, 1u);
    EXPECT_DOUBLE_EQ(read_only.GetTime(), 3.5);
    EXPECT_DOUBLE_EQ(read_only.GetChTime(), 3.5);
}

TEST(ObjectTime, ForwardingPreservesAssignmentWithoutValidationOrClamping) {
    Object object;
    object.SetTime(-0.0);
    EXPECT_TRUE(std::signbit(object.GetChTime()));
    object.SetChTime(std::numeric_limits<double>::infinity());
    EXPECT_TRUE(std::isinf(object.GetTime()));
    object.SetTime(std::numeric_limits<double>::quiet_NaN());
    EXPECT_TRUE(std::isnan(object.GetChTime()));
    EXPECT_EQ(object.updates, 0u);
}

}  // namespace
