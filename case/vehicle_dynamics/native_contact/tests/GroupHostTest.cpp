#include "case/vehicle_dynamics/native_contact/Group.h"
#include <gtest/gtest.h>
namespace app=crash::cases::vehicle_dynamics::native_contact;
TEST(NativeAppGroup, EmptyOrOversizedInputCannotCreateAnAdapter) {
    std::array<app::GroupInput,2> empty;
    EXPECT_THROW(app::Group::Adopt(std::move(empty),0),std::exception);
    EXPECT_THROW(app::Group::Adopt(std::move(empty),3),std::exception);
}
TEST(NativeAppGroup, UninitializedTransactionsGrantNoGroupAuthority) {
    std::array<app::GroupInput,2> input;
    input[0]={app::Role::Self,std::make_unique<app::native::Transaction>()};
    EXPECT_THROW(app::Group::Adopt(std::move(input),1),std::exception);
}
