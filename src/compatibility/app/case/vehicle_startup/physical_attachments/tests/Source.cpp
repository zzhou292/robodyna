#include "Support.h"
#include "PreparePost.h"

namespace crash::cases::vehicle_startup::physical_attachments::test {
const TiedSearchPostKinChk& Post() {
    static const auto value = [] {
        const auto& source = physical_model::test::Source().source();
        const auto& original = modelio::physical_scope::test::Inputs();
        return PreparePost(source,original.member);
    }();
    return value;
}
} // namespace crash::cases::vehicle_startup::physical_attachments::test
