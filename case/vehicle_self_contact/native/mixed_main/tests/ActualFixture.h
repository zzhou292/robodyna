#pragma once
#include "../../PostGapmMainSource.h"
#include "../../mixed_interface/tests/ActualFixture.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
inline const Mixed& ActualMixedSource() {
    static const auto source=[] {
        const auto made=Mixed::Prepare(mixed_interface::test::ActualInitialSource());
        output::Require(made.report.status==mixed_interface::Status::Ready&&made.source,
            "Post-GAPM actual fixture requires complete mixed source");return *made.source;
    }();return source;
}
inline const GapOperands& ActualGapOperands() {
    static const auto source=[] {
        const auto made=GapOperands::Prepare(ActualMixedSource().initial().context());
        output::Require(made.report.status==gap_operands::Status::Ready&&made.source,
            "Post-GAPM actual fixture requires genuine complete gap operands");return *made.source;
    }();return source;
}
inline std::string ActualCombine() {
    const auto* path=std::getenv("ROBO_SELF_CONTACT_COMBINE_MEMBER");
    output::Require(path&&*path,"Post-GAPM actual fixture lacks the authenticated combine member");
    return output::ReadBounded(path,modelio::self_contact::Limits{}.combine_member_bytes);
}
inline const PostGapmMainSource& ActualPostGapmSource() {
    static const auto source=[] {
        const auto made=PostGapmMainSource::Prepare(ActualMixedSource(),ActualGapOperands(),ActualCombine());
        output::Require(made.report.status==Status::Ready&&made.source,made.report.reason.c_str());
        return *made.source;
    }();return source;
}
}
