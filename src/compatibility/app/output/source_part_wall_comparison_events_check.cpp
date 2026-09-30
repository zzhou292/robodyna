#include "SourcePartWallComparisonEvents.h"
#include <gtest/gtest.h>

namespace crash::output::wall_comparison {
namespace {
IntervalPoint Point(std::uint64_t epoch,unsigned refinement=1) {
    IntervalPoint p;p.epoch=epoch;p.time=epoch*BaseStep/refinement;
    p.impulse=.3;p.momentum_allowance=1e-12;p.strictly_separated_nodes=117;return p;
}
IntervalPoint ContactPoint(std::uint64_t epoch,unsigned refinement=1) {
    auto p=Point(epoch,refinement);p.active_nodes=1;p.strictly_separated_nodes=116;
    p.reaction=4;p.reaction_error=1e-9;p.potential=.01;return p;
}
TEST(SourceWallComparisonEvents, EndpointsProveNMinusOneIntervalsAtEveryRefinement) {
    for(unsigned ratio:{1u,2u,4u}) {
        EventTracker tracker(ratio,.25L,1e-15);tracker.Observe(ContactPoint(1,ratio));
        for(std::uint64_t epoch=2;epoch<=1+128*ratio;++epoch)tracker.Observe(Point(epoch,ratio));
        EXPECT_FALSE(tracker.events().rebound.observed);
        tracker.Observe(Point(2+128*ratio,ratio));ASSERT_TRUE(tracker.events().rebound.observed);
        EXPECT_EQ(tracker.events().terminal_separation_start,2u);
        EXPECT_EQ(tracker.events().rebound.lower,BaseStep/ratio);
        EXPECT_EQ(tracker.events().rebound.upper,2*BaseStep/ratio);
        EXPECT_EQ(tracker.events().onset.uncertainty,1e-15);
    }
}
TEST(SourceWallComparisonEvents, LateRecontactResetsPreviouslyQualifiedRebound) {
    EventTracker tracker(1,.25L,0);tracker.Observe(ContactPoint(1));
    for(unsigned e=2;e<=130;++e)tracker.Observe(Point(e));ASSERT_TRUE(tracker.events().rebound.observed);
    tracker.Observe(ContactPoint(131));EXPECT_FALSE(tracker.events().rebound.observed);
    EXPECT_EQ(tracker.events().terminal_separation_start,0u);EXPECT_EQ(tracker.events().contact_intervals,2u);
    for(unsigned e=132;e<=260;++e)tracker.Observe(Point(e));ASSERT_TRUE(tracker.events().rebound.observed);
    EXPECT_EQ(tracker.events().rebound.upper,132*BaseStep);
    EXPECT_EQ(tracker.events().onset.upper,BaseStep);
}
TEST(SourceWallComparisonEvents, ApproachTouchOrUncertainComCannotBecomeRebound) {
    EventTracker approach(1,.25L,0);
    for(unsigned e=1;e<=140;++e)approach.Observe(Point(e));
    EXPECT_FALSE(approach.events().onset.observed);EXPECT_FALSE(approach.events().rebound.observed);
    EventTracker uncertain(1,.25L,0);uncertain.Observe(ContactPoint(1));
    for(unsigned e=2;e<=140;++e) {
        auto p=Point(e);p.impulse=.25+1e-16;p.impulse_error=1e-14;uncertain.Observe(p);
    }
    EXPECT_FALSE(uncertain.events().rebound.observed);
    EventTracker touch(1,.25L,0);touch.Observe(ContactPoint(1));
    for(unsigned e=2;e<=140;++e) {auto p=Point(e);p.strictly_separated_nodes=116;touch.Observe(p);}
    EXPECT_FALSE(touch.events().rebound.observed);
}
TEST(SourceWallComparisonEvents, MissingIntervalsAndInconsistentCertificatesAreRejected) {
    EventTracker tracker(1,.25L,0);
    EXPECT_THROW(tracker.Observe(ContactPoint(2)),std::runtime_error);
    auto p=ContactPoint(1);p.strictly_separated_nodes=117;EXPECT_THROW(tracker.Observe(p),std::runtime_error);
    p=ContactPoint(1);p.momentum_residual=1;EXPECT_THROW(tracker.Observe(p),std::runtime_error);
    p=ContactPoint(1);p.reaction_error=-1;EXPECT_THROW(tracker.Observe(p),std::runtime_error);
    p=ContactPoint(1);p.reaction=0;EXPECT_THROW(tracker.Observe(p),std::runtime_error);
    EXPECT_EQ(tracker.events().final_epoch,0u);EXPECT_EQ(tracker.events().peak_reaction,0);
    EXPECT_EQ(tracker.events().contact_intervals,0u);
}
} // namespace
} // namespace crash::output::wall_comparison
