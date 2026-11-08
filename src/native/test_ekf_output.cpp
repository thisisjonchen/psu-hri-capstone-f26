#include "live_ekf.h"
#include <gtest/gtest.h>
#include <algorithm>

namespace {
pals::Filter resting(double stamp, double height=.05) {
    pals::Filter f;
    f.initialize(0.,0.,.01,height,pals::Quaternion(0.,0.,0.,1.),pals::Vec(),pals::Vec());
    f.stamp=stamp;
    return f;
}
std::vector<pals::Imu> stationary_imu(double end, int count) {
    std::vector<pals::Imu> samples;
    for (int i=count-1;i>=0;--i)
        samples.push_back({end-i*.01,.01,pals::Quaternion(0.,0.,0.,1.),pals::Vec(0.,0.,.098)});
    return samples;
}
}

TEST(EkfOutput, ResumesAfterOutputGapWithoutRequiringExpiredHistory) {
    const auto previous=resting(1.);
    for (double gap:{.6,1.2,3.}) {
        // Fusion continued during the output pause; only the newest IMU history remains.
        auto estimate=resting(1.+gap-.3,.25);
        const auto intervals=stationary_imu(1.+gap,std::min(100,static_cast<int>(gap*100+.5)));
        ASSERT_TRUE(estimate.project(intervals));
        std::array<double,2> vertical={{.05,-.2}};
        ASSERT_TRUE(pals::vertical_output(estimate,&previous,intervals,vertical));
        EXPECT_NEAR(.25,vertical[0],1.e-9);
        EXPECT_NEAR(0.,vertical[1],1.e-9);

        // The next sample resumes normal smoothing, without inventing climb from a height correction.
        const auto resumed=estimate;
        const auto next=stationary_imu(estimate.stamp+.01,1);
        ASSERT_TRUE(estimate.project(next));
        estimate.state[2]+=.1;
        ASSERT_TRUE(pals::vertical_output(estimate,&resumed,next,vertical));
        EXPECT_GT(vertical[0],.25);
        EXPECT_LT(vertical[0],.35);
        EXPECT_NEAR(0.,vertical[1],1.e-9);
    }
}

TEST(EkfOutput, ShortGapPreservesSmoothingAndInvalidIntervalsStillFail) {
    const auto previous=resting(1.);
    auto estimate=previous;
    auto intervals=stationary_imu(1.02,2);
    ASSERT_TRUE(estimate.project(intervals));
    estimate.state[2]=.15;
    std::array<double,2> vertical={{.05,0.}};
    ASSERT_TRUE(pals::vertical_output(estimate,&previous,intervals,vertical));
    EXPECT_GT(vertical[0],.05);
    EXPECT_LT(vertical[0],.15);
    EXPECT_NEAR(0.,vertical[1],1.e-9);
    intervals.front().duration=.005; // Missing half an IMU interval must not be hidden by recovery.
    EXPECT_FALSE(pals::vertical_output(estimate,&previous,intervals,vertical));
}

TEST(EkfOutput, RecoveryDoesNotMakeStaleRangeOrOutputReady) {
    pals::Snapshot s;
    s.initialized=true; s.origin_ns=1000000000LL; s.delay=.3;
    s.filter=resting(2.); s.range_aid=1.7; s.horizontal_aid=1.7;
    std::string reason;
    EXPECT_TRUE(s.ready(3.,s.filter,reason));
    EXPECT_FALSE(s.ready(3.6,s.filter,reason));
    s.range_aid=2.3; s.horizontal_aid=2.3;
    EXPECT_FALSE(s.ready(3.6,s.filter,reason));
    s.filter.stamp=2.6;
    EXPECT_TRUE(s.ready(3.6,s.filter,reason));
    s.filter.state[2]=std::numeric_limits<double>::quiet_NaN();
    std::array<double,2> vertical={{0.,0.}};
    EXPECT_FALSE(pals::vertical_output(s.filter,nullptr,{},vertical));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
