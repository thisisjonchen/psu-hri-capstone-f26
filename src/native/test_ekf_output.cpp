#include "live_ekf.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

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

TEST(EkfOutput, TrackingDiagnosticsCompareTheSameTimeWithoutBridgingOutputGaps) {
    auto fusion=resting(1.05,.31);
    fusion.state[5]=-.03;
    const std::deque<pals::VerticalOutputSample> history={
        {1.,.30,-.1},{1.1,.32,.1},{1.4,.8,.4}};
    std::array<double,2> error;
    ASSERT_TRUE(pals::vertical_tracking_error(fusion,history,error));
    EXPECT_NEAR(0.,error[0],1.e-12);
    EXPECT_NEAR(-.03,error[1],1.e-12);
    fusion.stamp=1.1;
    ASSERT_TRUE(pals::vertical_tracking_error(fusion,history,error));
    EXPECT_NEAR(-.01,error[0],1.e-12);
    EXPECT_NEAR(-.13,error[1],1.e-12);
    for (double stamp:{.9,1.2,1.5}) {
        fusion.stamp=stamp;
        EXPECT_FALSE(pals::vertical_tracking_error(fusion,history,error));
        EXPECT_TRUE(std::isnan(error[0]));
        EXPECT_TRUE(std::isnan(error[1]));
    }
    EXPECT_FALSE(pals::vertical_tracking_error(fusion,{},error));
}

TEST(EkfOutput, RangeDiagnosticsRetainAttemptTimeAndReportAcceptedAndRejectedUpdates) {
    pals::Profile p{};
    p.acceleration_scale=9.8/2048.; p.gyro_scale=.001; p.delay=.35;
    p.imu_rotation=p.camera_rotation=pals::Rotation::eye();
    p.range_beam=pals::Vec(0.,0.,-1.);
    pals::LiveEKF ekf(p);
    std::int64_t ns=10000000000LL;
    auto step=[&]() {
        ns+=10000000LL;
        ekf.imu(ns,{0.,.01,pals::Quaternion(0.,0.,0.,1.),pals::Vec(0.,0.,.098)},ns/1.e9);
    };
    for (int i=0;i<950;++i) {
        ekf.mode("DISARMED",ns/1.e9);
        if (i%10==0) {
            ekf.range(ns,.05,.01,3.);
            ekf.uwb(ns,0.,0.,.0004);
        }
        step();
    }
    ASSERT_TRUE(ekf.snapshot().initialized);
    ekf.mode("FLYING",ns/1.e9);
    for (int i=0;i<40;++i) step();
    const auto before=ekf.snapshot();
    const auto accepted_ns=ns;
    ekf.range(ns,.06,.01,3.);
    for (int i=0;i<40;++i) step();
    const auto accepted=ekf.snapshot();
    EXPECT_EQ(before.range_correction.sequence+1,accepted.range_correction.sequence);
    EXPECT_NEAR((accepted_ns-accepted.origin_ns)/1.e9,accepted.range_correction.stamp,1.e-9);
    EXPECT_NEAR(.01,accepted.range_correction.innovation,.001);
    EXPECT_TRUE(accepted.range_correction.accepted);
    EXPECT_FALSE(accepted.range_correction.ground_reset);

    const auto rejected_ns=ns;
    ekf.range(ns,2.5,.01,3.);
    for (int i=0;i<40;++i) step();
    const auto rejected=ekf.snapshot();
    EXPECT_EQ(accepted.range_correction.sequence+1,rejected.range_correction.sequence);
    EXPECT_NEAR((rejected_ns-rejected.origin_ns)/1.e9,rejected.range_correction.stamp,1.e-9);
    EXPECT_GT(rejected.range_correction.innovation,2.);
    EXPECT_FALSE(rejected.range_correction.accepted);
    EXPECT_FALSE(rejected.range_correction.ground_reset);
    EXPECT_EQ(accepted.range_aid,rejected.range_aid);
    EXPECT_LT(rejected.filter.position()[2],.1);

    for (int i=0;i<10;++i) step();
    const auto unchanged=ekf.snapshot();
    EXPECT_EQ(rejected.range_correction.sequence,unchanged.range_correction.sequence);
    EXPECT_EQ(rejected.range_correction.stamp,unchanged.range_correction.stamp);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
