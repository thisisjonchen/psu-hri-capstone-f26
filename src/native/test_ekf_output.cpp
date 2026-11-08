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

TEST(EkfOutput, FreshRangeCorrectsProjectionWithoutChangingItsSeed) {
    pals::Profile profile{}; profile.range_beam=pals::Vec(0.,0.,-1.);
    const auto seed=resting(1.,.75);
    auto imu=stationary_imu(1.35,35);
    for (auto& sample:imu) sample.delta_velocity[2]+=.003;
    const std::vector<pals::RangeSample> ranges={{1.105,.75,0.,4.},{1.205,.75,0.,4.},{1.305,.75,0.,4.}};
    auto baseline=seed,corrected=seed,repeat=seed;
    ASSERT_TRUE(baseline.project(imu));
    ASSERT_TRUE(corrected.project(imu,ranges,profile));
    ASSERT_TRUE(repeat.project(imu,ranges,profile));
    EXPECT_LT(std::abs(corrected.state[2]-.75),std::abs(baseline.state[2]-.75));
    EXPECT_LT(std::abs(corrected.state[5]),std::abs(baseline.state[5]));
    EXPECT_EQ(corrected.state,repeat.state);
    EXPECT_EQ(corrected.covariance,repeat.covariance);
    EXPECT_DOUBLE_EQ(seed.stamp,1.);
    EXPECT_DOUBLE_EQ(seed.state[2],.75);
    EXPECT_NEAR(corrected.stamp,1.35,1.e-12);
    cv::Mat p(15,15,CV_64F,corrected.covariance.data()),eigenvalues;
    ASSERT_TRUE(cv::eigen(p,eigenvalues));
    EXPECT_GE(eigenvalues.at<double>(14),-1.e-12);
}

TEST(EkfOutput, ProjectionSkipsInvalidOldDuplicateAndFutureRanges) {
    pals::Profile profile{}; profile.range_beam=pals::Vec(0.,0.,-1.);
    const auto seed=resting(1.,.75);
    const auto imu=stationary_imu(1.3,30);
    auto baseline=seed,ignored=seed;
    ASSERT_TRUE(baseline.project(imu));
    ASSERT_TRUE(ignored.project(imu,{{.9,.4,0.,4.},{1.,.4,0.,4.},
        {1.1,std::numeric_limits<double>::quiet_NaN(),0.,4.},{1.2,5.,0.,4.},{1.4,.4,0.,4.}},profile));
    EXPECT_EQ(baseline.state,ignored.state);
    EXPECT_EQ(baseline.covariance,ignored.covariance);
    auto once=seed,duplicates=seed;
    ASSERT_TRUE(once.project(imu,{{1.105,.8,0.,4.}},profile));
    ASSERT_TRUE(duplicates.project(imu,{{1.105,.8,0.,4.},{1.105,.6,0.,4.},{1.04,.6,0.,4.}},profile));
    EXPECT_EQ(once.state,duplicates.state);
    EXPECT_EQ(once.covariance,duplicates.covariance);
    auto rebased=once;
    const auto next=stationary_imu(1.31,1);
    ASSERT_TRUE(once.project(next));
    ASSERT_TRUE(rebased.project(next,{{1.105,.8,0.,4.}},profile));
    EXPECT_EQ(once.state,rebased.state);
    EXPECT_EQ(once.covariance,rebased.covariance);
    auto rejected=seed;
    ASSERT_TRUE(rejected.project(imu,{{1.105,3.9,0.,4.}},profile));
    EXPECT_NEAR(rejected.state[2],.75,1.e-10);
    EXPECT_NEAR(rejected.state[5],0.,1.e-10);
}

TEST(EkfOutput, ProjectionUsesSharedRangeGeometryAndPreservesImuFailure) {
    pals::Profile profile{}; profile.range_beam=pals::Vec(0.,0.,-1.);
    profile.range_offset=pals::Vec(.03,0.,-.02);
    auto seed=resting(1.,.75);
    seed.state[7]=std::sin(.1); seed.state[9]=std::cos(.1);
    const auto imu=stationary_imu(1.1,10);
    auto expected=seed,actual=seed;
    ASSERT_TRUE(expected.project(imu));
    ASSERT_TRUE(expected.correct_range(.76,profile));
    ASSERT_TRUE(actual.project(imu,{{1.1,.76,0.,4.}},profile));
    EXPECT_EQ(expected.state,actual.state);
    EXPECT_EQ(expected.covariance,actual.covariance);
    auto broken=imu; broken.front().duration=.005;
    actual=seed;
    EXPECT_FALSE(actual.project(broken,{{1.105,.76,0.,4.}},profile));
    EXPECT_EQ(seed.state,actual.state);
    EXPECT_EQ(seed.covariance,actual.covariance);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
