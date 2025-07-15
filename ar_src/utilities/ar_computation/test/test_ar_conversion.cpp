#include <gtest/gtest.h>
#include "ar_conversion.h"
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <vector>

using namespace ar_computation;

TEST(ArConversionTest, DegRadConversion) {
    EXPECT_NEAR(degToRad(180.0), M_PI, 1e-9);
    EXPECT_NEAR(radToDeg(M_PI), 180.0, 1e-9);
}

TEST(ArConversionTest, VecToPoseAndBack) {
    std::vector<double> vec = {1, 2, 3, 0.1, 0.2, 0.3, 0.4};
    auto pose = vecToPose(vec);
    EXPECT_DOUBLE_EQ(pose.position.x, 1);
    EXPECT_DOUBLE_EQ(pose.position.y, 2);
    EXPECT_DOUBLE_EQ(pose.position.z, 3);
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.1);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.2);
    EXPECT_DOUBLE_EQ(pose.orientation.z, 0.3);
    EXPECT_DOUBLE_EQ(pose.orientation.w, 0.4);
    auto vec2 = poseQuatToVec(pose);
    for (size_t i = 0; i < vec.size(); ++i) {
        EXPECT_DOUBLE_EQ(vec[i], vec2[i]);
    }
}

TEST(ArConversionTest, PoseQuatToPoseMsgs) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1;
    pose.position.y = 2;
    pose.position.z = 3;
    pose.orientation.x = 0.1;
    pose.orientation.y = 0.2;
    pose.orientation.z = 0.3;
    pose.orientation.w = 0.4;
    auto pose_stamped = poseQuatToPoseMsgs(pose);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.position.x, 1);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.position.y, 2);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.position.z, 3);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.orientation.x, 0.1);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.orientation.y, 0.2);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.orientation.z, 0.3);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.orientation.w, 0.4);
}

TEST(ArConversionTest, VecToEigenAndBack) {
    std::vector<double> vec = {1, 2, 3, 4};
    auto eigen_vec = vecToEigenVec(vec);
    EXPECT_EQ(eigen_vec.size(), 4);
    for (int i = 0; i < eigen_vec.size(); ++i) {
        EXPECT_DOUBLE_EQ(eigen_vec[i], vec[i]);
    }
    auto vec2 = eigenVecToVec(eigen_vec);
    EXPECT_EQ(vec2.size(), 4);
    for (size_t i = 0; i < vec2.size(); ++i) {
        EXPECT_DOUBLE_EQ(vec2[i], vec[i]);
    }
}

int main (int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}