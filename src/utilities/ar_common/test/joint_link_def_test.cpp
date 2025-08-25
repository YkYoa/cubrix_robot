#include "ar_common/joint_link_definition.h"
#include <gtest/gtest.h>

using namespace ar;

TEST(JointLinkDefinitionTest, StringConvertion)
{
    EXPECT_EQ(S("test"), "test");
}

TEST(JointLinkDefinitionTest, JointGroups){
    EXPECT_EQ(jointgroup::ARM, "arm");
}

TEST(JointLinkDefinitionTest, ArmJoints) {
    EXPECT_EQ(joint::AJ1, "Arm_joint1");
    EXPECT_EQ(joint::AJ2, "Arm_joint2");
    EXPECT_EQ(joint::AJ3, "Arm_joint3");
    EXPECT_EQ(joint::AJ4, "Arm_joint4");
    EXPECT_EQ(joint::AJ5, "Arm_joint5");
    EXPECT_EQ(joint::AJ6, "Arm_joint6");
}


TEST(JointLinkDefinitionTest, ArmLinks) {
    EXPECT_EQ(joint::AL1, "Arm_link1");
    EXPECT_EQ(joint::AL2, "Arm_link2");
    EXPECT_EQ(joint::AL3, "Arm_link3");
    EXPECT_EQ(joint::AL4, "Arm_link4");
    EXPECT_EQ(joint::AL5, "Arm_link5");
    EXPECT_EQ(joint::AL6, "Arm_link6");
}

TEST(JointLinkDefinitionTest, EndEffectorLink){
    EXPECT_EQ(link::ENDEFF_ARM, "Arm_endlink");
}

TEST(JointLinkDefinitionTest, JointGroupsVector) {
    const std::vector<std::string> expected = 
        {"Arm_joint1", "Arm_joint2", "Arm_joint3", "Arm_joint4", "Arm_joint5", "Arm_joint6"};

    EXPECT_EQ(joint::ARM, expected);
}

TEST(JointLinkDefinitionTest, AnonymousNamespaceContent) {
    EXPECT_EQ(JointGroups.size(), 1);
    EXPECT_EQ(JointGroups[0], "arm");
    
    EXPECT_TRUE(GroupToJoints.empty());
    EXPECT_TRUE(GroupToEffs.empty());
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}