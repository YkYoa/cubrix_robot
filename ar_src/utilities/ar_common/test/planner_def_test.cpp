#include <gtest/gtest.h>
#include "ar_common/planner_definition.h"


using namespace ar;
using namespace ar::planner;


TEST(PlannerDefinitionTest, PipelineConstants) {
    EXPECT_EQ(PIPELINE_OMPL, "ompl");
    EXPECT_EQ(PIPELINE_PILZ, "pilz");
    EXPECT_EQ(PIPELINE_DEFAULT, PIPELINE_PILZ);
}

TEST(PlannerDefinitionTest, PlannerIds){
    EXPECT_EQ(OMPL_BITRRT, "BiTRRTkConfigDefault");
    EXPECT_EQ(OMPL_RRTC, "RRTConnectkConfigDefault");
    EXPECT_EQ(OMPL_RRTSTAR,"RRTstarkConfigDefault");
    EXPECT_EQ(PILZ_CIRC, "CIRC");
    EXPECT_EQ(PILZ_LINEAR, "LIN");
    EXPECT_EQ(PILZ_PTP, "PTP");
}

TEST(PlannerDefinitionTest, PlannerLists){
    EXPECT_EQ(OMPL_LIST.size(), 3);
    EXPECT_TRUE(OMPL_LIST.find(OMPL_BITRRT) != OMPL_LIST.end());
    EXPECT_TRUE(OMPL_LIST.find(OMPL_RRTC) != OMPL_LIST.end());
    EXPECT_TRUE(OMPL_LIST.find(OMPL_RRTSTAR) != OMPL_LIST.end());

    EXPECT_EQ(PILZ_LIST.size(), 3);
    EXPECT_TRUE(PILZ_LIST.find(PILZ_PTP) != PILZ_LIST.end());
    EXPECT_TRUE(PILZ_LIST.find(PILZ_CIRC) != PILZ_LIST.end());
    EXPECT_TRUE(PILZ_LIST.find(PILZ_LINEAR) != PILZ_LIST.end());
}

TEST(PlannerDefinitionTest, PipelineMapping) {
    // Verify pipeline map structure
    EXPECT_EQ(PIPELINE_LIST.size(), 2);
    EXPECT_TRUE(PIPELINE_LIST.find(PIPELINE_OMPL) != PIPELINE_LIST.end());
    EXPECT_TRUE(PIPELINE_LIST.find(PIPELINE_PILZ) != PIPELINE_LIST.end());
    
    // Verify OMPL planners
    const auto& ompl_planners = PIPELINE_LIST.at(PIPELINE_OMPL);
    EXPECT_TRUE(ompl_planners.find(OMPL_RRTC) != ompl_planners.end());
    EXPECT_TRUE(ompl_planners.find(OMPL_BITRRT) != ompl_planners.end());
    EXPECT_TRUE(ompl_planners.find(OMPL_RRTSTAR) != ompl_planners.end());
    
    // Verify PILZ planners
    const auto& pilz_planners = PIPELINE_LIST.at(PIPELINE_PILZ);
    EXPECT_TRUE(pilz_planners.find(PILZ_PTP) != pilz_planners.end());
    EXPECT_TRUE(pilz_planners.find(PILZ_LINEAR) != pilz_planners.end());
    EXPECT_TRUE(pilz_planners.find(PILZ_CIRC) != pilz_planners.end());
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}