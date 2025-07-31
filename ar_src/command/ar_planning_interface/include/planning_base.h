#pragma once


#include "moveit/planning_interface/planning_interface.h"



namespace ar_planning_interface
{



class PlanningBase : public PlanningPipeline
{
    struct PlanningBaseParams
    {
        std::string eef_link = "";
        std::string planning_group = "";
        
    };
    public:



    private:




    protected:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;





};






}// namespace ar_planning_interface