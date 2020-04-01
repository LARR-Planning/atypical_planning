#pragma once
#include <GlobalPlanner.h>
#include <RosWrapper.h>

class PlannerWrapper{
public:
    void run();

private:
    GlobalPlanner global_planner;
//    LocalPlanner local_planner; //TODO: implement local planner
    RosWrapper ros_wrapper;
};

