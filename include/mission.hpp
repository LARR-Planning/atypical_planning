#pragma once

#include <fstream>
#include <ros/ros.h>
#include <string>
#include <octomap/OcTree.h>

class Mission {
public:
    std::vector<double> startState, goalState;

    bool init(const ros::NodeHandle &nh);
    void update(std::vector<double> startState, std::vector<double> goalState);

private:
    ros::NodeHandle nh;
};

bool Mission::init(const ros::NodeHandle& _nh){
    nh = _nh;
    startState.resize(9);
    goalState.resize(9);

    return true;
}

void Mission::update(std::vector<double> _startState, std::vector<double> _goalState){
    startState = _startState;
    goalState = _goalState;
}