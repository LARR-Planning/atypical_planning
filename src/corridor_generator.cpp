// Configuration
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Octomap
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
//#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Submodule
#include <ecbs_planner.hpp>
#include <corridor.hpp>
#include <result_publisher.hpp>

int main(int argc, char* argv[]) {
    ros::init (argc, argv, "corridor_generator");
    ros::NodeHandle nh( "~" );

    // Set Mission
    Mission mission;
    if(!mission.init(nh)){
        return -1;
    }
    mission.update(std::vector<double>{-7,1.5,0,0,0,0,0,0,0}, std::vector<double>{7,1.5,0,0,0,0,0,0,0});

    // Set ROS Parameters
    Param param;
    if(!param.init(nh)){
        return -1;
    }


    // Submodules
    std::shared_ptr<octomap::OcTree> octree_obj;
    std::shared_ptr<ECBSPlanner> initTraj_obj;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<ResultPublisher> resultPub_obj;

    // Main Loop
    Timer timer_total;
    Timer timer_step;

    octree_obj.reset(new octomap::OcTree(param.package_path + "/worlds/map_reduced_tmp3.bt"));

    timer_total.reset();

    // Step 1: Plan Initial Trajectory
    timer_step.reset();
    initTraj_obj.reset(new ECBSPlanner(octree_obj, mission, param));
    if (!initTraj_obj.get()->update(param.log)) {
        return -1;
    }

    timer_step.stop();
    ROS_INFO_STREAM("Initial Trajectory Planner runtime: " << timer_step.elapsedSeconds());

    // Step 2: Generate SFC, RSFC
    timer_step.reset();

    corridor_obj.reset(new Corridor(initTraj_obj, octree_obj, mission, param));
    if (!corridor_obj.get()->update(param.log)) {
//        return -1;
    }

    timer_step.stop();
    ROS_INFO_STREAM("BoxGenerator runtime: " << timer_step.elapsedSeconds());

    // Publish Planning Result
    resultPub_obj.reset(new ResultPublisher(nh, corridor_obj, initTraj_obj, mission, param));

    ros::Rate rate(20);
    double start_time, current_time;
    start_time = ros::Time::now().toSec();
    while (ros::ok()) {
        current_time = ros::Time::now().toSec() - start_time;
        resultPub_obj.get()->update(current_time);
        resultPub_obj.get()->publish();
    }

    return 0;
}