#pragma once

#include <fstream>
#include <iostream>

#include <timer.hpp>

#include <Eigen/Dense>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <math.h>
#include "sp_const.hpp"

class Corridor {
public:
    SFC_t SFC;

    Corridor (std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
              std::shared_ptr<octomap::OcTree> _octomap_obj,
              Mission _mission,
              Param _param)
            : initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              octomap_obj(std::move(_octomap_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        initTraj = initTrajPlanner_obj.get()->initTraj;
        T = initTrajPlanner_obj.get()->T;
        makespan = T.back();
    }

    bool update(bool log){
        return updateObsBox(log);
    }

private:
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<octomap::OcTree> octomap_obj;
    Mission mission;
    Param param;

    initTraj_t initTraj;
    std::vector<double> T;
    double makespan;



    bool isObstacleInBox(const std::vector<double>& box, double margin){
        for(octomap::OcTree::leaf_bbx_iterator it = octomap_obj->begin_leafs_bbx(octomap::point3d(box[0], box[1], box[2]),
                                                                                 octomap::point3d(box[3], box[4], box[5])),
                    end=octomap_obj->end_leafs_bbx(); it!= end; ++it)
        {
            if (octomap_obj->isNodeOccupied(*it)) {
                return true;
            }
        }

        return false;
    }


    bool isBoxInBoundary(const std::vector<double>& box){
        return box[0] >= param.world_x_min &&
               box[1] >= param.world_y_min &&
               box[2] >= param.world_z_min &&
               box[3] <= param.world_x_max &&
               box[4] <= param.world_y_max;
               box[5] <= param.world_z_max;
    }

    bool isPointInBox(const octomap::point3d& point,
                      const std::vector<double>& box){
        return point.x() >= box[0] - SP_EPSILON &&
               point.y() >= box[1] - SP_EPSILON &&
               point.z() >= box[2] - SP_EPSILON &&
               point.x() <= box[3] + SP_EPSILON &&
               point.y() <= box[4] + SP_EPSILON &&
               point.z() <= box[5] + SP_EPSILON;
    }

    bool isBoxInBox(const std::vector<double>& box1,
                      const std::vector<double>& box2){
        return box1[0] <= box2[3] &&
               box1[1] <= box2[4] &&
               box1[2] <= box2[5] &&
               box2[0] <= box1[3] &&
               box2[1] <= box1[4] &&
               box2[2] <= box1[5];
    }

    void expand_box(std::vector<double>& box, double margin) {
        std::vector<double> box_cand, box_update;
        std::vector<int> axis_cand{0, 1, 3, 4};

        int i = -1;
        int axis;
        while (!axis_cand.empty()) {
            box_cand = box;
            box_update = box;

            //check update_box only! update_box + current_box = cand_box
            while ( !isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update) ) {
                i++;
                if(i >= axis_cand.size()){
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                box = box_cand;
                box_update = box_cand;

                //expand cand_box and get updated part of box(update_box)
                if (axis < 3) {
                    box_update[axis+3] = box_cand[axis];
                    if (axis == 2){
                        box_cand[axis] = box_cand[axis] - param.box_z_res;
                    }
                    else {
                        box_cand[axis] = box_cand[axis] - param.box_xy_res;
                    }
                    box_update[axis] = box_cand[axis];
                }
                else{
                    box_update[axis-3] = box_cand[axis];
                    if (axis == 5){
                        box_cand[axis] = box_cand[axis] + param.box_z_res;
                    }
                    else{
                        box_cand[axis] = box_cand[axis] + param.box_xy_res;
                    }
                    box_update[axis] = box_cand[axis];
                }
            }
            axis_cand.erase(axis_cand.begin() + i);
            if(i > 0) {
                i--;
            }
            else{
                i = axis_cand.size()-1;
            }
        }

        for(int j=0; j<2; j++) box[j] += margin;
        for(int j=3; j<5; j++) box[j] -= margin;
    }

    bool updateObsBox(bool log){
        double x_next, y_next, z_next, dx, dy, dz;

        Timer timer1;


        std::vector<double> box_prev{0,0,0,0,0,0};

        for (int i = 0; i < initTraj.size()-1; i++) {
            auto state = initTraj[i];
            double x = state.x();
            double y = state.y();
            double z = state.z();

            std::vector<double> box;
            auto state_next = initTraj[i+1];
            x_next = state_next.x();
            y_next = state_next.y();
            z_next = state_next.z();

            if(isPointInBox(octomap::point3d(x_next, y_next, z_next), box_prev)){
                continue;
            }

            // Initialize box
            box.emplace_back(round(std::min(x,x_next) / param.box_xy_res) * param.box_xy_res);
            box.emplace_back(round(std::min(y,y_next) / param.box_xy_res) * param.box_xy_res);
//            box.emplace_back(round(std::min(z,z_next) / param.box_z_res) * param.box_z_res);
            box.emplace_back(param.world_z_min);
            box.emplace_back(round(std::max(x,x_next) / param.box_xy_res) * param.box_xy_res + param.box_xy_res);
            box.emplace_back(round(std::max(y,y_next) / param.box_xy_res) * param.box_xy_res + param.box_xy_res);
//            box.emplace_back(round(std::max(z,z_next) / param.box_z_res) * param.box_z_res + param.box_z_res);
            box.emplace_back(param.world_z_max);
//            box.emplace_back(round(std::max(x,x_next) / param.box_xy_res) * param.box_xy_res);
//            box.emplace_back(round(std::max(y,y_next) / param.box_xy_res) * param.box_xy_res);
//            box.emplace_back(round(std::max(z,z_next) / param.box_z_res) * param.box_z_res); //TODO: consider this



            if (isObstacleInBox(box, param.agent_xy_size)) {
                ROS_ERROR("BoxGenerator: Invalid initTraj. Obstacle invades initTraj");
                return false;
            }
            expand_box(box, param.agent_xy_size);

            SFC.emplace_back(std::make_pair(box, -1));

            box_prev = box;
        }

        // Generate box time segment
        int box_max = SFC.size();
        int path_max = initTraj.size();
        Eigen::MatrixXd box_log = Eigen::MatrixXd::Zero(box_max, path_max);

        for(int i=0; i<box_max; i++){
            for(int j=0; j<path_max; j++){
                if(isPointInBox(initTraj[j], SFC[i].first)){
                    if (j==0){
                        box_log(i,j) = 1;
                    }
                    else{
                        box_log(i,j) = box_log(i,j-1) + 1;
                    }
                }
            }
        }

        std::cout<< box_log <<std::endl;

        int box_iter = 0;
        for(int path_iter = 0; path_iter < path_max; path_iter++) {
            if (box_iter == box_max-1){
                if(box_log(box_iter, path_iter) > 0){
                    continue;
                }
                else{
                    box_iter--;
                }
            }
            if(box_log(box_iter, path_iter) > 0 && box_log(box_iter+1, path_iter) > 0){
                int count = 1;
                while(path_iter+count < path_max && box_log(box_iter, path_iter+count)>0
                      && box_log(box_iter+1, path_iter+count)>0){
                    count++;
                }
                double obs_index = path_iter+count/2;
                SFC[box_iter].second = T[obs_index];

                path_iter = path_iter+count/2;
                box_iter++;
            }
            else if(box_log(box_iter, path_iter) == 0){
                box_iter--;
                path_iter--;
            }
        }
        SFC[box_max-1].second = makespan;


        timer1.stop();
        ROS_INFO_STREAM("SFC runtime: " << timer1.elapsedSeconds());
        return true;
    }
};