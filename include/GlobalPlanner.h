#pragma once
#include <vector>
#include <octomap/OcTree.h>

struct Corridor{
    double xl;
    double yl;
    double xu;
    double yu;
    double t_start;
    double t_end;
};

typedef std::vector<Corridor> CorridorSeq;
typedef octomap::OcTree GlobalMap;

class GlobalPlanner{
public:
    GlobalPlanner(GlobalMap global_map, CarState cur_state);
    CorridorSeq get_corridor_seq();

private:
    CorridorSeq corr_seq;
    GlobalMap global_map;
    CarState cur_state;
};
