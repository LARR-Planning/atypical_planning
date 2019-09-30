#pragma once

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-4
#define SP_INFINITY         1e+9

#define SP_PT_RBP            0
#define SP_PT_SCP            1

#define SP_IPT_ECBS          0

#include <octomap/OcTree.h>
typedef std::vector<octomap::point3d> initTraj_t;
typedef std::vector<std::pair<std::vector<double>, double>> SFC_t;

