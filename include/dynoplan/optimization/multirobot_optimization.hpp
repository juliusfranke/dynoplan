#pragma once

#include "dynobench/motions.hpp"
#include <bits/stdc++.h>
/* #include <map> */
/* #include <optional> */
#include <string>
// #include "dynobench/motions.hpp"
#include <Eigen/Dense>
#include <dynobench/multirobot_trajectory.hpp>
#include <yaml-cpp/yaml.h>

bool execute_optimizationMultiRobot(const YAML::Node &env,
                                    const std::string &initial_guess_file,
                                    const std::string &output_file,
                                    const std::string &dynobench_base,
                                    bool sum_robots_cost,
                                    MultiRobotTrajectory *solution = nullptr);

bool execute_optimizationMetaRobot(
    dynobench::Problem &problem,
    MultiRobotTrajectory &multi_robot_initial_guess,
    MultiRobotTrajectory &multi_robot_out, const std::string &dynobench_base,
    bool sum_robots_cost);
