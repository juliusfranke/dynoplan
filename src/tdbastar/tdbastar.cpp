// #include "dynoplan/dbastar/dbastar.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"

#include <boost/graph/graphviz.hpp>

// #include <flann/flann.hpp>
// #include <msgpack.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>

// #include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/datastructures/NearestNeighbors.h>
// #include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"

// boost stuff for the graph
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include "dynobench/general_utils.hpp"

#include "dynoplan/nigh_custom_spaces.hpp"

namespace dynoplan {

using dynobench::Trajectory;

using dynobench::FMT;

namespace ob = ompl::base;

using Sample = std::vector<double>;
using Sample_ = ob::State;

// nigh interface for OMPL

const char *duplicate_detection_str[] = {"NO", "HARD", "SOFT"};

bool compareAStarNode::operator()(const AStarNode *a,
                                  const AStarNode *b) const {
  // Sort order
  // 1. lowest fScore
  // 2. highest gScore

  // Our heap is a maximum heap, so we invert the comperator function here
  if (a->fScore != b->fScore) {
    return a->fScore > b->fScore;
  } else {
    return a->gScore < b->gScore;
  }
}

void plot_search_tree(std::vector<AStarNode *> nodes,
                      std::vector<Motion> &motions,
                      dynobench::Model_robot &robot, const char *filename) {

  std::cout << "plotting search tree to: " << filename << std::endl;
  std::ofstream out(filename);
  out << "nodes:" << std::endl;
  const std::string indent2 = "  ";
  const std::string indent4 = "    ";
  const std::string indent6 = "      ";
  for (auto &n : nodes) {
    out << indent2 << "-" << std::endl;
    out << indent4 << "x: " << n->state_eig.format(dynobench::FMT) << std::endl;
    out << indent4 << "fScore: " << n->fScore << std::endl;
    out << indent4 << "gScore: " << n->gScore << std::endl;
    out << indent4 << "hScore: " << n->hScore << std::endl;
  }
  
  out << "edges:" << std::endl;

  for (auto &n : nodes) {
    if (n->came_from) {
      out << indent2 << "-" << std::endl;
      out << indent4
          << "from: " << n->came_from->state_eig.format(dynobench::FMT)
          << std::endl;
      out << indent4 << "to: " << n->state_eig.format(dynobench::FMT)
          << std::endl;
      // get the motion

      Eigen::VectorXd offset(robot.get_offset_dim());
      robot.offset(n->came_from->state_eig, offset);
      LazyTraj lazy_traj{.offset = &offset,
                         .robot = &robot,
                         .motion = &motions.at(n->used_motion)};

      dynobench::Trajectory traj;
      traj.states = lazy_traj.motion->traj.states;
      traj.actions = lazy_traj.motion->traj.actions;
      lazy_traj.compute(traj);
      out << indent4 << "traj:" << std::endl;
      for (auto &s : traj.states) {
        out << indent6 << "- " << s.format(dynobench::FMT) << std::endl;
      }
    }
  }
}

void from_solution_to_yaml_and_traj(dynobench::Model_robot &robot,
                                    const std::vector<Motion> &motions,
                                    AStarNode *solution,
                                    const dynobench::Problem &problem,
                                    dynobench::Trajectory &traj_out,
                                    std::ofstream *out) {
  std::vector<const AStarNode *> result;

  CHECK(solution, AT);
  // TODO: check what happens if a solution is a single state?

  const AStarNode *n = solution;
  while (n != nullptr) {
    result.push_back(n);
    n = n->came_from;
  }

  std::reverse(result.begin(), result.end());

  std::cout << "result size " << result.size() << std::endl;

  auto space6 = std::string(6, ' ');
  if (result.size() == 1) {
    // eg. if start is closest state to the goal

    if (out) {
      *out << "  - states:" << std::endl;
      *out << space6 << "- ";
      *out << result.front()->state_eig.format(FMT) << std::endl;
      *out << "    actions: []" << std::endl;
    }
    traj_out.states.push_back(result.front()->state_eig);
  }

  if (out) {
    *out << "  - states:" << std::endl;
  }

  Eigen::VectorXd __tmp(robot.nx);
  Eigen::VectorXd __offset(robot.get_offset_dim());
  for (size_t i = 0; i < result.size() - 1; ++i) {
    const auto node_state = result.at(i)->state_eig;
    const auto &motion = motions.at(result.at(i + 1)->used_motion);
    int take_until = result.at(i + 1)->intermediate_state;
    if (take_until != -1) {
      if (out) {
        *out << std::endl;
        *out << space6 + "# (note: we have stopped at intermediate state) "
             << std::endl;
      }
    }
    if (out) {
      *out << space6 + "# (node_state) " << node_state.format(FMT) << std::endl;
      *out << std::endl;
      *out << space6 + "# motion " << motion.idx << " with cost " << motion.cost
           << std::endl; // debug
      *out << space6 + "# motion first state "
           << motion.traj.states.front().format(FMT) << std::endl;
      *out << space6 + "# motion last state "
           << motion.traj.states.back().format(FMT) << std::endl;
    }
 
    // transform the motion to match the state

    // get the motion
    robot.offset(node_state, __offset);
    if (out) {
      *out << space6 + "# (tmp) " << __tmp.format(FMT) << std::endl;
      *out << space6 + "# (offset) " << __offset.format(FMT) << std::endl;
    };

    auto &traj = motion.traj;
    Trajectory __traj = motion.traj;
    dynobench::TrajWrapper traj_wrap =
        dynobench::Trajectory_2_trajWrapper(__traj);
        
    robot.transform_primitive(__offset, traj.states, traj.actions, traj_wrap);
    std::vector<Eigen::VectorXd> xs = traj_wrap.get_states();
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();

    // TODO: missing additional offset, if any

    double jump = robot.lower_bound_time(node_state, xs.front());
    CSTR_V(node_state);
    CSTR_V(xs.front());
    std::cout << "jump " << jump << std::endl;

    if (out) {
      *out << space6 + "# (traj.states.front) "
           << traj.states.front().format(FMT) << std::endl;
      *out << space6 + "# (xs.front) " << xs.front().format(FMT) << std::endl;
    }

    size_t take_num_states = xs.size();
    if (take_until != -1)
      take_num_states = take_until + 1;

    DYNO_CHECK_LEQ(take_num_states, xs.size(), AT);
    for (size_t k = 0; k < take_num_states; ++k) {
      if (k < take_num_states - 1) {
        // print the state

        if (out) {
          *out << space6 << "- ";
        }
        traj_out.states.push_back(xs.at(k));
      } else if (i == result.size() - 2) {
        if (out) {
          *out << space6 << "- ";
        }
        traj_out.states.push_back(result.at(i + 1)->state_eig);
        // traj_out.states.push_back(xs.at(k)); This was before, fails if I have
        // change the parent of the last node before it goes out of the queue.
      } else {
        if (out) {
          *out << space6 << "# (last state) ";
        }
      }
      if (out) {
        *out << xs.at(k).format(FMT) << std::endl;
      }
    }

    // Continue here!!
    // Just get state + motion
    // skip last, then state... and so on!!!
  }
  if (out) {
    *out << space6 << "# goal state is " << problem.goal.format(FMT)
         << std::endl;
    *out << "    actions:" << std::endl;
  }

  for (size_t i = 0; i < result.size() - 1; ++i) {
    const auto &motion = motions.at(result.at(i + 1)->used_motion);
    int take_until = result.at(i + 1)->intermediate_state;
    if (take_until != -1) {
      if (out) {
        *out << space6 + "# (note: we have stop at intermediate state) "
             << std::endl;
      }
    }

    if (out) {
      *out << space6 + "# motion " << motion.idx << " with cost " << motion.cost
           << std::endl;
    }

    size_t take_num_actions = motion.actions.size();

    if (take_until != -1) {
      take_num_actions = take_until;
    }
    DYNO_CHECK_LEQ(take_num_actions, motion.actions.size(), AT);
    if (out) {
      *out << space6 + "# "
           << "take_num_actions " << take_num_actions << std::endl;
    }

    for (size_t k = 0; k < take_num_actions; ++k) {
      const auto &action = motion.traj.actions.at(k);
      if (out) {
        *out << space6 + "- ";
        *out << action.format(FMT) << std::endl;
        *out << std::endl;
      }
      Eigen::VectorXd x;
      traj_out.actions.push_back(action);
    }
    if (out)
      *out << std::endl;
  }
  DYNO_CHECK_LEQ((result.back()->state_eig - traj_out.states.back()).norm(),
                 1e-6, "");
};

double automatic_delta(double delta_in, double alpha, RobotOmpl &robot,
                       ompl::NearestNeighbors<Motion *> &T_m) {
  Motion fakeMotion;
  fakeMotion.idx = -1;
  auto si = robot.getSpaceInformation();
  fakeMotion.states.push_back(si->allocState());
  std::vector<Motion *> neighbors_m;
  size_t num_desired_neighbors = (size_t)-delta_in;
  size_t num_samples = std::min<size_t>(1000, T_m.size());

  auto state_sampler = si->allocStateSampler();
  double sum_delta = 0.0;
  for (size_t k = 0; k < num_samples; ++k) {
    do {
      state_sampler->sampleUniform(fakeMotion.states[0]);
    } while (!si->isValid(fakeMotion.states[0]));
    robot.setPosition(fakeMotion.states[0], fcl::Vector3d(0, 0, 0));

    T_m.nearestK(&fakeMotion, num_desired_neighbors + 1, neighbors_m);

    double max_delta =
        si->distance(fakeMotion.states[0], neighbors_m.back()->states.front());
    sum_delta += max_delta;
  }
  double adjusted_delta = (sum_delta / num_samples) / alpha;
  std::cout << "Automatically adjusting delta to: " << adjusted_delta
            << std::endl;

  si->freeState(fakeMotion.states.back());

  return adjusted_delta;
}

void filter_duplicates(std::vector<Motion> &motions, double delta, double alpha,
                       RobotOmpl &robot, ompl::NearestNeighbors<Motion *> &T_m,
                       double factor) {

  auto si = robot.getSpaceInformation();
  size_t num_duplicates = 0;
  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.states.push_back(si->allocState());
  std::vector<Motion *> neighbors_m;
  for (const auto &m : motions) {
    if (m.disabled) {
      continue;
    }

    si->copyState(fakeMotion.states[0], m.states[0]);
    T_m.nearestR(&fakeMotion, factor * delta * alpha, neighbors_m);

    for (Motion *nm : neighbors_m) {
      if (nm == &m || nm->disabled) {
        continue;
      }
      double goal_delta = si->distance(m.states.back(), nm->states.back());
      if (goal_delta < factor * delta * (1 - alpha)) {
        nm->disabled = true;
        ++num_duplicates;
      }
    }
  }
  std::cout << "There are " << num_duplicates << " duplicate motions!"
            << std::endl;
}

void __add_state_timed(AStarNode *node,
                       ompl::NearestNeighbors<AStarNode *> *T_n,
                       Time_benchmark &time_bench) {
  assert(node);
  assert(T_n);
  auto out = timed_fun([&] {
    T_n->add(node);
    return 0;
  });
  time_bench.time_nearestNode += out.second;
  time_bench.time_nearestNode_add += out.second;
};

bool check_lazy_trajectory(
    LazyTraj &lazy_traj, dynobench::Model_robot &robot,
    Time_benchmark &time_bench, dynobench::TrajWrapper &tmp_traj,
    Eigen::Ref<Eigen::VectorXd> aux_last_state,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state,
    int *num_valid_states, bool forward) {

  time_bench.time_alloc_primitive += 0; // no memory allocation :)

  // preliminary check only on bounds of last state

  if (robot.transform_primitive_last_state_available) {

    if (forward) {
      robot.transform_primitive_last_state(
          *lazy_traj.offset, lazy_traj.motion->traj.states,
          lazy_traj.motion->traj.actions, aux_last_state);

    } else {
      robot.transform_primitive_last_state_backward(
          *lazy_traj.offset, lazy_traj.motion->traj.states,
          lazy_traj.motion->traj.actions, aux_last_state);
    }

    if (!robot.is_state_valid(aux_last_state))
      return false;
  }

  Stopwatch wacht_tp;
  // forward or backward?

  lazy_traj.compute(tmp_traj, forward, check_state, num_valid_states);

  time_bench.time_transform_primitive += wacht_tp.elapsed_ms();


  Stopwatch watch_check_motion;

  if (check_state) {
    // bounds are check when doing the rollout
    assert(num_valid_states);
    if (*num_valid_states < lazy_traj.motion->traj.states.size()) {
      return false;
    }

  } else {
    // checking backwards is usually faster
    for (size_t i = 0; i < tmp_traj.get_size(); i++) {
      if (!robot.is_state_valid(
              tmp_traj.get_state(tmp_traj.get_size() - 1 - i))) {
        return false;
      }
    }
  }

  time_bench.check_bounds += watch_check_motion.elapsed_ms();

  bool motion_valid;
  auto &motion = lazy_traj.motion;
  time_bench.time_collisions += timed_fun_void([&] {
    if (robot.invariance_reuse_col_shape) {
      Eigen::VectorXd offset = *lazy_traj.offset;
      assert(offset.size() == 2 || offset.size() == 3);
      Eigen::Vector3d __offset;
      if (offset.size() == 2) {
        __offset = Eigen::Vector3d(offset(0), offset(1), 0);
      } else {
        __offset = offset.head<3>();
      }
      assert(motion);
      assert(motion->collision_manager);
      assert(robot.env.get());
      std::vector<fcl::CollisionObject<double> *> objs;
      motion->collision_manager->getObjects(objs);
      motion->collision_manager->shift(__offset);
      fcl::DefaultCollisionData<double> collision_data;
      motion->collision_manager->collide(robot.env.get(), &collision_data,
                                         fcl::DefaultCollisionFunction<double>);
      motion->collision_manager->shift(-__offset);
      motion_valid = !collision_data.result.isCollision();
    } else {
      motion_valid = dynobench::is_motion_collision_free(tmp_traj, robot);
    }
  });

  time_bench.num_col_motions++;

  return motion_valid;
};

void check_goal(dynobench::Model_robot &robot, Eigen::Ref<Eigen::VectorXd> x,
                const Eigen::Ref<const Eigen::VectorXd> &goal,
                dynobench::TrajWrapper &traj_wrapper, double distance_bound,
                size_t num_check_goal, int &chosen_index) {
  x = traj_wrapper.get_state(traj_wrapper.get_size() - 1);
  Eigen::VectorXd intermediate_sol(robot.nx);
  for (size_t nn = 0; nn < num_check_goal; nn++) {
    size_t index_to_check =
        float(nn + 1) / (num_check_goal + 1) * traj_wrapper.get_size();
    if (double d = robot.distance(traj_wrapper.get_state(index_to_check), goal);
        d <= distance_bound) {
      x = traj_wrapper.get_state(index_to_check);
      chosen_index = index_to_check;
      std::cout << "Found a solution with "
                   "intermetidate checks! "
                   "\n"
                << "x:" << x.format(FMT) << " d:" << d << std::endl;
      break;
    }
  }
};

void tdbastar(const dynobench::Problem &problem, Options_tdbastar options_tdbastar,
             Trajectory &traj_out, Out_info_tdb &out_info_tdb, int &robot_id, std::ofstream &out) {
  std::cout << "*** options_tdbastar ***" << std::endl;
  options_tdbastar.print(std::cout);
  std::cout << "***" << std::endl;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotTypes[robot_id] + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);
  load_env(*robot, problem);
  const int nx = robot->nx;

  CHECK(options_tdbastar.motions_ptr,
        "motions should be loaded before calling dbastar");
  std::vector<Motion> &motions = *options_tdbastar.motions_ptr;

  auto check_motions = [&] {
    for (size_t idx = 0; idx < motions.size(); ++idx) {
      if (motions[idx].idx != idx) {
        return false;
      }
    }
    return true;
  };

  assert(check_motions());

  Time_benchmark time_bench;

  // build kd-tree for motion primitives
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  ompl::NearestNeighbors<AStarNode *> *T_n = nullptr;

  if (options_tdbastar.use_nigh_nn) {
    T_m = nigh_factory2<Motion *>(problem.robotTypes[robot_id], robot);
  } else {
    NOT_IMPLEMENTED;
  }

  time_bench.time_nearestMotion += timed_fun_void([&] {
    for (size_t i = 0;
         i < std::min(motions.size(), options_tdbastar.max_motions); ++i) {
      T_m->add(&motions.at(i));
    }
  });

  if (options_tdbastar.use_nigh_nn) {
    T_n = nigh_factory2<AStarNode *>(problem.robotTypes[robot_id], robot);
  } else {
    NOT_IMPLEMENTED;
  }

  Expander expander(robot.get(), T_m,
                    options_tdbastar.alpha * options_tdbastar.delta);

  if (options_tdbastar.alpha <= 0 || options_tdbastar.alpha >= 1) {
    ERROR_WITH_INFO("Alpha needs to be between 0 and 1!");
  }

  if (options_tdbastar.delta < 0) {
    NOT_IMPLEMENTED; // HERE i could compute delta based on desired branching
                     // factor!
  }

  std::shared_ptr<Heu_fun> h_fun = nullptr;
  std::vector<Heuristic_node> heu_map;

  h_fun = std::make_shared<Heu_euclidean>(robot, problem.goals[robot_id]);
  
  // all_nodes manages the memory.
  // c-pointer don't have onwership.
  std::vector<std::unique_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_unique<AStarNode>());

  AStarNode *start_node = all_nodes.at(0).get();
  start_node->gScore = 0;
  start_node->state_eig = problem.starts[robot_id];
  start_node->hScore = h_fun->h(problem.starts[robot_id]);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->came_from = nullptr;
  start_node->is_in_open = true;

  DYNO_DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
  DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");

  auto goal_node = std::make_unique<AStarNode>();
  goal_node->state_eig = problem.goals[robot_id];

  open_t open;
  start_node->handle = open.push(start_node);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.traj.states.push_back(Eigen::VectorXd::Zero(robot->nx));

  AStarNode tmp_node;
  tmp_node.state_eig = Eigen::VectorXd::Zero(robot->nx);

  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goals[robot_id]);

  std::mt19937 g = std::mt19937{std::random_device()()};

  double cost_bound =
      options_tdbastar.maxCost; //  std::numeric_limits<double>::infinity();

  if (options_tdbastar.fix_seed) {
    expander.seed(0);
    g = std::mt19937{0};
    srand(0);
  } else {
    srand(time(0));
  }

  time_bench.time_nearestNode_add +=
      timed_fun_void([&] { T_n->add(start_node); });

  const size_t print_every = 1000;

  double last_f_score = start_node->fScore;
  auto print_search_status = [&] {
    std::cout << "expands: " << time_bench.expands << " open: " << open.size()
              << " best distance: " << best_distance_to_goal
              << " fscore: " << last_f_score << std::endl;
  };

  Stopwatch watch;
  Terminate_status status = Terminate_status::UNKNOWN;

  auto stop_search = [&] {
    if (static_cast<size_t>(time_bench.expands) >=
        options_tdbastar.max_expands) {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }

    if (watch.elapsed_ms() > options_tdbastar.search_timelimit) {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      return true;
    }

    if (open.empty()) {
      status = Terminate_status::EMPTY_QUEUE;
      std::cout << "BREAK search:"
                << "EMPTY_QUEUE" << std::endl;
      return true;
    }

    return false;
  };

  AStarNode *best_node = nullptr;
  std::vector<AStarNode *> closed_list;
  std::vector<AStarNode *> neighbors_n;
  std::vector<Trajectory> expanded_trajs; // for debugging

  const bool debug = false; // Set to true to write save to disk a lot of stuff

  const bool check_intermediate_goal = true;
  const size_t num_check_goal =
      4; // Eg, for n = 4 I check: 1/5 , 2/5 , 3/5 , 4/5

  std::function<bool(Eigen::Ref<Eigen::VectorXd>)> ff =
      [&](Eigen::Ref<Eigen::VectorXd> state) {
        return robot->is_state_valid(state);
      };

  // we allocate a trajectory for the largest motion primitive

  dynobench::TrajWrapper traj_wrapper;
  {
    std::vector<Motion *> motions;
    T_m->list(motions);
    size_t max_traj_size = (*std::max_element(motions.begin(), motions.end(),
                                              [](Motion *a, Motion *b) {
                                                return a->traj.states.size() <
                                                       b->traj.states.size();
                                              }))
                               ->traj.states.size();

    traj_wrapper.allocate_size(max_traj_size, robot->nx, robot->nu);
  }

  Eigen::VectorXd aux_last_state(robot->nx);
  while (!stop_search()) {

    // POP best node in queue
    time_bench.time_queue += timed_fun_void([&] {
      best_node = open.top();
      open.pop();
    });
    last_f_score = best_node->fScore;
    closed_list.push_back(best_node);
    best_node->is_in_open = false;

    if (time_bench.expands % print_every == 0) {
      print_search_status();
    }

    time_bench.expands++;

    // CHECK if best node is close ENOUGH to goal
    double distance_to_goal =
        robot->distance(best_node->state_eig, problem.goals[robot_id]);

    if (distance_to_goal < best_distance_to_goal) {
      best_distance_to_goal = distance_to_goal;
    }

    if (distance_to_goal <
        options_tdbastar.delta_factor_goal * options_tdbastar.delta) {
      std::cout << "FOUND SOLUTION" << std::endl;
      std::cout << "COST: " << best_node->gScore + best_node->hScore
                << std::endl;
      std::cout << "x: " << best_node->state_eig.format(FMT) << std::endl;
      std::cout << "d: " << distance_to_goal << std::endl;
      status = Terminate_status::SOLVED;
      break;
    }

    // EXPAND the best node
    size_t num_expansion_best_node = 0;
    std::vector<LazyTraj> lazy_trajs;
    time_bench.time_lazy_expand += timed_fun_void(
        [&] { expander.expand_lazy(best_node->state_eig, lazy_trajs); });

    // CSTR_(lazy_trajs.size());
    // dynobench::Trajectory tmp_traj;
    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];

      int num_valid_states = -1;
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());

      bool motion_valid =
          check_lazy_trajectory(lazy_traj, *robot, time_bench, traj_wrapper,
                                aux_last_state, &ff, &num_valid_states);
      if (!motion_valid) {
        continue;
      }
      // Additional CHECK: if a intermediate state is close to goal. It really
      // helps!

      int chosen_index = -1;

      check_goal(*robot, tmp_node.state_eig, problem.goals[robot_id], traj_wrapper,
                 options_tdbastar.delta_factor_goal * options_tdbastar.delta,
                 num_check_goal, chosen_index);

      // Tentative hScore, gScore
      double hScore;
      time_bench.time_hfun +=
          timed_fun_void([&] { hScore = h_fun->h(tmp_node.state_eig); });
      assert(hScore >= 0);

      double cost_motion = chosen_index != -1
                               ? chosen_index * robot->ref_dt
                               : (traj_wrapper.get_size() - 1) * robot->ref_dt;

      assert(cost_motion >= 0);

      double gScore = best_node->gScore + cost_motion +
                      options_tdbastar.cost_delta_factor *
                          robot->lower_bound_time(best_node->state_eig,
                                                  traj_wrapper.get_state(0));

      // CHECK if new State is NOVEL
      time_bench.time_nearestNode_search += timed_fun_void([&] {
        T_n->nearestR(&tmp_node,
                      (1. - options_tdbastar.alpha) * options_tdbastar.delta,
                      neighbors_n);
      });

      if (!neighbors_n.size() || chosen_index != -1) {
        // STATE is NOVEL, we add the node
        num_expansion_best_node++;
        all_nodes.push_back(std::make_unique<AStarNode>());
        AStarNode *__node = all_nodes.back().get();
        __node->state_eig = tmp_node.state_eig;
        __node->gScore = gScore;
        __node->hScore = hScore;
        __node->fScore = gScore + hScore;
        __node->came_from = best_node;
        __node->used_motion = lazy_traj.motion->idx;
        if (chosen_index != -1)
          __node->intermediate_state = chosen_index;
        __node->is_in_open = true;

        time_bench.time_queue +=
            timed_fun_void([&] { __node->handle = open.push(__node); });
        time_bench.time_nearestNode_add +=
            timed_fun_void([&] { T_n->add(__node); });

        if (debug) {

          expanded_trajs.push_back(
              dynobench::trajWrapper_2_Trajectory(traj_wrapper));
        }

      } else {
        for (auto &n : neighbors_n) {
          // STATE is not novel, we udpate
          // the similar nodes
          if (double tentative_g =
                  gScore +
                  robot->lower_bound_time(tmp_node.state_eig, n->state_eig);
              tentative_g < n->gScore) {
            n->gScore = tentative_g;
            n->fScore = tentative_g + n->hScore;
            n->came_from = best_node;
            n->used_motion = lazy_traj.motion->idx;
            // TODO: think if add a flag
            // so that nodes thouching the
            // goal can not be modified
            n->intermediate_state = -1; // reset intermediate
                                        // state.
            // The motion is taken fully,
            // because otherwise it would
            // be novel and enter inside
            // the other if.
            if (n->is_in_open) {
              time_bench.time_queue +=
                  timed_fun_void([&] { open.increase(n->handle); });
            } else {
              time_bench.time_queue +=
                  timed_fun_void([&] { n->handle = open.push(n); });
            }
          }
        }
      }

      if (num_expansion_best_node >= options_tdbastar.limit_branching_factor) {
        break;
      }
    }
    // CSTR_(num_expansion_best_node);
  }

  time_bench.time_search = watch.elapsed_ms();

  time_bench.time_nearestMotion += expander.time_in_nn;
  time_bench.time_nearestNode =
      time_bench.time_nearestNode_add + time_bench.time_nearestNode_search;

  time_bench.extra_time =
      time_bench.time_search - time_bench.time_collisions -
      time_bench.time_nearestNode_add - time_bench.time_nearestNode_search -
      time_bench.time_lazy_expand - time_bench.time_alloc_primitive -
      time_bench.time_transform_primitive - time_bench.time_queue -
      time_bench.check_bounds - time_bench.time_hfun;

  assert(time_bench.extra_time >= 0);
  assert(time_bench.extra_time / time_bench.time_search * 100 <
         20.); // sanity check -- could this fail?

  std::cout << "extra time " << time_bench.extra_time << " "
            << time_bench.extra_time / time_bench.time_search * 100 << "%"
            << std::endl;

  if (debug) {
    std::ofstream out("/tmp/dynoplan/nodes_list.yaml");
    out << "close_list:" << std::endl;
    for (auto &c : closed_list) {
      out << "- " << c->state_eig.format(dynobench::FMT) << std::endl;
    }
    out << "all_nodes:" << std::endl;
    for (auto &c : all_nodes) {
      out << "- " << c->state_eig.format(dynobench::FMT) << std::endl;
    }
    std::ofstream out2("/tmp/dynoplan/expanded_trajs.yaml");

    out2 << "trajs:" << std::endl;
    for (auto &traj : expanded_trajs) {
      out2 << "  - " << std::endl;
      traj.to_yaml_format(out2, "    ");
    }

    {
      dynobench::Trajectories trajs;
      trajs.data = expanded_trajs;
      trajs.save_file_msgpack("/tmp/dynoplan/expanded_trajs.msgpack");
    }

    // write in msgpack format
  }

  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  AStarNode *solution = nullptr;

  if (status == Terminate_status::SOLVED) {
    solution = best_node;
    out_info_tdb.solved = true;
  } else {
    auto nearest = T_n->nearest(goal_node.get());
    std::cout << "Close distance T_n to goal: "
              << robot->distance(goal_node->getStateEig(),
                                 nearest->getStateEig())
              << std::endl;
    solution = nearest;
    out_info_tdb.solved = false;
  }

  // auto filename = "/tmp/dynoplan/dbastar_out.yaml";
  // create_dir_if_necessary(filename);
  // std::ofstream out(filename);

  // out << "solved: " << (status == Terminate_status::SOLVED) << std::endl;
  // out << "problem: " << problem.name << std::endl;
  // out << "robot: " << problem.robotTypes[robot_id] << std::endl;
  // out << "status: " << static_cast<int>(status) << std::endl;
  // out << "status_str: " << terminate_status_str[static_cast<int>(status)]
  //     << std::endl;
  // out << "all_nodes: " << all_nodes.size() << std::endl;
  // out << "closed_list: " << closed_list.size() << std::endl;
  // out << "gscore_sol: " << solution->gScore << std::endl;
  // out << "distance_to_goal: "
  //     << robot->distance(solution->getStateEig(), goal_node->getStateEig())
  //     << std::endl;
  // time_bench.write(out);

  // out << "result:" << std::endl;
  from_solution_to_yaml_and_traj(*robot, motions, solution, problem, traj_out,
                                 &out);
  traj_out.start = problem.starts[robot_id];
  traj_out.goal = problem.goals[robot_id];
  traj_out.check(robot, true);
  traj_out.cost = traj_out.actions.size() * robot->ref_dt;

  if (status == Terminate_status::SOLVED) {
    dynobench::Feasibility_thresholds thresholds;
    thresholds.col_tol =
        5 * 1e-2; // NOTE: for the systems with 0.01 s integration step,
    // I check collisions only at 0.05s . Thus, an intermediate state
    // could be slightly in collision.
    thresholds.goal_tol = options_tdbastar.delta;
    thresholds.traj_tol = options_tdbastar.delta;
    traj_out.update_feasibility(thresholds, true);
    CHECK(traj_out.feasible, "");
  }

  traj_out.update_feasibility(dynobench::Feasibility_thresholds(), true);

  // {
  //   std::string filename_id =
  //       "/tmp/dynoplan/traj_db_" + gen_random(6) + ".yaml";
  //   std::cout << "saving traj to: " << filename_id << std::endl;
  //   create_dir_if_necessary(filename_id.c_str());
  //   std::ofstream out(filename_id);
  //   traj_out.to_yaml_format(out);
  // }

  out_info_tdb.solved = status == Terminate_status::SOLVED;
  out_info_tdb.cost = traj_out.cost;
  out_info_tdb.time_search = time_bench.time_search;
  out_info_tdb.data = time_bench.to_data();
  out_info_tdb.data.insert(std::make_pair(
      "terminate_status", terminate_status_str[static_cast<int>(status)]));
  out_info_tdb.data.insert(
      std::make_pair("solved", std::to_string(bool(out_info_tdb.solved))));
  out_info_tdb.data.insert(
      std::make_pair("delta", std::to_string(options_tdbastar.delta)));
  out_info_tdb.data.insert(
      std::make_pair("num_primitives", std::to_string(motions.size())));
}

void write_heu_map(const std::vector<Heuristic_node> &heu_map, const char *file,
                   const char *header) {
  std::ofstream out(file);

  if (header) {
    out << header << std::endl;
  }
  const char *four_space = "    ";
  out << "heu_map:" << std::endl;
  for (auto &v : heu_map) {
    out << "  -" << std::endl;
    out << four_space << "x: " << v.x.format(FMT) << std::endl;
    out << four_space << "d: " << v.d << std::endl;
    out << four_space << "p: " << v.p << std::endl;
  }
}

void load_heu_map(const char *file, std::vector<Heuristic_node> &heu_map) {
  std::cout << "loading heu map -- file: " << file << std::endl;
  std::ifstream in(file);
  CHECK(in.is_open(), AT);
  YAML::Node node = YAML::LoadFile(file);

  if (node["heu_map"]) {

    for (const auto &state : node["heu_map"]) {
      std::vector<double> x = state["x"].as<std::vector<double>>();
      Eigen::VectorXd xe = Eigen::VectorXd::Map(x.data(), x.size());
      double d = state["d"].as<double>();
      int p = state["p"].as<double>();
      heu_map.push_back({xe, d, p});
    }
  } else {
    ERROR_WITH_INFO("missing map key");
  }
}

} // namespace dynoplan