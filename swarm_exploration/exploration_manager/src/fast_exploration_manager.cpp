// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
// #include <active_perception/uniform_grid.h>
#include <active_perception/hgrid.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/planner_manager.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_tsp_solver/SolveTSP.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);

  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // uniform_grid_.reset(new UniformGrid(edt_environment_, nh));
  hgrid_.reset(new HGrid(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/drone_num", ep_->drone_num_, 1);
  nh.param("exploration/drone_id", ep_->drone_id_, 1);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);

  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  tsp_client_ =
      nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
  acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>(
      "/solve_acvrp_" + to_string(ep_->drone_id_), true);

  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;
  ed_->plan_num_ = 0;

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Do global and local tour planning and retrieve the next viewpoint

  ed_->frontier_tour_.clear();
  Vector3d next_pos;
  double next_yaw;
  // Find the tour passing through viewpoints
  // Optimal tour is returned as indices of frontier
  vector<int> grid_ids, frontier_ids;
  // findGlobalTour(pos, vel, yaw, indices);
  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

  if (grid_ids.empty()) {

    return NO_GRID;

    // No grid is assigned to this drone, but keep moving is necessary
    // Move to the closest targets
    ROS_WARN("Empty grid");

    double min_cost = 100000;
    int min_cost_id = -1;
    vector<Vector3d> tmp_path;
    for (int i = 0; i < ed_->averages_.size(); ++i) {
      auto tmp_cost =
          ViewNode::computeCost(pos, ed_->points_[i], yaw[0], ed_->yaws_[i], vel, yaw[1], tmp_path);
      if (tmp_cost < min_cost) {
        min_cost = tmp_cost;
        min_cost_id = i;
      }
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];

  } else if (frontier_ids.size() == 0) {
    // // The assigned grid contains no frontier, find the one closest to the grid
    // ROS_WARN("No frontier in grid");

    Eigen::Vector3d grid_center = ed_->grid_tour_[1];

    double min_cost = 100000;
    int min_cost_id = -1;
    for (int i = 0; i < ed_->points_.size(); ++i) {
      // double cost = (grid_center - ed_->averages_[i]).norm();
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCost(
          grid_center, ed_->averages_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];

    // // Simply go to the center of the unknown grid
    // next_pos = grid_center;
    // Eigen::Vector3d dir = grid_center - pos;
    // next_yaw = atan2(dir[1], dir[0]);

  } else if (frontier_ids.size() == 1) {
    // ROS_WARN("Single frontier");
    if (ep_->refine_local_) {
      // Single frontier, find the min cost viewpoint for it
      ed_->refined_ids_ = { frontier_ids[0] };
      ed_->unrefined_points_ = { ed_->points_[frontier_ids[0]] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { frontier_ids[0] }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      if (grid_ids.size() <= 1) {
        // Only one grid is assigned
        double min_cost = 100000;
        int min_cost_id = -1;
        vector<Vector3d> tmp_path;
        for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
          auto tmp_cost = ViewNode::computeCost(
              pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
          if (tmp_cost < min_cost) {
            min_cost = tmp_cost;
            min_cost_id = i;
          }
        }
        next_pos = ed_->n_points_[0][min_cost_id];
        next_yaw = n_yaws[0][min_cost_id];
      } else {
        // More than one grid, the next grid is considered for path planning
        // vector<Eigen::Vector3d> grid_pos = { ed_->grid_tour_[2] };
        // Eigen::Vector3d dir = ed_->grid_tour_[2] - ed_->grid_tour_[1];
        // vector<double> grid_yaw = { atan2(dir[1], dir[0]) };

        Eigen::Vector3d grid_pos;
        double grid_yaw;
        if (hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw)) {
          ed_->n_points_.push_back({ grid_pos });
          n_yaws.push_back({ grid_yaw });
        }

        ed_->refined_points_.clear();
        ed_->refined_views_.clear();
        vector<double> refined_yaws;
        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
        next_pos = ed_->refined_points_[0];
        next_yaw = refined_yaws[0];
      }
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    }
  } else {
    // ROS_WARN("Multiple frontier");
    // More than two frontiers are assigned
    // Do refinement for the next few viewpoints in the global tour
    t1 = ros::Time::now();

    ed_->refined_ids_.clear();
    ed_->unrefined_points_.clear();
    int knum = min(int(frontier_ids.size()), ep_->refined_num_);
    for (int i = 0; i < knum; ++i) {
      auto tmp = ed_->points_[frontier_ids[i]];
      ed_->unrefined_points_.push_back(tmp);
      ed_->refined_ids_.push_back(frontier_ids[i]);
      if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
    }

    // Get top N viewpoints for the next K frontiers
    ed_->n_points_.clear();
    vector<vector<double>> n_yaws;
    frontier_finder_->getViewpointsInfo(
        pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

    ed_->refined_points_.clear();
    ed_->refined_views_.clear();
    vector<double> refined_yaws;
    refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
    next_pos = ed_->refined_points_[0];
    next_yaw = refined_yaws[0];

    // Get marker for view visualization
    for (int i = 0; i < ed_->refined_points_.size(); ++i) {
      Vector3d view =
          ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
      ed_->refined_views_.push_back(view);
    }
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    for (int i = 0; i < ed_->refined_points_.size(); ++i) {
      vector<Vector3d> v1, v2;
      frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
      frontier_finder_->percep_utils_->getFOV(v1, v2);
      ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
      ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
    }
    double local_time = (ros::Time::now() - t1).toSec();
    ROS_INFO("Local refine time: %lf", local_time);
  }

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;
  ed_->next_pos_ = next_pos;
  ed_->next_yaw_ = next_yaw;

  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  double total = (ros::Time::now() - t2).toSec();
  ROS_INFO("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

int FastExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = ros::Time::now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  // bool start_unknown = (edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN);
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END) {
    ROS_ERROR("No path to next viewpoint");
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      ROS_WARN("init plan.");
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    ROS_ERROR("Lower bound not satified!");

  double traj_plan_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

  return SUCCEED;
}

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d& pos) {

  auto t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit();

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  ROS_INFO("Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf", ep_->drone_id_, frontier_time,
      view_time, mat_time);

  ROS_INFO("Total t: %lf", (ros::Time::now() - t2).toSec());
  return ed_->frontiers_.size();
}

void FastExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids) {
  auto t1 = ros::Time::now();

  // Select nearby drones according to their states' stamp
  vector<Eigen::Vector3d> positions = { cur_pos };
  // vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0) };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // Partitioning-based tour planning
  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    return;
  }
  grid_ids = ego_ids;

  double grid_time = (ros::Time::now() - t1).toSec();

  // Frontier-based single drone tour planning
  // Restrict frontier within the first visited grid
  t1 = ros::Time::now();

  vector<int> ftr_ids;
  // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
  ROS_INFO("Find frontier tour, %d involved------------", ftr_ids.size());

  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }

  // Consider next grid in frontier tour planning
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }

  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  double ftr_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d cur_yaw, vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();
  std::cout << "mat:   " << cost_mat.rows() << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE ="
           << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tou"
                                                                      "r\n";
  par_file << "RUNS = 1\n";
  par_file.close();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp");
  // Problem specification part, follow the format of TSPLIB
  string prob_spec;
  prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
              "\nEDGE_WEIGHT_TYPE : "
              "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  // solveTSPLKH((ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par").c_str());
  lkh_tsp_solver::SolveTSP srv;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve TSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1) break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  std::cout << "Tour " << ep_->drone_id_ << ": ";
  for (auto id : indices) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);

  // if (tsp_time > 0.1) ROS_BREAK();
}

void FastExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
    const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local refine graph size: 1, ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

void FastExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
    vector<int>& other_ids) {
  // ROS_INFO("Allocate grid.");

  auto t1 = ros::Time::now();
  auto t2 = t1;

  if (grid_ids.size() == 1) {  // Only one grid, no need to run ACVRP
    auto pt = hgrid_->getCenter(grid_ids.front());
    // double d1 = (positions[0] - pt).norm();
    // double d2 = (positions[1] - pt).norm();
    vector<Eigen::Vector3d> path;
    double d1 = ViewNode::computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    double d2 = ViewNode::computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    if (d1 < d2) {
      ego_ids = grid_ids;
      other_ids = {};
    } else {
      ego_ids = {};
      other_ids = grid_ids;
    }
    return;
  }

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
  hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

  // int unknown = hgrid_->getTotalUnknwon();
  int unknown;

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through AmTSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = positions.size();

  vector<int> unknown_nums;
  int capacity = 0;
  for (int i = 0; i < grid_ids.size(); ++i) {
    int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
    unknown_nums.push_back(unum);
    capacity += unum;
    // std::cout << "Grid " << i << ": " << unum << std::endl;
  }
  // std::cout << "Total: " << capacity << std::endl;
  capacity = capacity * 0.75 * 0.1;

  // int prob_type;
  // if (grid_ids.size() >= 3)
  //   prob_type = 2;  // Use ACVRP
  // else
  //   prob_type = 1;  // Use AmTSP

  const int prob_type = 2;

  // Create problem file--------------------------
  ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : pairopt\n";

  if (prob_type == 1)
    file << "TYPE : ATSP\n";
  else if (prob_type == 2)
    file << "TYPE : ACVRP\n";

  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (prob_type == 2) {
    file << "CAPACITY : " + to_string(capacity) + "\n";   // ACVRP
    file << "VEHICLES : " + to_string(drone_num) + "\n";  // ACVRP
  }

  // Cost matrix
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }

  if (prob_type == 2) {  // Demand section, ACVRP only
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << to_string(i + 2) + " 0\n";
    }
    for (int i = 0; i < grid_ids.size(); ++i) {
      int grid_unknown = unknown_nums[i] * 0.1;
      file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  // Create par file------------------------------------------
  int min_size = int(grid_ids.size()) / 2;
  int max_size = ceil(int(grid_ids.size()) / 2.0);
  file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
  if (prob_type == 1) {
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_OBJECTIVE = MINMAX\n";
    file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
    file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (prob_type == 2) {
    file << "TRACE_LEVEL = 1\n";  // ACVRP
    file << "SEED = 0\n";         // ACVRP
  }
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 3;
  // if (!tsp_client_.call(srv)) {
  if (!acvrp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ACVRP.");
    return;
  }
  // system("/home/boboyu/software/LKH-3.0.6/LKH
  // /home/boboyu/workspaces/hkust_swarm_ws/src/swarm_exploration/utils/lkh_mtsp_solver/resource/amtsp3_1.par");

  double mtsp_time = (ros::Time::now() - t1).toSec();
  std::cout << "Allocation time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }
  // // Print tour ids
  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }

  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : ego_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  for (auto& id : other_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  // // Remove repeated grid
  // unordered_map<int, int> ego_map, other_map;
  // for (auto id : ego_ids) ego_map[id] = 1;
  // for (auto id : other_ids) other_map[id] = 1;

  // ego_ids.clear();
  // other_ids.clear();
  // for (auto p : ego_map) ego_ids.push_back(p.first);
  // for (auto p : other_map) other_ids.push_back(p.first);

  // sort(ego_ids.begin(), ego_ids.end());
  // sort(other_ids.begin(), other_ids.end());
}

double FastExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos,
    const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f) {
  if (grid_ids.empty()) return 0.0;

  double cost = 0.0;
  vector<Eigen::Vector3d> path;
  cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
  for (int i = 0; i < grid_ids.size() - 1; ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}

bool FastExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {

  ROS_INFO("Find grid tour---------------");

  auto t1 = ros::Time::now();

  auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

  // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

  vector<int> first_ids, second_ids;
  hgrid_->inputFrontiers(ed_->averages_);

  hgrid_->updateGridData(
      ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);

  if (grid_ids.empty()) {
    ROS_WARN("Empty dominance.");
    ed_->grid_tour_.clear();
    return false;
  }

  std::cout << "Allocated grid: ";
  for (auto id : grid_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
  if (!init)
    hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
  else
    hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through ATSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = 1;

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // std::cout << "AmTSP time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }
  others.resize(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  for (auto& other : others) {
    for (auto& id : other) id -= 1 + drone_num;
  }
  std::cout << "Grid tour: ";
  for (auto& id : indices) {
    id = grid_ids[id];
    std::cout << id << ", ";
  }
  std::cout << "" << std::endl;

  // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
  grid_ids = indices;
  hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;

  // hgrid_->checkFirstGrid(grid_ids.front());

  return true;
}

void FastExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

  auto t1 = ros::Time::now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
  Eigen::MatrixXd mat;
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();
  // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("mat time: %lf", mat_time);

  // Find optimal allocation through AmTSP
  t1 = ros::Time::now();

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("AmTSP time: %lf", mtsp_time);

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  vector<vector<int>> others(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
    // else {
    //   others[tours[i][0] - 2].insert(
    //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
    // }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  // for (auto& other : others) {
  //   for (auto& id : other)
  //     id -= 1 + drone_num;
  // }

  if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
    indices.pop_back();
  }
  // Subset of frontier inside first grid
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }

  // ed_->other_tours_.clear();
  // for (int i = 1; i < positions.size(); ++i) {
  //   ed_->other_tours_.push_back({});
  //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
  // }

  double parse_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
  //     parse_time, indices.size());
}

}  // namespace fast_planner
