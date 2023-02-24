#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::pair;
using std::shared_ptr;
using std::tuple;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
// class UniformGrid;
class HGrid;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED, NO_GRID };

class FastExplorationManager {
public:
  FastExplorationManager();
  ~FastExplorationManager();

  void initialize(ros::NodeHandle& nh);

  int planExploreMotion(
      const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);

  int planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
      const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw);

  int updateFrontierStruct(const Eigen::Vector3d& pos);

  void allocateGrids(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
      const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
      vector<int>& other_ids);

  // Find optimal tour visiting unknown grid
  bool findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, vector<int>& ids, vector<vector<int>>& others,
      bool init = false);

  void calcMutualCosts(const Eigen::Vector3d& pos, const double& yaw, const Eigen::Vector3d& vel,
      const vector<pair<Eigen::Vector3d, double>>& views, vector<float>& costs);

  double computeGridPathCost(const Eigen::Vector3d& pos, const vector<int>& grid_ids,
      const vector<int>& first, const vector<vector<int>>& firsts,
      const vector<vector<int>>& seconds, const double& w_f);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<HGrid> hgrid_;
  shared_ptr<SDFMap> sdf_map_;
  // shared_ptr<UniformGrid> uniform_grid_;

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  ros::ServiceClient tsp_client_, acvrp_client_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
      vector<int>& indices);

  // Find optimal coordinated tour for quadrotor swarm
  void findGridAndFrontierPath(const Vector3d& cur_pos, const Vector3d& cur_vel,
      const Vector3d& cur_yaw, vector<int>& grid_ids, vector<int>& ftr_ids);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
      vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

  void shortenPath(vector<Vector3d>& path);

  void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, vector<int>& ids);

public:
  typedef shared_ptr<FastExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif