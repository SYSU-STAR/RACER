#include <active_perception/uniform_grid.h>
#include <active_perception/hgrid.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>

namespace fast_planner {

HGrid::HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh) {

  this->edt_ = edt;
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);
  nh.param("partitioning/consistent_cost2", consistent_cost2_, 3.5);
  nh.param("partitioning/use_swarm_tf", use_swarm_tf_, false);
  nh.param("partitioning/w_first", w_first_, 1.0);

  path_finder_.reset(new Astar);
  path_finder_->init(nh, edt);

  grid1_.reset(new UniformGrid(edt, nh, 1));
  grid2_.reset(new UniformGrid(edt, nh, 2));

  // Swarm tf
  grid1_->use_swarm_tf_ = grid2_->use_swarm_tf_ = use_swarm_tf_;
  double yaw = 0.0;
  rot_sw_ << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  trans_sw_ << 0.0, 0.0, 0;
  grid1_->rot_sw_ = grid2_->rot_sw_ = rot_sw_;
  grid1_->trans_sw_ = grid2_->trans_sw_ = trans_sw_;

  // Wait for swarm basecoor transform and initialize grid
  // while (!updateBaseCoor()) {
  //   ROS_WARN("Wait for basecoor.");
  //   ros::Duration(0.5).sleep();
  //   ros::spinOnce();
  // }
  grid1_->initGridData();
  grid2_->initGridData();
  updateBaseCoor();
}

HGrid::~HGrid() {
}

bool HGrid::updateBaseCoor() {

  // Eigen::Vector4d tf;
  // if (!edt_->sdf_map_->getBaseCoor(1, tf)) return false;
  // double yaw = tf[3];
  // rot_sw_ << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // trans_sw_ = tf.head<3>();

  rot_sw_ = Eigen::Matrix3d::Identity();
  trans_sw_ = Eigen::Vector3d::Zero();

  grid1_->rot_sw_ = grid2_->rot_sw_ = rot_sw_;
  grid1_->trans_sw_ = grid2_->trans_sw_ = trans_sw_;
  grid1_->updateBaseCoor();
  grid2_->updateBaseCoor();

  return true;
}

void HGrid::inputFrontiers(const vector<Eigen::Vector3d>& avgs) {
  // Input frontier to both levels
  grid1_->inputFrontiers(avgs);
  grid2_->inputFrontiers(avgs);
}

void HGrid::updateGridData(const int& drone_id, vector<int>& grid_ids, bool reallocated,
    const vector<int>& last_grid_ids, vector<int>& first_ids, vector<int>& second_ids) {

  // Convert grid_ids to the ids of bi-level uniform grid
  vector<int> grid_ids1, grid_ids2;
  const int grid_num1 = grid1_->grid_data_.size();
  for (auto id : grid_ids) {
    if (id < grid_num1)
      grid_ids1.push_back(id);
    else
      grid_ids2.push_back(id - grid_num1);  // Id of level 2 grid
  }

  // std::cout << "Input ids: ";
  // for (auto id : grid_ids)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // std::cout << "level 1 ids: ";
  // for (auto id : grid_ids1)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // std::cout << "level 2 ids: ";
  // for (auto id : grid_ids2)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // Update at level 1
  vector<int> tmp_ids1 = grid_ids1;
  vector<int> parti_ids1, parti_ids1_all;
  grid1_->updateGridData(drone_id, grid_ids1, parti_ids1, parti_ids1_all);

  // std::cout << "updated level 1 ids: ";
  // for (auto id : grid_ids1)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // std::cout << "divided level 1 ids: ";
  // for (auto id : parti_ids1)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // Merge the newly partitioned and original grid ids
  vector<int> fine_ids;
  for (auto id : parti_ids1) {
    vector<int> tmp_ids;
    coarseToFineId(id, tmp_ids);
    fine_ids.insert(fine_ids.end(), tmp_ids.begin(), tmp_ids.end());
  }
  grid_ids2.insert(grid_ids2.end(), fine_ids.begin(), fine_ids.end());

  // Activate newly divided grids
  vector<int> fine_ids_all;
  for (auto id : parti_ids1_all) {
    vector<int> tmp_ids;
    coarseToFineId(id, tmp_ids);
    fine_ids_all.insert(fine_ids_all.end(), tmp_ids.begin(), tmp_ids.end());
  }
  grid2_->activateGrids(fine_ids_all);

  // if (reallocated) grid2_->activateGrids(grid_ids2);

  // std::cout << "merged level 2 ids: ";
  // for (auto id : grid_ids2)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  vector<int> parti_ids2, parti_ids2_all;  // Should be empty, no partition at level 2
  grid2_->updateGridData(drone_id, grid_ids2, parti_ids2, parti_ids2_all);

  // std::cout << "updated level 2 ids: ";
  // for (auto id : grid_ids2)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  grid_ids = grid_ids1;
  for (auto& id : grid_ids2) {
    grid_ids.push_back(id + grid_num1);
  }

  // Maintain consistency of next visited grid
  // if (reallocated) return;
  getConsistentGrid(last_grid_ids, grid_ids, first_ids, second_ids);
}

void HGrid::getConsistentGrid(const vector<int>& last_ids, const vector<int>& cur_ids,
    vector<int>& first_ids, vector<int>& second_ids) {

  if (last_ids.empty()) return;
  // std::cout << "last id: ";
  // for (auto id : last_ids)
  //   std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // Find the first two level 1 grids in last sequence
  const int grid_num1 = grid1_->grid_data_.size();
  int grid_id1 = last_ids[0];
  if (grid_id1 >= grid_num1) {
    int tmp = grid_id1 - grid_num1;
    fineToCoarseId(tmp, grid_id1);
  }

  // std::cout << "level 1 grid 1: " << grid_id1 << std::endl;

  int grid_id2 = -1;
  for (int i = 1; i < last_ids.size(); ++i) {
    if (last_ids[i] < grid_num1) {
      grid_id2 = last_ids[i];
      break;
    } else {
      int fine = last_ids[i] - grid_num1;
      int coarse;
      fineToCoarseId(fine, coarse);
      if (coarse != grid_id1) {
        grid_id2 = coarse;
        break;
      }
    }
  }
  // std::cout << "level 1 grid 2: " << grid_id2 << std::endl;

  first_ids.clear();
  // In the current sequence, try to find the first level 1 grid...
  for (auto id : cur_ids) {
    if (id == grid_id1) {
      first_ids = { id };
      // std::cout << "1" << std::endl;
    }
  }

  if (first_ids.empty()) {
    // or its sub-grids
    for (auto id : cur_ids) {
      if (id < grid_num1) continue;
      int coarse;
      fineToCoarseId(id - grid_num1, coarse);
      if (coarse == grid_id1) {
        first_ids.push_back(id);
      }
    }
  }

  vector<int>* ids_ptr;
  if (!first_ids.empty()) {
    // Already find the first, should find the second
    ids_ptr = &second_ids;
  } else {
    // No first yet, continue to find the first
    ids_ptr = &first_ids;
  }

  // Can not find first grid/sub-grid, try to find the second one
  if (grid_id2 == -1) return;

  for (auto id : cur_ids) {
    if (id == grid_id2) {
      *ids_ptr = { id };
      // std::cout << "3" << std::endl;
      return;
    }
  }

  for (auto id : cur_ids) {
    if (id < grid_num1) continue;
    int coarse;
    fineToCoarseId(id - grid_num1, coarse);
    if (coarse == grid_id2) {
      // std::cout << "4" << std::endl;
      ids_ptr->push_back(id);
    }
  }
  return;
}

void HGrid::coarseToFineId(const int& coarse, vector<int>& fines) {
  // 0: 0, 1
  // 1: 2, 3
  // 2: 4, 5
  fines.clear();
  Eigen::Vector3i cidx;  // coarse idx
  grid1_->adrToIndex(coarse, cidx);

  vector<Eigen::Vector3i> fine_idxs;
  fine_idxs.emplace_back(cidx[0] * 2, cidx[1] * 2, cidx[2]);
  fine_idxs.emplace_back(cidx[0] * 2 + 1, cidx[1] * 2, cidx[2]);
  fine_idxs.emplace_back(cidx[0] * 2, cidx[1] * 2 + 1, cidx[2]);
  fine_idxs.emplace_back(cidx[0] * 2 + 1, cidx[1] * 2 + 1, cidx[2]);

  for (auto idx : fine_idxs) {
    fines.push_back(grid2_->toAddress(idx));
  }
}

void HGrid::fineToCoarseId(const int& fine, int& coarse) {
  Eigen::Vector3i fidx;
  grid2_->adrToIndex(fine, fidx);

  Eigen::Vector3i cidx;
  cidx[0] = fidx[0] / 2;
  cidx[1] = fidx[1] / 2;
  cidx[2] = fidx[2];

  coarse = grid1_->toAddress(cidx);
}

void HGrid::getCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, Eigen::MatrixXd& mat) {
  // first_ids and second_ids are drone_num x 1-4 vectors

  // Fill the cost matrix
  const int drone_num = positions.size();
  const int grid_num = grid_ids.size();
  const int dimen = 1 + drone_num + grid_num;
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // std::cout << "First id: ";
  // for (auto ids : first_ids)
  //   for (auto id : ids)
  //     std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // std::cout << "Second id: ";
  // for (auto ids : second_ids)
  //   for (auto id : ids)
  //     std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = -1000;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to grid
  for (int i = 0; i < grid_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to grid
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < grid_num; ++j) {
      double cost = getCostDroneToGrid(positions[i], grid_ids[j], first_ids[i]);
      mat(1 + i, 1 + drone_num + j) = cost;
      mat(1 + drone_num + j, 1 + i) = 0;
    }
  }
  // Costs between grid
  for (int i = 0; i < grid_num; ++i) {
    for (int j = i + 1; j < grid_num; ++j) {
      double cost = getCostGridToGrid(grid_ids[i], grid_ids[j], first_ids, second_ids, drone_num);
      mat(1 + drone_num + i, 1 + drone_num + j) = cost;
      mat(1 + drone_num + j, 1 + drone_num + i) = cost;
    }
  }

  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }
}

double HGrid::getCostDroneToGrid(
    const Eigen::Vector3d& pos, const int& grid_id, const vector<int>& first) {
  auto& grid = getGrid(grid_id);
  double dist1, cost;
  dist1 = (pos - grid.center_).norm();
  if (dist1 < 5.0) {
    path_finder_->reset();
    if (path_finder_->search(pos, grid.center_) == Astar::REACH_END) {
      auto path = path_finder_->getPath();
      cost = path_finder_->pathLength(path);
    } else {
      cost = dist1 + consistent_cost2_;
    }
  } else {
    cost = 1.5 * dist1 + consistent_cost2_;
  }
  // Consistency cost with previous first grid
  if (!first.empty()) {
    for (auto first_id : first) {
      if (grid_id == first_id) {
        cost += consistent_cost_;
        break;
      }
    }
  }
  // if (drone_num > 1) cost *= w_first_;
  return cost;
}

double HGrid::getCostGridToGrid(const int& id1, const int& id2, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const int& drone_num) {
  auto& grid1 = getGrid(id1);
  auto& grid2 = getGrid(id2);
  double dist_cost = 0.0;

  if (isClose(id1, id2)) {
    // Neighbor grid, search path to compute exact cost
    path_finder_->reset();
    if (path_finder_->search(grid1.center_, grid2.center_) == Astar::REACH_END) {
      auto path = path_finder_->getPath();
      dist_cost = path_finder_->pathLength(path);
    } else {
      dist_cost = (grid1.center_ - grid2.center_).norm() + consistent_cost2_;
    }

    // Make level 2 grids in the same level 1 grid adhere together
    if (drone_num <= 1 && inSameLevel1(id1, id2)) {
      dist_cost += consistent_cost_;
    }

    if (!firsts.empty()) {
      // Consistency between firsts and second
      bool is_first, is_second;
      for (int k = 0; k < drone_num; ++k) {
        is_first = false;
        is_second = false;
        for (auto first_id : firsts[k]) {
          if (id1 == first_id) {
            is_first = true;
            break;
          }
        }
        for (auto second_id : seconds[k]) {
          if (id2 == second_id) {
            is_second = true;
            break;
          }
        }
        if (is_first && is_second) break;
      }
      if (is_first && is_second) {
        dist_cost += consistent_cost_;
      }
    }
  } else {
    // Not nearby grids, approximate cost by straight line dist
    // double dist_cost = 1.5 * (grid1.center_ - grid2.center_).norm() - consistent_cost_;
    dist_cost = 1.5 * (grid1.center_ - grid2.center_).norm() + consistent_cost2_;
    // double dist_cost = (grid1.center_ - grid2.center_).norm();
  }
  return dist_cost;
}

int HGrid::getUnknownCellsNum(const int& grid_id) {
  // Get unknown cell number of a grid
  return getGrid(grid_id).unknown_num_;
}

Eigen::Vector3d HGrid::getCenter(const int& id) {
  return getGrid(id).center_;
}

GridInfo& HGrid::getGrid(const int& id) {
  int grid_num1 = grid1_->grid_data_.size();
  if (id < grid_num1)
    return grid1_->grid_data_[id];
  else
    return grid2_->grid_data_[id - grid_num1];
}

void HGrid::getActiveGrids(vector<int>& grid_ids) {
  grid_ids.clear();
  const int grid_num1 = grid1_->grid_data_.size();
  for (int i = 0; i < grid_num1; ++i) {
    if (grid1_->grid_data_[i].active_ && grid1_->grid_data_[i].is_cur_relevant_) {
      grid_ids.push_back(i);
    }
  }
  for (int i = 0; i < grid2_->grid_data_.size(); ++i) {
    if (grid2_->grid_data_[i].active_ && grid2_->grid_data_[i].is_cur_relevant_) {
      grid_ids.push_back(i + grid_num1);
    }
  }
}

bool HGrid::getNextGrid(const vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw) {
  // Current level 1 grid id
  const int grid_num1 = grid1_->grid_data_.size();
  int grid_id1;
  if (grid_ids[0] < grid_num1)
    grid_id1 = grid_ids[0];
  else {
    int fine = grid_ids[0] - grid_num1;  // level 2 id
    fineToCoarseId(fine, grid_id1);
  }

  // std::cout << "current level 1 id: " << grid_id1 << std::endl;

  // Find the next different level 1 grid id
  int grid_id2 = -1;
  for (int i = 1; i < grid_ids.size(); ++i) {
    if (grid_ids[i] < grid_num1) {
      grid_id2 = grid_ids[i];
      break;
    } else {
      int fine = grid_ids[i] - grid_num1;
      int coarse;
      fineToCoarseId(fine, coarse);
      if (coarse != grid_id1) {
        grid_id2 = grid_ids[i];
        break;
      }
    }
  }

  // std::cout << "next level 1 id: " << grid_id2 << std::endl;

  if (grid_id2 == -1) return false;

  auto& grid1 = getGrid(grid_id1);
  auto& grid2 = getGrid(grid_id2);
  grid_pos = grid2.center_;
  Eigen::Vector3d dir = grid2.center_ - grid1.center_;
  grid_yaw = atan2(dir[1], dir[0]);

  // std::cout << "grid pos: " << grid_pos.transpose() << std::endl;

  return true;
}

bool HGrid::isClose(const int& id1, const int& id2) {
  // Convert to coarse level ids
  const int grid_num1 = grid1_->grid_data_.size();

  int tmp_id1 = id1;
  if (tmp_id1 >= grid_num1) {
    int fine = tmp_id1 - grid_num1;
    fineToCoarseId(fine, tmp_id1);
  }
  int tmp_id2 = id2;
  if (tmp_id2 >= grid_num1) {
    int fine = tmp_id2 - grid_num1;
    fineToCoarseId(fine, tmp_id2);
  }
  Eigen::Vector3i idx1, idx2;
  grid1_->adrToIndex(tmp_id1, idx1);
  grid1_->adrToIndex(tmp_id2, idx2);

  for (int i = 0; i < 3; ++i) {
    if (abs(idx1[i] - idx2[i]) > 1) return false;
  }
  return true;

  // int diff = abs(tmp_id1 - tmp_id2);
  // if (diff == 1 || diff == grid1_->grid_num_[1]) return true;
  // return false;
}

bool HGrid::inSameLevel1(const int& id1, const int& id2) {
  // Check whether two level 2 grids are contained in the same level 1 grid
  const int grid_num1 = grid1_->grid_data_.size();
  if (id1 < grid_num1 || id2 < grid_num1) return false;

  int tmp1 = id1 - grid_num1;
  int tmp2 = id2 - grid_num1;
  int coarse1, coarse2;
  fineToCoarseId(tmp1, coarse1);
  fineToCoarseId(tmp2, coarse2);
  if (coarse1 == coarse2) return true;
  return false;
}

bool HGrid::isConsistent(const int& id1, const int& id2) {
  const int grid_num1 = grid1_->grid_data_.size();

  int tmp1 = id1;
  if (tmp1 >= grid_num1) {
    tmp1 -= grid_num1;
    int coarse;
    fineToCoarseId(tmp1, coarse);
    tmp1 = coarse;
  }
  int tmp2 = id2;
  if (tmp2 >= grid_num1) {
    tmp2 -= grid_num1;
    int coarse;
    fineToCoarseId(tmp2, coarse);
    tmp2 = coarse;
  }

  if (tmp1 == tmp2) return true;
  return false;
}

void HGrid::getGridTour(const vector<int>& ids, const Eigen::Vector3d& pos,
    vector<Eigen::Vector3d>& tour, vector<Eigen::Vector3d>& tour2) {
  const int grid_num1 = grid1_->grid_data_.size();

  // Get the centers of the visited grids
  vector<Eigen::Vector3d> centers = { pos };
  for (auto id : ids) {
    if (id < grid_num1) {
      centers.push_back(grid1_->grid_data_[id].center_);
    } else {
      int tmp = id - grid_num1;
      centers.push_back(grid2_->grid_data_[tmp].center_);
    }
  }
  tour = centers;

  // Find the exact path visiting the grids
  tour2 = { pos };
  for (int i = 0; i < centers.size() - 1; ++i) {
    path_finder_->reset();
    if (path_finder_->search(centers[i], centers[i + 1]) == Astar::REACH_END) {
      auto path = path_finder_->getPath();
      tour2.insert(tour2.end(), path.begin() + 1, path.end());
    } else {
      tour2.push_back(centers[i + 1]);
    }
  }
}

void HGrid::getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& ftr_ids) {
  ftr_ids.clear();
  int tmp = grid_ids.front();
  int grid_num1 = grid1_->grid_data_.size();

  if (tmp < grid_num1) {
    auto& grid = grid1_->grid_data_[tmp];
    for (auto pair : grid.contained_frontier_ids_) ftr_ids.push_back(pair.first);
  } else {
    // Find all frontier in the same level 1 grid
    tmp -= grid_num1;
    int coarse;
    fineToCoarseId(tmp, coarse);
    vector<int> fines;
    coarseToFineId(coarse, fines);

    vector<int> allocated_fines;  // level 2 grid allocated to current drone
    for (auto fine : fines) {
      for (auto id : grid_ids) {
        if (fine + grid_num1 == id) {
          allocated_fines.push_back(fine);
          break;
        }
      }
    }

    for (auto fine : allocated_fines) {
      auto& grid = grid2_->grid_data_[fine];
      for (auto pair : grid.contained_frontier_ids_) ftr_ids.push_back(pair.first);
    }
  }
}

void HGrid::checkFirstGrid(const int& id) {
  auto& grid = getGrid(id);

  std::cout << "grid id: " << id << std::endl;
  std::cout << "unknown num: " << grid.unknown_num_ << std::endl;
  std::cout << "center: " << grid.center_.transpose() << std::endl;
  std::cout << "relevant: " << grid.is_cur_relevant_ << ", " << grid.is_prev_relevant_ << std::endl;
}

void HGrid::getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2) {
  pts1.clear();
  pts2.clear();

  for (auto& grid : grid1_->grid_data_) {
    if (!grid.active_) continue;
    for (int i = 0; i < 4; ++i) {
      pts1.push_back(grid.vertices_[i]);
      pts2.push_back(grid.vertices_[(i + 1) % 4]);
    }
  }
  for (auto& grid : grid2_->grid_data_) {
    if (!grid.active_) continue;
    for (int i = 0; i < 4; ++i) {
      pts1.push_back(grid.vertices_[i]);
      pts2.push_back(grid.vertices_[(i + 1) % 4]);
    }
  }
  for (auto& pt : pts1) pt[2] = 0.5;
  for (auto& pt : pts2) pt[2] = 0.5;
}

void HGrid::getGridMarker2(vector<Eigen::Vector3d>& pts, vector<string>& texts) {
  pts.clear();
  texts.clear();

  for (int i = 0; i < grid1_->grid_data_.size(); ++i) {
    auto& grid = grid1_->grid_data_[i];
    if (!grid.active_ || !grid.is_cur_relevant_) continue;
    pts.push_back(grid.center_);
    texts.push_back(to_string(i));
  }
  const int grid_num1 = grid1_->grid_data_.size();
  for (int i = 0; i < grid2_->grid_data_.size(); ++i) {
    auto& grid = grid2_->grid_data_[i];
    if (!grid.active_ || !grid.is_cur_relevant_) continue;
    pts.push_back(grid.center_);
    texts.push_back(to_string(i + grid_num1));
  }
}

}  // namespace fast_planner