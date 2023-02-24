#include <active_perception/uniform_grid.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>

namespace fast_planner {

UniformGrid::UniformGrid(
    const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level) {

  this->edt_ = edt;

  // Read min, max, resolution here
  nh.param("sdf_map/box_min_x", min_[0], 0.0);
  nh.param("sdf_map/box_min_y", min_[1], 0.0);
  nh.param("sdf_map/box_min_z", min_[2], 0.0);
  nh.param("sdf_map/box_max_x", max_[0], 0.0);
  nh.param("sdf_map/box_max_y", max_[1], 0.0);
  nh.param("sdf_map/box_max_z", max_[2], 0.0);
  nh.param("partitioning/min_unknown", min_unknown_, 10000);
  nh.param("partitioning/min_frontier", min_frontier_, 100);
  nh.param("partitioning/min_free", min_free_, 3000);
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);
  nh.param("partitioning/w_unknown", w_unknown_, 3.5);

  double grid_size;
  nh.param("partitioning/grid_size", grid_size, 5.0);

  auto size = max_ - min_;

  // resolution_ = size / 3;
  for (int i = 0; i < 2; ++i) {
    int num = ceil(size[i] / grid_size);
    resolution_[i] = size[i] / double(num);
    for (int j = 1; j < level; ++j) resolution_[i] *= 0.5;
  }
  resolution_[2] = size[2];
  initialized_ = false;
  level_ = level;

  // path_finder_.reset(new Astar);
  // path_finder_->init(nh, edt);
}

UniformGrid::~UniformGrid() {
}

void UniformGrid::initGridData() {
  Eigen::Vector3d size = max_ - min_;
  for (int i = 0; i < 3; ++i) grid_num_(i) = ceil(size(i) / resolution_[i]);
  grid_data_.resize(grid_num_[0] * grid_num_[1] * grid_num_[2]);

  std::cout << "data size: " << grid_data_.size() << std::endl;
  std::cout << "grid num: " << grid_num_.transpose() << std::endl;
  std::cout << "resolution: " << resolution_.transpose() << std::endl;

  // Init each grid info
  for (int x = 0; x < grid_num_[0]; ++x) {
    for (int y = 0; y < grid_num_[1]; ++y) {
      for (int z = 0; z < grid_num_[2]; ++z) {
        Eigen::Vector3i id(x, y, z);
        auto& grid = grid_data_[toAddress(id)];

        Eigen::Vector3d pos;
        indexToPos(id, 0.5, pos);
        if (use_swarm_tf_) {
          pos = rot_sw_ * pos + trans_sw_;
        }

        grid.center_ = pos;
        grid.unknown_num_ = resolution_[0] * resolution_[1] * resolution_[2] /
                            pow(edt_->sdf_map_->getResolution(), 3);

        grid.is_prev_relevant_ = true;
        grid.is_cur_relevant_ = true;
        grid.need_divide_ = false;
        if (level_ == 1)
          grid.active_ = true;
        else
          grid.active_ = false;
      }
    }
  }
}

void UniformGrid::updateBaseCoor() {
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    // if (!grid.active_) continue;

    Eigen::Vector3i id;
    adrToIndex(i, id);

    // Compute vertices and box of grid in current drone's frame
    Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
    indexToPos(id, 0.0, left_bottom);
    indexToPos(id, 1.0, right_top);
    left_top[0] = left_bottom[0];
    left_top[1] = right_top[1];
    left_top[2] = left_bottom[2];
    right_bottom[0] = right_top[0];
    right_bottom[1] = left_bottom[1];
    right_bottom[2] = left_bottom[2];
    right_top[2] = left_bottom[2];

    vector<Eigen::Vector3d> vertices = { left_bottom, right_bottom, right_top, left_top };
    if (use_swarm_tf_) {
      for (auto& vert : vertices) vert = rot_sw_ * vert + trans_sw_;
    }

    Eigen::Vector3d vmin, vmax;
    vmin = vmax = vertices[0];
    for (int j = 1; j < vertices.size(); ++j) {
      for (int k = 0; k < 2; ++k) {
        vmin[k] = min(vmin[k], vertices[j][k]);
        vmax[k] = max(vmax[k], vertices[j][k]);
      }
    }
    grid.vertices_ = vertices;
    grid.vmin_ = vmin;
    grid.vmax_ = vmax;

    // Compute normals of four separating lines
    grid.normals_.clear();
    for (int j = 0; j < 4; ++j) {
      Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
      grid.normals_.push_back(dir);
    }
    // std::cout << "Vertices of grid " << toAddress(id) << std::endl;
    // for (auto v : grid.vertices_)
    //   std::cout << v.transpose() << "; ";
    // std::cout << "\nNormals: " << std::endl;
    // for (auto n : grid.normals_)
    //   std::cout << n.transpose() << "; ";
    // std::cout << "\nbox: " << grid.vmin_.transpose() << ", " << grid.vmax_.transpose()
    //           << std::endl;
  }
}

void UniformGrid::updateGridData(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids,
    vector<int>& parti_ids_all) {

  // parti_ids are ids of grids that are assigned to THIS drone and should be divided
  // parti_ids_all are ids of ALL grids that should be divided

  for (auto& grid : grid_data_) {
    grid.is_updated_ = false;
  }
  parti_ids.clear();

  bool reset = (level_ == 2);
  Vector3d update_min, update_max;
  edt_->sdf_map_->getUpdatedBox(update_min, update_max, reset);

  vector<Eigen::Vector3d> update_mins, update_maxs;
  edt_->sdf_map_->mm_->getChunkBoxes(update_mins, update_maxs, reset);

  // Rediscovered grid
  vector<int> rediscovered_ids;

  auto have_overlap = [](
      const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
    for (int m = 0; m < 2; ++m) {
      double bmin = max(min1[m], min2[m]);
      double bmax = min(max1[m], max2[m]);
      if (bmin > bmax + 1e-3) return false;
    }
    return true;
  };

  // For each grid, check overlap with updated box and update it if necessary
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    if (!grid.active_) continue;

    // Check overlap with updated boxes
    bool overlap = false;
    for (int j = 0; j < update_mins.size(); ++j) {
      if (have_overlap(grid.vmin_, grid.vmax_, update_mins[j], update_maxs[j])) {
        overlap = true;
        break;
      }
    }
    bool overlap_with_fov = have_overlap(grid.vmin_, grid.vmax_, update_min, update_max);
    if (!overlap && !overlap_with_fov) continue;

    // Update the grid
    Eigen::Vector3i idx;
    adrToIndex(i, idx);
    updateGridInfo(idx);

    if (grid.need_divide_) {
      parti_ids_all.push_back(i);
      grid.active_ = false;
    }

    // Rediscovered relevant grid
    if (!overlap_with_fov) continue;
    if (!grid.is_prev_relevant_ && grid.is_cur_relevant_ && level_ > 1) {
      rediscovered_ids.push_back(i);
      ROS_WARN("Grid %d is rediscovered", i);
    }
  }

  // Patch code. To avoid incomplete update of level 2 grids...
  for (auto id : extra_ids_) {
    if (grid_data_[id].is_updated_) continue;

    Eigen::Vector3i idx;
    adrToIndex(id, idx);
    updateGridInfo(idx);
  }
  extra_ids_.clear();

  // Update the list of relevant grid
  relevant_id_.clear();
  relevant_map_.clear();
  for (int i = 0; i < grid_data_.size(); ++i) {
    if (isRelevant(grid_data_[i])) {
      relevant_id_.push_back(i);
      relevant_map_[i] = 1;
    }
  }

  // Update the dominance grid of ego drone
  if (!initialized_) {
    if (drone_id == 1 && level_ == 1) grid_ids = relevant_id_;
    // else
    //   grid_ids = {};
    ROS_WARN("Init grid allocation.");
    initialized_ = true;
  } else {
    for (auto it = grid_ids.begin(); it != grid_ids.end();) {
      if (relevant_map_.find(*it) == relevant_map_.end()) {
        // Remove irrelevant ones
        // std::cout << "Remove irrelevant: " << *it << std::endl;
        it = grid_ids.erase(it);
      } else if (grid_data_[*it].need_divide_) {
        // Partition coarse grid
        // std::cout << "Remove divided: " << *it << std::endl;
        parti_ids.push_back(*it);
        it = grid_ids.erase(it);
        // grid_data_[*it].active_ = false;
      } else {
        ++it;
      }
    }
    // Add rediscovered ones
    grid_ids.insert(grid_ids.end(), rediscovered_ids.begin(), rediscovered_ids.end());

    // sort(grid_ids.begin(), grid_ids.end());
  }
}

void UniformGrid::updateGridInfo(const Eigen::Vector3i& id) {
  int adr = toAddress(id);
  auto& grid = grid_data_[adr];
  if (grid.is_updated_) {  // Ensure only one update to avoid repeated computation
    return;
  }
  grid.is_updated_ = true;

  grid.is_prev_relevant_ = grid.is_cur_relevant_;

  Eigen::Vector3d gmin, gmax;
  // indexToPos(id, 0.0, gmin);
  indexToPos(id, 1.0, gmax);  // Only the first 2 values of vmax is useful, should compute max here

  // Check if a voxel is inside the rotated box
  auto inside_box = [](const Eigen::Vector3d& vox, const GridInfo& grid) {
    // Check four separating planes(lines)
    for (int m = 0; m < 4; ++m) {
      if ((vox - grid.vertices_[m]).dot(grid.normals_[m]) <= 0.0) return false;
    }
    return true;
  };

  // Count known
  const double res = edt_->sdf_map_->getResolution();
  grid.center_.setZero();
  grid.unknown_num_ = 0;
  int free = 0;
  for (double x = grid.vmin_[0]; x <= grid.vmax_[0]; x += res) {
    for (double y = grid.vmin_[1]; y <= grid.vmax_[1]; y += res) {
      for (double z = grid.vmin_[2]; z <= gmax[2]; z += res) {

        Eigen::Vector3d pos(x, y, z);
        if (!inside_box(pos, grid)) continue;

        int state = edt_->sdf_map_->getOccupancy(pos);
        if (state == SDFMap::FREE) {
          free += 1;
        } else if (state == SDFMap::UNKNOWN) {
          grid.center_ = (grid.center_ * grid.unknown_num_ + pos) / (grid.unknown_num_ + 1);
          grid.unknown_num_ += 1;
        }
      }
    }
  }

  grid.is_cur_relevant_ = isRelevant(grid);

  // cout << "level: " << level_ << ", grid id: " << id.transpose() << ", adr: " << adr
  //      << ", unknown: " << grid.unknown_num_ << ", center: " << grid.center_.transpose()
  //      << ", rele: " << grid.is_cur_relevant_ << endl;

  if (level_ == 1 && grid.active_ && free > min_free_) {
    grid.need_divide_ = true;
  }
}

int UniformGrid::toAddress(const Eigen::Vector3i& id) {
  return id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
}

void UniformGrid::adrToIndex(const int& adr, Eigen::Vector3i& idx) {
  // id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
  int tmp_adr = adr;
  const int a = grid_num_(1) * grid_num_(2);
  const int b = grid_num_(2);

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void UniformGrid::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - min_(i)) / resolution_[i]);
}

void UniformGrid::indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos) {
  // inc: 0 for min, 1 for max, 0.5 for mid point
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + inc) * resolution_[i] + min_(i);
}

void UniformGrid::activateGrids(const vector<int>& ids) {
  for (auto id : ids) {
    grid_data_[id].active_ = true;
  }
  extra_ids_ = ids;  // To avoid incomplete update
}

bool UniformGrid::insideGrid(const Eigen::Vector3i& id) {
  // Check inside min max
  for (int i = 0; i < 3; ++i) {
    if (id[i] < 0 || id[i] >= grid_num_[i]) {
      return false;
    }
  }
  return true;
}

void UniformGrid::inputFrontiers(const vector<Eigen::Vector3d>& avgs) {
  for (auto& grid : grid_data_) {
    grid.contained_frontier_ids_.clear();
  }
  Eigen::Vector3i id;
  Eigen::Matrix3d Rt = rot_sw_.transpose();
  Eigen::Vector3d t_inv = -Rt * trans_sw_;

  for (int i = 0; i < avgs.size(); ++i) {
    Eigen::Vector3d pos = avgs[i];
    if (use_swarm_tf_) {
      pos = Rt * pos + t_inv;
    }
    posToIndex(pos, id);
    if (!insideGrid(id)) continue;
    auto& grid = grid_data_[toAddress(id)];
    grid.contained_frontier_ids_[i] = 1;
  }
}

bool UniformGrid::isRelevant(const GridInfo& grid) {
  // return grid.unknown_num_ >= min_unknown_ || grid.frontier_num_ >= min_frontier_;
  // return grid.unknown_num_ >= min_unknown_ || !grid.frontier_cell_nums_.empty();
  return grid.unknown_num_ >= min_unknown_ || !grid.contained_frontier_ids_.empty();
}

void UniformGrid::getCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& prev_first_grid,
    const vector<int>& grid_ids, Eigen::MatrixXd& mat) {
}

void UniformGrid::getGridTour(const vector<int>& ids, vector<Eigen::Vector3d>& tour) {
  tour.clear();
  for (int i = 0; i < ids.size(); ++i) {
    tour.push_back(grid_data_[ids[i]].center_);
  }
}

void UniformGrid::getFrontiersInGrid(const int& grid_id, vector<int>& ftr_ids) {
  // Find frontier having more than 1/4 within the first grid
  auto& first_grid = grid_data_[grid_id];
  ftr_ids.clear();
  // for (auto pair : first_grid.frontier_cell_nums_) {
  //   ftr_ids.push_back(pair.first);
  // }
  for (auto pair : first_grid.contained_frontier_ids_) {
    ftr_ids.push_back(pair.first);
  }
}

void UniformGrid::getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2) {

  Eigen::Vector3d p1 = min_;
  Eigen::Vector3d p2 = min_ + Eigen::Vector3d(max_[0] - min_[0], 0, 0);
  for (int i = 0; i <= grid_num_[1]; ++i) {
    Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(0, resolution_[1] * i, 0);
    Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(0, resolution_[1] * i, 0);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }

  p1 = min_;
  p2 = min_ + Eigen::Vector3d(0, max_[1] - min_[1], 0);
  for (int i = 0; i <= grid_num_[0]; ++i) {
    Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
    Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }
  for (auto& p : pts1) p[2] = 0.5;
  for (auto& p : pts2) p[2] = 0.5;
}

}  // namespace fast_planner
