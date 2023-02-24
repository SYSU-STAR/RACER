#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>

#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner {

MultiMapManager::MultiMapManager() {
}

MultiMapManager::~MultiMapManager() {
}

void MultiMapManager::setMap(SDFMap* map) {
  this->map_ = map;
}

void MultiMapManager::init() {
  node_.param("exploration/drone_id", drone_id_, 1);
  node_.param("exploration/vis_drone_id", vis_drone_id_, -1);
  node_.param("exploration/drone_num", map_num_, 2);
  node_.param("multi_map_manager/chunk_size", chunk_size_, 200);

  stamp_timer_ = node_.createTimer(ros::Duration(0.1), &MultiMapManager::stampTimerCallback, this);
  chunk_timer_ = node_.createTimer(ros::Duration(0.1), &MultiMapManager::chunkTimerCallback, this);

  stamp_pub_ = node_.advertise<plan_env::ChunkStamps>("/multi_map_manager/chunk_stamps_send", 10);
  chunk_pub_ = node_.advertise<plan_env::ChunkData>("/multi_map_manager/chunk_data_send", 5000);
  marker_pub_ = node_.advertise<visualization_msgs::Marker>(
      "/multi_map_manager/marker_" + std::to_string(drone_id_), 10);

  stamp_sub_ = node_.subscribe(
      "/multi_map_manager/chunk_stamps_recv", 10, &MultiMapManager::stampMsgCallback, this);
  chunk_sub_ = node_.subscribe("/multi_map_manager/chunk_data_recv", 5000,
      &MultiMapManager::chunkCallback, this, ros::TransportHints().tcpNoDelay());

  multi_map_chunks_.resize(map_num_);
  for (auto& data : multi_map_chunks_) {
    data.idx_list_ = {};
  }
  chunk_boxes_.resize(map_num_);
  for (auto& box : chunk_boxes_) {
    box.valid_ = false;
  }
  chunk_buffer_.resize(map_num_);
  buffer_map_.resize(map_num_);
  last_chunk_stamp_time_.resize(map_num_);
  for (auto time : last_chunk_stamp_time_) time = 0.0;

  // // Test the idx list operation

  // // Find missed
  // vector<int> self1 = { 1, 1000 };
  // vector<int> self2 = { 5, 100, 105, 105, 120, 300, 400, 600, 700, 1000 };
  // vector<vector<int>> selfs = { self1, self2 };

  // vector<int> other1 = {};
  // vector<int> other2 = { 10, 80 };
  // vector<int> other3 = { 10, 80, 150, 200, 501, 550 };
  // vector<vector<int>> others = { other1, other2, other3 };

  // for (auto sf : selfs) {
  //   for (auto ot : others) {
  //     vector<int> missed;
  //     findMissedChunkIds(sf, ot, missed);
  //     std::cout << "Missed result: ";
  //     for (auto id : missed) std::cout << id << ", ";
  //     std::cout << "" << std::endl;
  //   }
  // }

  // // Merge chunk ids
  // // result: 1, 1000,
  // // result: 1, 9, 81, 1000,
  // // result: 1, 9, 81, 149, 201, 500, 551, 1000,
  // std::cout << "Compute merged ids" << std::endl;
  // vector<int> input1 = { 1, 1000 };
  // vector<int> input2 = { 1, 9, 81, 1000 };
  // vector<int> input3 = { 1, 7, 85, 140, 250, 450, 560, 900 };
  // vector<vector<int>> inputs = { input1, input2, input3 };
  // for (auto ot : others) {
  //   for (auto ip : inputs) {
  //     vector<int> output;
  //     mergeChunkIds(ot, ip, output);
  //     std::cout << "Merged result: ";
  //     for (auto id : output) std::cout << id << ", ";
  //     std::cout << "" << std::endl;
  //   }
  // }
}

void MultiMapManager::updateMapChunk(const vector<uint32_t>& adrs) {
  adr_buffer_.insert(adr_buffer_.end(), adrs.begin(), adrs.end());

  if (adr_buffer_.size() >= chunk_size_) {
    // Insert chunk from too long buffer
    int i = 0;
    for (; i + chunk_size_ < adr_buffer_.size(); i += chunk_size_) {
      MapChunk chunk;
      chunk.voxel_adrs_.insert(
          chunk.voxel_adrs_.end(), adr_buffer_.begin() + i, adr_buffer_.begin() + i + chunk_size_);
      chunk.idx_ = multi_map_chunks_[drone_id_ - 1].chunks_.size() + 1;
      // if (drone_id_ == 1) std::cout << "Drone 1 insert chunk " << chunk.idx_ << std::endl;
      chunk.need_query_ = true;
      chunk.empty_ = false;
      multi_map_chunks_[drone_id_ - 1].chunks_.push_back(chunk);
    }
    if (multi_map_chunks_[drone_id_ - 1].idx_list_.empty()) {
      multi_map_chunks_[drone_id_ - 1].idx_list_ = { 1, 1 };
    }
    multi_map_chunks_[drone_id_ - 1].idx_list_.back() =
        multi_map_chunks_[drone_id_ - 1].chunks_.back().idx_;

    // std::cout << "idx list: ";
    // for (auto id : multi_map_chunks_[drone_id_ - 1].idx_list_) std::cout << id << ", ";
    // std::cout << "" << std::endl;
    // std::cout << "chunk size: " << multi_map_chunks_[drone_id_ - 1].chunks_.size() << std::endl;

    // Remove already inserted data
    vector<uint32_t> tmp;
    tmp.insert(tmp.end(), adr_buffer_.begin() + i, adr_buffer_.end());
    adr_buffer_ = tmp;
  }
}

void MultiMapManager::stampTimerCallback(const ros::TimerEvent& e) {
  // Send stamp of chunks to other drones
  plan_env::ChunkStamps msg;
  msg.from_drone_id = drone_id_;
  msg.time = ros::Time::now().toSec();

  for (auto chunks : multi_map_chunks_) {
    plan_env::IdxList idx_list;
    idx_list.ids = chunks.idx_list_;
    msg.idx_lists.push_back(idx_list);
  }
  stamp_pub_.publish(msg);

  return;

  // Test, visualize the chunks
  static int pub_num = 0;
  static int pub_id = 0;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.ns = "own";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1;
  mk.color.g = 0;
  mk.color.b = 0;
  mk.color.a = 0.5;
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  auto& data = multi_map_chunks_[drone_id_ - 1];
  if (pub_num < data.chunks_.size()) {
    for (int i = pub_num; i < data.chunks_.size(); ++i) {
      // std::cout << "Chunk size : " << chunk.voxel_adrs_.size() << std::endl;
      auto& chunk = data.chunks_[i];
      // std::cout << chunk.stamp_ - first_stamp << ", ";

      for (auto adr : chunk.voxel_adrs_) {
        Eigen::Vector3i idx;
        Eigen::Vector3d pos;
        adrToIndex(adr, idx);
        map_->indexToPos(idx, pos);
        if (map_->getOccupancy(idx) == SDFMap::OCCUPIED) {
          geometry_msgs::Point pt;
          pt.x = pos[0];
          pt.y = pos[1];
          pt.z = pos[2];
          mk.points.push_back(pt);
        }
      }
    }
    if (!mk.points.empty()) {
      pub_num = data.chunks_.size();
      mk.id = pub_id++;
      marker_pub_.publish(mk);
    }
  }
}

void MultiMapManager::stampMsgCallback(const plan_env::ChunkStampsConstPtr& msg) {
  if (msg->from_drone_id == drone_id_) return;
  if (drone_id_ == map_num_) return;  // Ground node does not send chunk

  // auto t1 = ros::Time::now();

  // Check msg time to avoid overwhelming
  if (msg->time - last_chunk_stamp_time_[msg->from_drone_id - 1] < 0.3) return;
  last_chunk_stamp_time_[msg->from_drone_id - 1] = msg->time;

  // Check others' stamp info and send chunks unknown by them
  for (int i = 0; i < multi_map_chunks_.size(); ++i) {
    if (i == msg->from_drone_id - 1) continue;
    vector<int> missed;
    findMissedChunkIds(multi_map_chunks_[i].idx_list_, msg->idx_lists[i].ids, missed);
    sendChunks(i + 1, msg->from_drone_id, missed);
  }

  // ROS_ERROR("Stamp time: %lf", (ros::Time::now() - t1).toSec());
}

void MultiMapManager::findMissedChunkIds(
    const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids) {
  // Compute the complement set of other idx
  if (other_idx_list.empty()) {
    miss_ids = self_idx_list;
    return;
  }

  vector<int> not_in_other;
  if (other_idx_list[0] > 1) {
    not_in_other.push_back(1);
    not_in_other.push_back(other_idx_list[0] - 1);
  }
  for (int i = 1; i < other_idx_list.size(); i += 2) {
    not_in_other.push_back(other_idx_list[i] + 1);
    if (i == other_idx_list.size() - 1) {
      int infinite = std::numeric_limits<int>::max();
      not_in_other.push_back(infinite);
    } else {
      not_in_other.push_back(other_idx_list[i + 1] - 1);
    }
  }

  // Compute the intersection of self and not_in_other (brute-force, O(n^2))
  for (int i = 0; i < self_idx_list.size(); i += 2) {
    for (int j = 0; j < not_in_other.size(); j += 2) {
      int minr, maxr;
      if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j],
              not_in_other[j + 1], minr, maxr)) {
        miss_ids.push_back(minr);
        miss_ids.push_back(maxr);
      }
    }
  }
}

bool MultiMapManager::findIntersect(
    const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr) {
  minr = max(min1, min2);
  maxr = min(max1, max2);
  if (minr <= maxr) return true;
  return false;
}

void MultiMapManager::sendChunks(
    const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list) {
  auto& data = multi_map_chunks_[chunk_drone_id - 1];

  for (int i = 0; i < idx_list.size(); i += 2) {
    for (int j = idx_list[i]; j <= idx_list[i + 1]; ++j) {
      plan_env::ChunkData msg;
      msg.from_drone_id = drone_id_;
      msg.to_drone_id = to_drone_id;
      msg.chunk_drone_id = chunk_drone_id;
      msg.idx = data.chunks_[j - 1].idx_;
      msg.voxel_adrs = data.chunks_[j - 1].voxel_adrs_;
      if (chunk_drone_id == drone_id_ && data.chunks_[j - 1].need_query_) {
        // Should query the occ info in map if they are still empty
        getOccOfChunk(data.chunks_[j - 1].voxel_adrs_, data.chunks_[j - 1].voxel_occ_);
        data.chunks_[j - 1].need_query_ = false;
      }
      msg.voxel_occ_ = data.chunks_[j - 1].voxel_occ_;

      // // Swarm communication
      // msg.pos_x = drone_pos_[0];
      // msg.pos_y = drone_pos_[1];
      // msg.pos_z = drone_pos_[2];

      chunk_pub_.publish(msg);
      // std::cout << "Drone " << drone_id_ << " send chunk " << msg.idx << " of drone "
      //           << int(msg.chunk_drone_id) << " to drone " << int(msg.to_drone_id) << std::endl;
    }
  }

  // for (int i = idx; i < data.chunks_.size(); ++i) {
  // }
}

void MultiMapManager::getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs) {
  for (auto adr : adrs) {
    uint8_t occ = map_->md_->occupancy_buffer_[adr] > map_->mp_->min_occupancy_log_ ? 1 : 0;
    occs.push_back(occ);
  }
}

void MultiMapManager::chunkCallback(const plan_env::ChunkDataConstPtr& msg) {
  // Receive chunks from other drones, store them in chunk buffer
  if (msg->from_drone_id == drone_id_) return;
  // if(msg->to_drone_id != drone_id_) return;

  // Ignore chunks that are in the insertion buffer
  if (buffer_map_[msg->chunk_drone_id - 1].find(msg->idx) !=
      buffer_map_[msg->chunk_drone_id - 1].end())
    return;

  // ROS_ERROR("received msg idx: %d, from %d to %d", msg->idx, msg->from_drone_id,
  // msg->to_drone_id);
  chunk_buffer_[msg->chunk_drone_id - 1].push_back(*msg);
  buffer_map_[msg->chunk_drone_id - 1][msg->idx] = 1;

  return;
}

void MultiMapManager::chunkTimerCallback(const ros::TimerEvent& e) {

  // Not process chunk until swarm basecoor transform is available
  Eigen::Vector4d tmp;
  // if (!map_->getBaseCoor(1, tmp)) {
  //   ROS_WARN("basecoor not available yet.");
  //   return;
  // }

  // auto t1 = ros::Time::now();

  // Process chunks in buffers
  for (int i = 0; i < chunk_buffer_.size(); ++i) {
    auto& buffer = chunk_buffer_[i];
    if (buffer.empty()) continue;

    // Compute the idx list of buffered chunks
    sort(buffer.begin(), buffer.end(),
        [](const plan_env::ChunkData& chunk1, const plan_env::ChunkData& chunk2) {
          return chunk1.idx < chunk2.idx;
        });
    vector<int> idx_list = { int(buffer.front().idx) };
    int last_idx = idx_list[0];
    for (int j = 1; j < buffer.size(); ++j) {
      if (buffer[j].idx - last_idx > 1) {
        idx_list.push_back(last_idx);
        idx_list.push_back(buffer[j].idx);
      }
      last_idx = buffer[j].idx;
    }
    idx_list.push_back(last_idx);

    // std::cout << "process drone " << i + 1 << "'s chunks, input idx list: ";
    // for (auto id : idx_list) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    // Update ChunksData's chunks_
    auto& chunks_data = multi_map_chunks_[i];

    // std::cout << "self idx list " << i + 1 << ": ";
    // for (auto id : chunks_data.idx_list_) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    // Add placeholder for chunks
    int len_inc = last_idx - chunks_data.chunks_.size();
    for (int j = 0; j < len_inc; ++j) {
      chunks_data.chunks_.push_back(MapChunk());
      auto& back_chunk = chunks_data.chunks_.back();
      back_chunk.idx_ = chunks_data.chunks_.size();
      back_chunk.empty_ = true;
      back_chunk.need_query_ = false;
    }

    // Process data in buffer
    for (auto msg : buffer) {
      auto& chunk = chunks_data.chunks_[msg.idx - 1];
      if (chunk.empty_) {  // Only insert a chunk once
        chunk.voxel_adrs_ = msg.voxel_adrs;
        chunk.voxel_occ_ = msg.voxel_occ_;
        insertChunkToMap(chunk, msg.chunk_drone_id);
        chunk.empty_ = false;
      }
    }

    // Update ChunksData's idx_list_
    vector<int> union_list;
    mergeChunkIds(idx_list, chunks_data.idx_list_, union_list);
    chunks_data.idx_list_ = union_list;

    // std::cout << "merged idx list " << i + 1 << ": ";
    // for (auto id : union_list) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    buffer.clear();
    buffer_map_[i].clear();
  }
  // ROS_ERROR("chunk time: %lf", (ros::Time::now() - t1).toSec());
}

void MultiMapManager::mergeChunkIds(
    const vector<int>& list1, const vector<int>& list2, vector<int>& output) {

  // std::cout << "list1: ";
  // for (auto id : list1) std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // std::cout << "list2: ";
  // for (auto id : list2) std::cout << id << ", ";
  // std::cout << "" << std::endl;

  if (list1.empty()) {
    output = list2;
    return;
  }

  output = list1;
  int tmp1, tmp2;
  for (int i = 0; i < list2.size(); i += 2) {
    // For each interval in list2, merge it into output list
    bool intersect = false;
    for (int j = 0; j < output.size(); j += 2) {
      if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2)) {
        output[j] = min(output[j], list2[i]);
        output[j + 1] = max(output[j + 1], list2[i + 1]);
        intersect = true;
      }
    }
    if (!intersect) {  // Insert the interval in appropriate position
      vector<int> tmp = { list2[i], list2[i + 1] };
      if (list2[i + 1] < output.front()) {
        output.insert(output.begin(), tmp.begin(), tmp.end());
      } else if (list2[i] > output.back()) {
        output.insert(output.end(), tmp.begin(), tmp.end());
      } else {
        for (auto iter = output.begin() + 1; iter != output.end(); iter += 2) {
          if (*iter < list2[i] && *(iter + 1) > list2[i + 1]) {
            output.insert(iter + 1, tmp.begin(), tmp.end());
            break;
          }
        }
      }
    }
    // Remove redundant idx
    for (auto iter = output.begin() + 1; iter != output.end() - 1;) {
      if (*iter >= *(iter + 1) - 1) {
        iter = output.erase(iter);
        iter = output.erase(iter);
      } else {
        iter += 2;
      }
    }
    // std::cout << "output: ";
    // for (auto id : output) std::cout << id << ", ";
    // std::cout << "" << std::endl;
  }
}

void MultiMapManager::adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx) {
  // x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z
  uint32_t tmp_adr = adr;
  const int a = map_->mp_->map_voxel_num_[1] * map_->mp_->map_voxel_num_[2];
  const int b = map_->mp_->map_voxel_num_[2];

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void MultiMapManager::insertChunkToMap(const MapChunk& chunk, const int& drone_id) {

  // // Transform from other drone's local frame to this drone's
  // Eigen::Vector4d transform;
  // map_->getBaseCoor(drone_id, transform);

  // double yaw = transform[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = transform.head<3>();

  for (int i = 0; i < chunk.voxel_adrs_.size(); ++i) {
    // Insert occ info

    auto& adr = chunk.voxel_adrs_[i];

    Eigen::Vector3i idx;
    adrToIndex(adr, idx);

    Eigen::Vector3d pos;
    map_->indexToPos(idx, pos);

    // pos = rot * pos + trans;
    if (!map_->isInMap(pos)) continue;

    map_->posToIndex(pos, idx);
    auto adr_tf = map_->toAddress(idx);

    // map_->md_->occupancy_buffer_[adr] =
    //     chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;
    map_->md_->occupancy_buffer_[adr_tf] =
        chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;

    // Update the chunk box

    if (chunk_boxes_[drone_id - 1].valid_) {
      for (int k = 0; k < 3; ++k) {
        chunk_boxes_[drone_id - 1].min_[k] = min(chunk_boxes_[drone_id - 1].min_[k], pos[k]);
        chunk_boxes_[drone_id - 1].max_[k] = max(chunk_boxes_[drone_id - 1].max_[k], pos[k]);
      }
    } else {
      chunk_boxes_[drone_id - 1].min_ = chunk_boxes_[drone_id - 1].max_ = pos;
      chunk_boxes_[drone_id - 1].valid_ = true;
    }

    // Update the all box
    for (int k = 0; k < 3; ++k) {
      map_->md_->all_min_[k] = min(map_->md_->all_min_[k], pos[k]);
      map_->md_->all_max_[k] = max(map_->md_->all_max_[k], pos[k]);
    }
    // Inflate for the occupied
    if (chunk.voxel_occ_[i] == 1) {
      static const int inf_step = ceil(map_->mp_->obstacles_inflation_ / map_->mp_->resolution_);
      for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
        for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
          for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z) {
            Eigen::Vector3i inf_pt(idx[0] + inf_x, idx[1] + inf_y, idx[2] + inf_z);
            if (!map_->isInMap(inf_pt)) continue;
            int inf_adr = map_->toAddress(inf_pt);
            map_->md_->occupancy_buffer_inflate_[inf_adr] = 1;
          }
      // vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
      // map_->inflatePoint(idx, inf_step, inf_pts);

      // for (auto inf_pt : inf_pts) {
      //   if (!map_->isInMap(inf_pt)) continue;
      //   int idx_inf = map_->toAddress(inf_pt);
      //   if (idx_inf >= 0 &&
      //       idx_inf < map_->mp_->map_voxel_num_(0) * map_->mp_->map_voxel_num_(1) *
      //                     map_->mp_->map_voxel_num_(2)) {
      //     map_->md_->occupancy_buffer_inflate_[idx_inf] = 1;
      //   }
      // }
    }
  }
}

void MultiMapManager::getChunkBoxes(
    vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset) {
  for (auto& box : chunk_boxes_) {
    if (box.valid_) {
      mins.push_back(box.min_);
      maxs.push_back(box.max_);
      if (reset) box.valid_ = false;
    }
  }
}

// MultiMapManager::
}  // namespace fast_planner