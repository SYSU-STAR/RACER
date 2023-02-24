#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <string>

#include <lkh_tsp_solver/lkh_interface.h>
#include <lkh_tsp_solver/SolveTSP.h>

std::string tsp_dir_;
int drone_id_;

bool tspCallback(lkh_tsp_solver::SolveTSP::Request& req, lkh_tsp_solver::SolveTSP::Response& res) {

  solveTSPLKH(tsp_dir_.c_str());

  ROS_WARN("TSP server %d finish", drone_id_);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tsp_node");
  ros::NodeHandle nh("~");

  // Read mtsp file dir
  nh.param("exploration/tsp_dir", tsp_dir_, std::string("null"));
  nh.param("exploration/drone_id", drone_id_, 1);

  tsp_dir_ = tsp_dir_ + "/drone_" + std::to_string(drone_id_) + ".par";

  ros::ServiceServer tsp_server =
      nh.advertiseService("/solve_tsp_" + std::to_string(drone_id_), tspCallback);

  ROS_WARN("TSP server %d is ready.", drone_id_);
  ros::spin();

  return 1;
}
