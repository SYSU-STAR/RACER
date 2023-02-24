#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <string>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

using std::string;

std::string mtsp_dir1_;
std::string mtsp_dir2_;
std::string mtsp_dir3_;
int drone_id_, problem_id_;

bool mtspCallback(
    lkh_mtsp_solver::SolveMTSP::Request& req, lkh_mtsp_solver::SolveMTSP::Response& res) {

  if (req.prob == 1)
    solveMTSPWithLKH3(mtsp_dir1_.c_str());
  else if (req.prob == 2)
    solveMTSPWithLKH3(mtsp_dir2_.c_str());
  else if (req.prob == 3) {
    // solveMTSPWithLKH3(mtsp_dir3_.c_str());
    string cmd = "/usr/local/bin/LKH " + mtsp_dir3_;
    system(cmd.c_str());
  }

  // ROS_INFO("MTSP server %d solve prob", drone_id_);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mtsp_node");
  ros::NodeHandle nh("~");

  // Read mtsp file dir
  std::string mtsp_dir;
  nh.param("exploration/mtsp_dir", mtsp_dir, std::string("null"));
  nh.param("exploration/drone_id", drone_id_, 1);
  nh.param("exploration/problem_id", problem_id_, 1);

  mtsp_dir1_ = mtsp_dir + "/amtsp_" + std::to_string(drone_id_) + ".par";
  mtsp_dir2_ = mtsp_dir + "/amtsp2_" + std::to_string(drone_id_) + ".par";
  mtsp_dir3_ = mtsp_dir + "/amtsp3_" + std::to_string(drone_id_) + ".par";

  string service_name;
  if (problem_id_ == 1) {  // TSP
    service_name = "/solve_tsp_" + std::to_string(drone_id_);
  } else if (problem_id_ == 2) {  // ACVRP
    service_name = "/solve_acvrp_" + std::to_string(drone_id_);
  }
  ros::ServiceServer mtsp_server = nh.advertiseService(service_name, mtspCallback);

  ROS_WARN("MTSP server %d is ready.", drone_id_);
  ros::spin();

  return 1;
}
