#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <swarm_msgs/swarm_drone_basecoor.h>
#include <vector>
#include <map>
#include <Eigen/Eigen>

using std::vector;
using std::map;

class SwarmTfSim {
public:
  SwarmTfSim(ros::NodeHandle& nh) {

    nh.param("drone_num", drone_num_, 1);

    drone_num_ -= 1;  // The last one is for ground node and should be excluded

    init_sub_ = nh.subscribe("/pcl_render_node/init", 10, &SwarmTfSim::initPosCallback, this);
    basecoor_timer_ = nh.createTimer(ros::Duration(1.0), &SwarmTfSim::swarmTfCallback, this);

    for (int i = 0; i < drone_num_; ++i) {
      ros::Publisher bc_pub = nh.advertise<swarm_msgs::swarm_drone_basecoor>(
          "/swarm_sim_tf/basecoor_" + std::to_string(i + 1), 10);
      basecoor_pub_.push_back(bc_pub);
    }

    init_pts_[1] = Eigen::Vector3d(0, 0, 0);  // Drone 1's frame is aligned with world frame
  }

  ~SwarmTfSim() {
  }

private:
  void initPosCallback(const geometry_msgs::PointConstPtr& msg) {
    Eigen::Vector3d pos;
    pos << msg->x, msg->y, 0.0;
    int id = std::round(msg->z);
    if (id > 1) init_pts_[id] = pos;
  }

  void swarmTfCallback(const ros::TimerEvent& e) {
    if (init_pts_.size() < drone_num_) {
      ROS_ERROR("Initial positions are incomplete.");
      return;
    }

    // Compute and publish basecoor msg for each drone
    for (int i = 0; i < drone_num_; ++i) {
      swarm_msgs::swarm_drone_basecoor coors;
      for (int j = 0; j < drone_num_; ++j) {
        Eigen::Vector3d tf_ij = init_pts_[j + 1] - init_pts_[i + 1];
        geometry_msgs::Point pt;
        pt.x = tf_ij[0];
        pt.y = tf_ij[1];
        pt.z = tf_ij[2];
        coors.ids.push_back(j + 1);
        coors.drone_basecoor.push_back(pt);
        coors.drone_baseyaw.push_back(0.0);
      }
      coors.self_id = i + 1;
      basecoor_pub_[i].publish(coors);
    }
  }

  // Data -----
  ros::Subscriber init_sub_;
  ros::Timer basecoor_timer_;
  vector<ros::Publisher> basecoor_pub_;
  int drone_num_;
  map<int, Eigen::Vector3d> init_pts_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_node");
  ros::NodeHandle nh("~");

  SwarmTfSim swarm_tf_sim(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}