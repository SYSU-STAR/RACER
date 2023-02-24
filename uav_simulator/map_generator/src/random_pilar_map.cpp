#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x_;
uniform_real_distribution<double> rand_y_;
uniform_real_distribution<double> rand_w_;

int obs_num_, retry_num_;
double x_size_, y_size_, min_r_, max_r_, resolution_;
double fb_min_x_, fb_max_x_, fb_min_y_, fb_max_y_;
double min_x_, max_x_, min_y_, max_y_;

ros::Publisher all_map_pub_;
sensor_msgs::PointCloud2 map_msg_;
pcl::PointCloud<pcl::PointXYZ> map_cloud_;

vector<Eigen::Vector3d> points_;

void generatePilarMap() {
  pcl::PointXYZ pt_random;

  rand_x_ = uniform_real_distribution<double>(min_x_, max_x_);
  rand_y_ = uniform_real_distribution<double>(min_y_, max_y_);
  rand_w_ = uniform_real_distribution<double>(min_r_, max_r_);

  for (double x = -16; x <= 16; x += resolution_)
    for (double y = -16; y <= 16; y += resolution_) {
      pt_random.x = x;
      pt_random.y = y;
      pt_random.z = 0.0;
      map_cloud_.points.push_back(pt_random);
    }

  // generate polar obs
  int gen_num = 0;
  int fail_num = 0;
  while (gen_num < obs_num_) {
    double x, y, w;
    x = rand_x_(eng);
    y = rand_y_(eng);
    w = rand_w_(eng);

    // Not in drone's initializaton area
    if (x < fb_max_x_ && x > fb_min_x_ && y < fb_max_y_ && y > fb_min_y_) continue;

    // Not in previously sampled regions
    x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
    Eigen::Vector3d pt(x, y, w);
    bool occupied = false;
    for (auto pre : points_) {
      if ((pre - pt).head<2>().norm() < 1.414 * (pre[2] + pt[2]) + 2.0) {
        occupied = true;
        break;
      }
    }
    if (occupied) {
      if (++fail_num > retry_num_)
        return;
      else
        continue;
    }
    points_.push_back(pt);

    int wid_num_ = ceil(w / resolution_);
    for (int r = -wid_num_; r < wid_num_; r++)
      for (int s = -wid_num_; s < wid_num_; s++) {
        int hei_num_ = ceil(3.0 / resolution_);
        for (int t = -10; t < hei_num_; t++) {
          pt_random.x = x + r * resolution_ + 1e-2;
          pt_random.y = y + s * resolution_ + 1e-2;
          pt_random.z = (t + 0.5) * resolution_ + 1e-2;
          map_cloud_.points.push_back(pt_random);
        }
      }
    gen_num++;
    fail_num = 0;
  }

  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  ROS_WARN("Finished generate random map ");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilar_map");
  ros::NodeHandle n("~");

  all_map_pub_ = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  n.param("map/x_size", x_size_, 50.0);
  n.param("map/y_size", y_size_, 50.0);
  n.param("map/obs_num", obs_num_, 30);
  n.param("map/retry_num", retry_num_, 30);
  n.param("map/resolution", resolution_, 0.1);
  n.param("map/min_r", min_r_, 0.3);
  n.param("map/max_r", max_r_, 0.8);
  n.param("map/fb_min_x", fb_min_x_, 0.8);
  n.param("map/fb_max_x", fb_max_x_, 0.8);
  n.param("map/fb_min_y", fb_min_y_, 0.8);
  n.param("map/fb_max_y", fb_max_y_, 0.8);

  min_x_ = -x_size_ / 2.0;
  max_x_ = +x_size_ / 2.0;

  min_y_ = -y_size_ / 2.0;
  max_y_ = +y_size_ / 2.0;

  obs_num_ = min(obs_num_, (int)x_size_ * 10);

  ros::Duration(0.5).sleep();

  // init random device
  int seed;
  n.param("map/seed", seed, -1);
  if (seed < 0) {
    seed = rd() % INT32_MAX;
  }
  std::cout << "map seed: " << seed << std::endl;
  eng = default_random_engine(seed);

  generatePilarMap();
  while (ros::ok()) {
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = "world";
    all_map_pub_.publish(map_msg_);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ROS_WARN_THROTTLE(1, "seed: %d", seed);
  }
}
