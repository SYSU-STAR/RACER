#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>
#include <traj_utils/planning_visualization.h>
#include <swarmtal_msgs/drone_onboard_command.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;

ros::Publisher traj_pub, swarm_pos_cmd_pub, point_pub, vel_pub, acc_pub;

void displayTrajWithColor(
    vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_cmd");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  traj_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  swarm_pos_cmd_pub =
      node.advertise<swarmtal_msgs::drone_onboard_command>("/drone_commander/onboard_command", 50);
  point_pub = node.advertise<geometry_msgs::Point>("/send_cmd/pos", 10);
  vel_pub = node.advertise<geometry_msgs::Point>("/send_cmd/vel", 10);
  acc_pub = node.advertise<geometry_msgs::Point>("/send_cmd/acc", 10);

  // nh.param("send_cmd/init_x", init_x_, 0.0);

  ros::Duration(1.0).sleep();

  // Generate 'eight' shape trajectory

  const double rd = 1.0;
  const int pt_num = 17;
  const double max_vel = 1.0;
  Eigen::MatrixXd waypoints(pt_num, 3);
  waypoints.row(0) = Eigen::Vector3d(0, 0, 0.8);  // Start

  // First round
  waypoints.row(1) = Eigen::Vector3d(rd, rd, 1.0);
  waypoints.row(2) = Eigen::Vector3d(2 * rd, 0, 1.2);
  waypoints.row(3) = Eigen::Vector3d(3 * rd, -rd, 1.5);
  waypoints.row(4) = Eigen::Vector3d(4 * rd, 0, 1.8);
  waypoints.row(5) = Eigen::Vector3d(3 * rd, rd, 1.6);
  waypoints.row(6) = Eigen::Vector3d(2 * rd, 0, 1.2);
  waypoints.row(7) = Eigen::Vector3d(rd, -rd, 1.0);
  waypoints.row(8) = Eigen::Vector3d(0, 0, 0.8);

  // Second round
  waypoints.row(9) = Eigen::Vector3d(rd, rd, 1.0);
  waypoints.row(10) = Eigen::Vector3d(2 * rd, 0, 1.2);
  waypoints.row(11) = Eigen::Vector3d(3 * rd, -rd, 1.5);
  waypoints.row(12) = Eigen::Vector3d(4 * rd, 0, 1.8);
  waypoints.row(13) = Eigen::Vector3d(3 * rd, rd, 1.6);
  waypoints.row(14) = Eigen::Vector3d(2 * rd, 0, 1.2);
  waypoints.row(15) = Eigen::Vector3d(rd, -rd, 1.0);
  waypoints.row(16) = Eigen::Vector3d(0, 0, 0.8);

  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    times[i] = (waypoints.row(i + 1) - waypoints.row(i)).norm() / max_vel;
  }
  times[0] *= 1.5;
  times[pt_num - 2] *= 1.5;
  Eigen::Vector3d zero(0, 0, 0);

  PolynomialTraj traj;
  PolynomialTraj::waypointsTraj(waypoints, Eigen::Vector3d(0.05,0,0), zero, zero, zero, times, traj);

  vector<Eigen::Vector3d> points;
  traj.getSamplePoints(points);
  displayTrajWithColor(points, 0.05, Eigen::Vector4d(1, 0, 0, 1), 0);

  // Send cmd
  double duration = traj.getTotalTime();
  ROS_WARN(
      "Start to send cmd in 1 second, traj mean vel is %lf, duration is: %lf", max_vel, duration);
  ros::Duration(1.0).sleep();

  ROS_WARN("Move to start point");
  for (int i = 0; i < 250; ++i) {
    swarmtal_msgs::drone_onboard_command _cmd;
    _cmd.command_type = 0;
    _cmd.param1 = waypoints(0, 0) * 10000;
    _cmd.param2 = waypoints(0, 1) * 10000;
    _cmd.param3 = waypoints(0, 2) * 10000;
    _cmd.param4 = 0.0 * 10000;
    _cmd.param5 = 0.0 * 10000;
    _cmd.param6 = 0.0 * 10000;
    _cmd.param7 = 0.0 * 10000;
    _cmd.param8 = 0.0 * 10000;
    _cmd.param9 = 0.0 * 10000;
    _cmd.param10 = 0.0 * 10000;
    swarm_pos_cmd_pub.publish(_cmd);
    ros::Duration(0.02).sleep();
  }

  ROS_WARN("Start traj cmd.");

  auto t1 = ros::Time::now();
  double tn = (ros::Time::now() - t1).toSec();
  Eigen::Vector3d last_pos(0, 0, 0.5);

  while (ros::ok() && tn <= duration && tn > 0.0) {
    auto pos = traj.evaluate(tn, 0);
    auto vel = traj.evaluate(tn, 1);
    auto acc = traj.evaluate(tn, 2);

    swarmtal_msgs::drone_onboard_command _cmd;
    _cmd.command_type = 0;
    _cmd.param1 = pos(0) * 10000;
    _cmd.param2 = pos(1) * 10000;
    _cmd.param3 = pos(2) * 10000;

    bool enable_yaw = true;
    double yaw = 0.0;
    if (enable_yaw) {
      auto dir = pos - last_pos;
      if (dir.norm() > 1e-3) {
        yaw = atan2(dir[1], dir[0]);
      }
      last_pos = pos;
      _cmd.param4 = -yaw * 10000;
    } else {
      _cmd.param4 = 666666;
    }

    _cmd.param5 = vel(0) * 10000;
    _cmd.param6 = vel(1) * 10000;
    _cmd.param7 = vel(2) * 10000;

    _cmd.param8 = acc(0) * 10000;
    _cmd.param9 = acc(1) * 10000;
    _cmd.param10 = 0;//acc(2) * 10000;

    swarm_pos_cmd_pub.publish(_cmd);
    ros::Duration(0.02).sleep();
    tn = (ros::Time::now() - t1).toSec();

    std::cout << "Pos: " << pos.transpose() << std::endl;
    std::cout << "Vel: " << vel.transpose() << std::endl;
    std::cout << "Acc: " << acc.transpose() << std::endl;
    std::cout << "Yaw: " << yaw << std::endl;
    std::cout << "" << std::endl;

    // Vis
    geometry_msgs::Point pt;
    pt.x = pos[0];
    pt.y = pos[1];
    pt.z = pos[2];
    point_pub.publish(pt);
    pt.x = vel[0];
    pt.y = vel[1];
    pt.z = vel[2];
    vel_pub.publish(pt);
    pt.x = acc[0];
    pt.y = acc[1];
    pt.z = acc[2];
    acc_pub.publish(pt);
  }

  ROS_WARN("Send cmd done.");

  return 0;
}
