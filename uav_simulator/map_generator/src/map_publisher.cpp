#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
string file_name;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node;

  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  file_name = argv[1];

  // file_name = "/home/boboyu/workspaces/catkin_ws/src/uav_simulator/map_generator/resource/tmp.pcd";
  // ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 10, true);

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }

  // // Process map
  // for (int i = 0; i < cloud.points.size(); ++i)
  // {
  //   auto pt = cloud.points[i];
  //   pcl::PointXYZ pr;
  //   pr.x = pt.x;
  //   pr.y = -pt.z;
  //   pr.z = pt.y;
  //   cloud.points[i] = pr;
  // }

  for (double x = -7; x <= 7; x += 0.1)
    for (double y = -15; y <= 15; y += 0.1)
    {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }

  // cout << "Publishing map..." << endl;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";

  while (ros::ok())
  {
    ros::Duration(0.2).sleep();
    cloud_pub.publish(msg);
  }

  cout << "finish publish map." << endl;

  return 0;
}