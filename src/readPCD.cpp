#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_publisher");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_topic", 1);

  std::string folder_path;
  nh.getParam("folder_path", folder_path);
  std::cout<<"folder_path= "<<folder_path<<std::endl;

  // 查找指定文件夹下的所有PCD文件
  std::vector<std::string> pcd_files;
  for (boost::filesystem::directory_iterator it(folder_path); it != boost::filesystem::directory_iterator(); ++it)
  {
    if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".pcd")
    {
      pcd_files.push_back(it->path().string());
    }
  }

  // 发布PCD文件
  ros::Rate loop_rate(10);  // 发布频率
  PointCloudT::Ptr cloud(new PointCloudT);
  for (const auto& pcd_file : pcd_files)
  {
    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1)
    {
      ROS_ERROR_STREAM("Couldn't read file " << pcd_file);
      continue;
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    ROS_INFO_STREAM("Public pcd: "<< pcd_file);
    output.header.frame_id = "camera_init";

    pub.publish(output);
    loop_rate.sleep();
  }

  return 0;
}