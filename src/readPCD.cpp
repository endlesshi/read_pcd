#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <algorithm>
#include "ground_plane_fitting.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "pcd_publisher");
  ros::NodeHandle nh("~");
  ros::Publisher pub_p = nh.advertise<sensor_msgs::PointCloud2>("/pcd_topic", 10);
  ros::Publisher pub_ng = nh.advertise<sensor_msgs::PointCloud2>("/points_no_ground", 10);
  ros::Publisher pub_g = nh.advertise<sensor_msgs::PointCloud2>("/points_ground", 10);

  std::string folder_path;
  bool isBin;
  nh.param<std::string>("folder_path", folder_path, "/");
  nh.param<bool>("isBin", isBin, true);
  std::cout<<"folder_path= "<<folder_path<<std::endl;
  std::cout<<"isBin? "<<isBin<<std::endl;

  // 查找指定文件夹下的所有PCD文件
  std::vector<std::string> pcd_files;
  for (boost::filesystem::directory_iterator it(folder_path); it != boost::filesystem::directory_iterator(); ++it)
  {
    if(isBin){
      if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".bin")
      {
        pcd_files.push_back(it->path().string());
      }
    }
    else{
      if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".pcd")
      {
        pcd_files.push_back(it->path().string());
      }
    }
    
  }

  std::cout<<"pcd_files.size()= "<<pcd_files.size()<<std::endl; 
  ros::Rate loop_rate(10);  // 发布频率
  int num_pcd=0;
  std::sort(pcd_files.begin(), pcd_files.end());

  for (const auto& pcd_file : pcd_files)
  {
    PointCloudT::Ptr points (new PointCloudT);
    PointCloudT::Ptr notground_points (new PointCloudT);
    PointCloudT::Ptr ground_points (new PointCloudT);

    if(isBin){
      fstream input(pcd_file.c_str(), ios::in | ios::binary);
      if(!input.good()){
          cerr << "Couldn't read pcd_file: " << pcd_file << endl;
      }
      
      int i;
      for (i=0; input.good() && !input.eof(); i++) {
          pcl::PointXYZI point;
          input.read((char *) &point.x, 3*sizeof(float));
          input.read((char *) &point.intensity, sizeof(float));
          points->push_back(point);
      }
      input.close();
    }
    else{
      if (pcl::io::loadPCDFile<PointT>(pcd_file, *points) == -1)
      {
        ROS_ERROR_STREAM("Couldn't read file " << pcd_file);
        continue;
      }
    }
    cout << "Read "<<num_pcd<<" th pcd:" << pcd_file << endl;
    //Segment the ground
    GroundPlaneFit Seg;
    Seg.mainLoop(points,notground_points,ground_points);


    // 发布PCD文件
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*points, output);
    output.header.frame_id = "camera_init";
    pub_p.publish(output);


    pcl::toROSMsg(*notground_points, output);
    output.header.frame_id = "camera_init";
    pub_ng.publish(output);


    pcl::toROSMsg(*ground_points, output);
    output.header.frame_id = "camera_init";
    pub_g.publish(output);

    num_pcd++;
    loop_rate.sleep();
  }

  return 0;
}