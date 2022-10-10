#ifndef CLOUD_CONVERTER_HPP
#define CLOUD_CONVERTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <iostream>

typedef pcl::PointXYZ PointType;

class CloudConverter
{
public:
    CloudConverter(){};
    void readDirectory(const std::string& name, std::vector<std::string>& v);
    void loadPCD(std::string filename, pcl::PointCloud<PointType>::Ptr cloud);
    void PCDtoPC2(ros::Time t, pcl::PointCloud<PointType>::Ptr cloud, sensor_msgs::PointCloud2 &cloud_msg);
};

#endif //CLOUD_CONVERTER_HPP
