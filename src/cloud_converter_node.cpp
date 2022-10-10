#include "../include/pcd2bag/cloud_converter.hpp"

#include <rosbag/bag.h>
#include <dirent.h>
#include <unistd.h>


void CloudConverter::readDirectory(const std::string& name, std::vector<std::string>& v)
{
  DIR* dirp = opendir(name.c_str());
  struct dirent *dp;

  while((dp = readdir(dirp)))
  {
    char *ext;
    ext = strrchr(dp->d_name, '.');

    if(strcmp(ext, ".pcd") == 0)
    {
      v.push_back(dp->d_name);
    }
    else
    {
      continue;
    }
  }

  std::sort(v.begin(), v.end());

  closedir(dirp);
}

void CloudConverter::loadPCD(std::string filename, pcl::PointCloud<PointType>::Ptr cloud)
{
    if(pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1)
    {
        ROS_ERROR_STREAM("Cannot read the file");
        return;
    }
}

void CloudConverter::PCDtoPC2(ros::Time t, pcl::PointCloud<PointType>::Ptr cloud, sensor_msgs::PointCloud2 &cloud_msg)
{
    if(cloud->empty())
        return;

    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = t;
    cloud_msg.header.frame_id = "lidar";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd2bag");
    ROS_INFO("\033[1;32m----> PCD to rosbag is started.\033[0m");

    ros::NodeHandle nh;
    std::string pointcloud_topic;
    std::string dataset_path;
    std::string save_path;
    std::string file_name;

    //Get params
    {
        nh.param<std::string>("/pcd2bag/pointcloud_topic", pointcloud_topic, "");
        nh.param<std::string>("/pcd2bag/dataset_path", dataset_path, "");
        nh.param<std::string>("/pcd2bag/save_path", save_path, "");
        nh.param<std::string>("/pcd2bag/file_name", file_name, "");
    }
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);

    CloudConverter CC;

    std::vector<std::string> file_list;
    CC.readDirectory(dataset_path, file_list);

    int index = 0;
    ros::Time t = ros::Time::now();

    rosbag::Bag bag;
    system((std::string("mkdir -p ") + save_path).c_str());
    std::string file_idx_name = file_name + std::to_string(index) + ".bag";
    bag.open(save_path + file_idx_name, rosbag::bagmode::Write);
    ROS_INFO_STREAM("Write " << file_idx_name.c_str() << "..");

    for(auto iter=file_list.begin(); iter!=file_list.end(); iter++)
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

        CC.loadPCD(dataset_path + *iter, cloud);

        t += ros::Duration(0.05);                    // +20Hz
        CC.PCDtoPC2(t, cloud, cloud_msg);

        bag.write(pointcloud_topic, t, cloud_msg);  // save pcd to rosbag
        pub_cloud.publish(cloud_msg);               // visualization

        if(bag.getSize() >= 1000000000)             // 1GB
        {
            bag.close();
            index++;

            file_idx_name = file_name + std::to_string(index) + ".bag";
            bag.open(save_path + file_idx_name, rosbag::bagmode::Write);
            ROS_INFO_STREAM("Write " << file_idx_name.c_str() << "..");
        }

        usleep(10);
    }
    bag.close();
    ROS_INFO_STREAM("Converting finished!");

    ros::spinOnce();

    return 0;
}
