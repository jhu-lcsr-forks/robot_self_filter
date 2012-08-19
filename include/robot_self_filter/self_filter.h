#ifndef __ROBOT_SELF_FILTER_SELF_FILTER_H
#define __ROBOT_SELF_FILTER_SELF_FILTER_H

#include <ros/ros.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <robot_self_filter/self_see_filter.h>

namespace robot_self_filter {
  class SelfFilter {

  public:
    SelfFilter(ros::NodeHandle nh = ros::NodeHandle("~"));
    ~SelfFilter();

  private:

    // Callback to do no filtering
    void noFilterCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    // Callback to do filtering
    void cloudCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    tf::TransformListener tf_;
    ros::NodeHandle nh_, root_handle_;

    tf::MessageFilter<pcl::PointCloud<pcl::PointXYZ> > *mn_;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > *sub_;

    filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> > *self_filter_;
    std::string sensor_frame_;
    double subsample_param_;

    ros::Publisher pointCloudPublisher_;
    ros::Subscriber no_filter_sub_;

    pcl::VoxelGrid<pcl::PointXYZ> grid_;
  };
}

#endif // ifndef __ROBOT_SELF_FILTER_SELF_FILTER_H
