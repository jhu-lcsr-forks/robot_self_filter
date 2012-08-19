/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <ros/ros.h>
#include <sstream>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <robot_self_filter/self_see_filter.h>
#include <robot_self_filter/self_filter.h>

using namespace robot_self_filter;

SelfFilter::SelfFilter(ros::NodeHandle nh) : 
  nh_ (nh)
{
  // Get filter parameters
  nh_.param<std::string> ("sensor_frame", sensor_frame_, std::string ());
  nh_.param<double> ("subsample_value", subsample_param_, 0.01);

  // Construct the actual self-filter
  self_filter_ = new filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> > (nh_);

  // Set up message filters to delay message until TF data is available
  sub_ = new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > (root_handle_, "cloud_in", 1);	
  mn_ = new tf::MessageFilter<pcl::PointCloud<pcl::PointXYZ> > (*sub_, tf_, "", 1);

  // Construct publisher
  pointCloudPublisher_ = root_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_out", 1);

  // Get the frames in the URDF
  std::vector<std::string> frames;
  self_filter_->getSelfMask()->getLinkNames(frames);

  if (frames.empty()) {
    ROS_WARN ("No valid URDF frames available, not performing filtering.");
    no_filter_sub_ = root_handle_.subscribe<pcl::PointCloud<pcl::PointXYZ> >("cloud_in", 1, boost::bind(&SelfFilter::noFilterCallback, this, _1));
  } else {
    ROS_INFO ("Received %d URDF frames to filter out of the point cloud.", (int)frames.size() );
    mn_->setTargetFrames (frames);
    mn_->registerCallback (boost::bind (&SelfFilter::cloudCallback, this, _1));
  }
}

SelfFilter::~SelfFilter()
{
  delete self_filter_;
  delete mn_;
  delete sub_;
}

void SelfFilter::noFilterCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  pointCloudPublisher_.publish(cloud);
  ROS_DEBUG("Self filter publishing unfiltered frame");
}

void SelfFilter::cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  ROS_DEBUG ("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec ());
  std::vector<int> mask;
  ros::WallTime tm = ros::WallTime::now ();

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

  if (subsample_param_ != 0) {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    // Set up the downsampling filter
    grid_.setLeafSize (subsample_param_, subsample_param_, subsample_param_);     // 1cm leaf size
    grid_.setInputCloud (cloud);
    grid_.filter (*cloud_downsampled);

    self_filter_->updateWithSensorFrame (*cloud_downsampled, *cloud_filtered, sensor_frame_);
  } else {
    self_filter_->updateWithSensorFrame (*cloud, *cloud_filtered, sensor_frame_);
  }      

  double sec = (ros::WallTime::now() - tm).toSec ();

  pointCloudPublisher_.publish (cloud_filtered);
  ROS_DEBUG ("Self filter: reduced %d points to %d points in %f seconds", (int)cloud->points.size(), (int)cloud_filtered->points.size (), sec);
}
