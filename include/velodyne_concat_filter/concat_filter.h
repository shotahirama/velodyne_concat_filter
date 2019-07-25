/*
 * Copyright 2019 Shota Hirama
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VELODYNE_CONCAT_FILTER_H_
#define VELODYNE_CONCAT_FILTER_H_

#include <mutex>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

namespace velodyne_concat_filter
{
class ConcatFilter
{
public:
  ConcatFilter();
  ConcatFilter(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~ConcatFilter();
  void initialize();

private:
  void callback(const sensor_msgs::PointCloud2ConstPtr &msg, int i);
  void topic_monitor();

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher concat_point_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::string> topics_;
  std::shared_ptr<std::thread> topic_monitor_thread_;
  volatile bool running_;
  std::mutex mutex_;
  double topic_monitor_rate_;
  std::string target_frame_;
  std::vector<ros::Subscriber> subs_;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pointcloud2_vec_;
  std::vector<ros::Time> callback_stamps_;
  ros::Time old_time_;
  ros::Duration query_duration_;
};
}  // namespace velodyne_concat_filter

#endif