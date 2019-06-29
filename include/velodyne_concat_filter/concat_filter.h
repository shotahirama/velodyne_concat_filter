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
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>

namespace velodyne_concat_filter
{
class ConcatFilter : public nodelet::Nodelet
{
public:
  ConcatFilter();
  ~ConcatFilter();
  virtual void onInit();

private:
  void callback(const sensor_msgs::PointCloud2ConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2, const sensor_msgs::PointCloud2ConstPtr &msg3, const sensor_msgs::PointCloud2ConstPtr &msg4, const sensor_msgs::PointCloud2ConstPtr &msg5);
  void topic_monitor();

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicyT;
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;

  ros::NodeHandle nh_;
  ros::Publisher concat_point_pub_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>> sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> sync_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::string> topics_, current_topics_;
  std::shared_ptr<std::thread> topic_monitor_thread_;
  volatile bool running_;
  std::mutex mutex_;
  double wait_for_message_timeout_;
  double topic_monitor_rate_;
  std::string target_frame_;
};
}

#endif