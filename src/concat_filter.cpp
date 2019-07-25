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

#include <velodyne_concat_filter/concat_filter.h>

namespace velodyne_concat_filter
{
ConcatFilter::ConcatFilter() : tf_listener_(tf_buffer_), running_(false) {}
ConcatFilter::ConcatFilter(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : tf_listener_(tf_buffer_), running_(false)
{
  nh_ = nh;
  private_nh_ = private_nh;
}

ConcatFilter::~ConcatFilter()
{
  if (running_) {
    ROS_INFO("shutting thread");
    running_ = false;
    topic_monitor_thread_->join();
    ROS_INFO("thread shutdown");
  }
}

void ConcatFilter::initialize()
{
  if (!private_nh_.getParam("velodyne_topics", topics_)) {
    topics_ = {"/velodyne_front/velodyne_points", "/velodyne_rear/velodyne_points", "/velodyne_right/velodyne_points", "/velodyne_left/velodyne_points", "/velodyne_top/velodyne_points"};
  }
  assert(topics_.size() > 0);
  if (!private_nh_.getParam("topic_monitor_rate", topic_monitor_rate_)) {
    topic_monitor_rate_ = 10;
    query_duration_ = ros::Duration(1.0 / topic_monitor_rate_);
  }
  if (!private_nh_.getParam("target_frame", target_frame_)) {
    target_frame_ = "base_link";
  }
  concat_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("concat_points", 1);

  pointcloud2_vec_.resize(topics_.size());
  callback_stamps_.resize(topics_.size());
  for (int i = 0; i < topics_.size(); i++) {
    pointcloud2_vec_[i].reset(new sensor_msgs::PointCloud2);
    subs_.emplace_back(nh_.subscribe<sensor_msgs::PointCloud2>(topics_[i], 1, boost::bind(&ConcatFilter::callback, this, _1, i)));
  }
  running_ = true;
  topic_monitor_thread_ = std::make_shared<std::thread>(&ConcatFilter::topic_monitor, this);
  topic_monitor_thread_->detach();
}

void ConcatFilter::callback(const sensor_msgs::PointCloud2ConstPtr &msg, int i)
{
  pointcloud2_vec_[i] = msg;
}

void ConcatFilter::topic_monitor()
{
  ros::Rate rate(topic_monitor_rate_);
  while (running_) {
    PointCloudT::Ptr concat_cloud = boost::make_shared<PointCloudT>();
    std::vector<PointCloudT::Ptr> clouds(topics_.size());
    try {
      for (size_t i = 0; i < topics_.size(); i++) {
        clouds[i] = boost::make_shared<PointCloudT>();
        if (!pointcloud2_vec_[i]->data.empty()) {
          sensor_msgs::PointCloud2 pc = *pointcloud2_vec_[i];
          auto diff = ros::Duration(pc.header.stamp.toSec() - old_time_.toSec()) + query_duration_;
          if (diff.toNSec() > 0) {
            std::string source_frame = pc.header.frame_id;
            if (source_frame[0] == '/') {
              source_frame.erase(source_frame.begin());
            }
            const geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(target_frame_, source_frame, ros::Time(0), ros::Duration(0.1));
            sensor_msgs::PointCloud2 transform_cloud;
            pcl_ros::transformPointCloud(tf2::transformToEigen(transformStamped.transform).matrix().cast<float>(), pc, transform_cloud);
            pcl::fromROSMsg(transform_cloud, *clouds[i]);
            *concat_cloud += *clouds[i];
          } else {
            ROS_WARN("drop points frame_id: %s", pointcloud2_vec_[i]->header.frame_id.c_str());
          }
        }
      }
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    if (concat_cloud->points.size() > 0) {
      sensor_msgs::PointCloud2 pubmsg;
      pcl::toROSMsg(*concat_cloud, pubmsg);
      pubmsg.header.stamp = ros::Time::now();
      pubmsg.header.frame_id = target_frame_;
      concat_point_pub_.publish(pubmsg);
    }
    old_time_ = ros::Time::now();
    rate.sleep();
  }
}

}  // namespace velodyne_concat_filter