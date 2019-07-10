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
ConcatFilter::ConcatFilter() : private_nh_("~"), tf_listener_(tf_buffer_), running_(false) {}
ConcatFilter::ConcatFilter(ros::NodeHandle &nh) : tf_listener_(tf_buffer_), running_(false)
{
  nh_ = nh;
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
  assert(2 <= topics_.size() && topics_.size() <= 5);
  int synchronizer_queue_size;
  if (!private_nh_.getParam("synchronizer_queue_size", synchronizer_queue_size)) {
    synchronizer_queue_size = 10;
  }
  if (!private_nh_.getParam("wait_for_message_timeout", wait_for_message_timeout_)) {
    wait_for_message_timeout_ = 0.3;
  }
  if (!private_nh_.getParam("topic_monitor_rate", topic_monitor_rate_)) {
    topic_monitor_rate_ = 1;
  }
  if (!private_nh_.getParam("target_frame", target_frame_)) {
    target_frame_ = "base_link";
  }
  current_topics_ = topics_;
  concat_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("concat_points", 1);
  for (size_t i = 0; i < 5; i++) {
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub;
    if (i < topics_.size()){
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, topics_[i], 1);
    } else {
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, topics_[0], 1);
    }
    sub_.emplace_back(sub);
  }
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(SyncPolicyT(synchronizer_queue_size), *sub_[0], *sub_[1], *sub_[2], *sub_[3], *sub_[4]);
  sync_->registerCallback(boost::bind(&ConcatFilter::callback, this, _1, _2, _3, _4, _5));
  running_ = true;
  topic_monitor_thread_ = std::make_shared<std::thread>(&ConcatFilter::topic_monitor, this);
  topic_monitor_thread_->detach();
}

void ConcatFilter::callback(const sensor_msgs::PointCloud2ConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2, const sensor_msgs::PointCloud2ConstPtr &msg3, const sensor_msgs::PointCloud2ConstPtr &msg4, const sensor_msgs::PointCloud2ConstPtr &msg5)
{
  size_t current_topics_size = current_topics_.size();
  std::vector<sensor_msgs::PointCloud2ConstPtr> msgs{msg1, msg2, msg3, msg4, msg5};
  std::vector<PointCloudT::Ptr> clouds(current_topics_size);
  PointCloudT::Ptr concat_cloud = boost::make_shared<PointCloudT>();
  try {
    for (size_t i = 0; i < current_topics_size; i++) {
      std::string source_frame = msgs[i]->header.frame_id;
      if (source_frame[0] == '/')
      {
        source_frame.erase(source_frame.begin());
      }
      const geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(target_frame_, source_frame, ros::Time(0), ros::Duration(0.1));
      sensor_msgs::PointCloud2 transform_cloud;
      pcl_ros::transformPointCloud(tf2::transformToEigen(transformStamped.transform).matrix().cast<float>(), *msgs[i], transform_cloud);
      clouds[i] = boost::make_shared<PointCloudT>();
      pcl::fromROSMsg(transform_cloud, *clouds[i]);
    }
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  for (size_t i = 0; i < clouds.size(); i++) {
    *concat_cloud += *clouds[i];
  }
  sensor_msgs::PointCloud2 pubmsg;
  pcl::toROSMsg(*concat_cloud, pubmsg);
  pubmsg.header.stamp = ros::Time::now();
  pubmsg.header.frame_id = target_frame_;
  concat_point_pub_.publish(pubmsg);
}

void ConcatFilter::topic_monitor()
{
  ros::Rate rate(topic_monitor_rate_);
  while (running_) {
    std::vector<std::string> available_topics;
    for (auto topic : topics_) {
      auto available_topic = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, ros::Duration(wait_for_message_timeout_));
      if (!available_topic) {
        ROS_WARN("%s is not available", topic.c_str());
      } else {
        available_topics.emplace_back(topic);
      }
    }

    auto current_topics_tmp = current_topics_;
    std::sort(current_topics_tmp.begin(), current_topics_tmp.end());
    std::sort(available_topics.begin(), available_topics.end());
    if (current_topics_tmp != available_topics) {
      sync_.reset();
      current_topics_ = available_topics;
      if (!current_topics_.empty()) {
        for (size_t i = 0; i < sub_.size(); i++) {
          if (i < current_topics_.size()) {
            sub_[i].reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, current_topics_[i], 1));
          } else {
            sub_[i].reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, current_topics_[0], 1));
          }
        }
        sync_.reset(new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *sub_[0], *sub_[1], *sub_[2], *sub_[3], *sub_[4]));
        sync_->registerCallback(boost::bind(&ConcatFilter::callback, this, _1, _2, _3, _4, _5));
        ROS_WARN("Switch callback");
      }
    }
    rate.sleep();
  }
}
}