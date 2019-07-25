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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <velodyne_concat_filter/concat_filter.h>

namespace velodyne_concat_filter
{
class ConcatFilterNodelet : public nodelet::Nodelet
{
public:
  ConcatFilterNodelet() {}
  virtual void onInit()
  {
    cf_ = std::make_shared<ConcatFilter>(getNodeHandle(), getPrivateNodeHandle());
    cf_->initialize();
  }

private:
  std::shared_ptr<ConcatFilter> cf_;
};
}  // namespace velodyne_concat_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(velodyne_concat_filter::ConcatFilterNodelet, nodelet::Nodelet);