# velodyne_concat_filter

## Usage
```
roslaunch velodyne_concat_filter velodyne_concat_filter_node.launch
```
* If you want to use Nodelet
```
roslaunch velodyne_concat_filter velodyne_concat_filter_nodelet.launch
```

## ROS Parameters
|paramter name|default value|description|
|---|---|---|
|velodyne_topics|/velodyne_front/velodyne_points, /velodyne_top/velodyne_points|velodyne topics to concat.|
|topic_monitor_rate|10 (Hz)|Rate for monitor velodyne topic. This parameter is ros::Rate argument.|
|target_frame|base_link|Target frame for lookuptTransform.|
