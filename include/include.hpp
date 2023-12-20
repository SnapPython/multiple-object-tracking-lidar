#include <algorithm>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <limits>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "kf_tracker/CKalmanFilter.h"
#include "kf_tracker/featureDetection.h"
#include <opencv2/opencv.hpp>


namespace multiple_object_tracking
{
class MultipleObjectTracking : public rclcpp::Node
{
public:
  explicit MultipleObjectTracking(const rclcpp::NodeOptions & options);

  ~MultipleObjectTracking() override;

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_marker();
    void publish_marker_array();
    void publish_pointcloud();
    void publish_pointcloud2();
    void publish_pointcloud3();
    void publish_pointcloud4();
    void publish_pointcloud5();
    void publish_pointcloud6();
};
}