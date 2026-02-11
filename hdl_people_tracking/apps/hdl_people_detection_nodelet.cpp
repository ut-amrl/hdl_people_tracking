#include <mutex>
#include <memory>
#include <iostream>
#include <boost/format.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <hdl_people_tracking/msg/cluster_array.hpp>

#include <hdl_people_detection/people_detector.h>
#include <hdl_people_detection/background_subtractor.hpp>

namespace hdl_people_tracking {

/**
 * @brief A node to detect people using a 3D LIDAR
 */
class HdlPeopleDetectionNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlPeopleDetectionNode(const rclcpp::NodeOptions& options) : Node("hdl_people_detection_node", options) {
    initialize_params();

    // publishers
    backsub_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("backsub_points", 5);
    cluster_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_points", 5);
    human_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("human_points", 5);
    detection_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("detection_markers", 5);

    backsub_voxel_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("backsub_voxel_points", 1);
    backsub_voxel_markers_pub = this->create_publisher<visualization_msgs::msg::Marker>("backsub_voxel_marker", 1);

    clusters_pub = this->create_publisher<hdl_people_tracking::msg::ClusterArray>("clusters", 10);

    // subscribers
    globalmap_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/globalmap", 1, std::bind(&HdlPeopleDetectionNode::globalmap_callback, this, std::placeholders::_1));

    if(this->declare_parameter<bool>("static_sensor", false)) {
      static_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 32, std::bind(&HdlPeopleDetectionNode::callback_static, this, std::placeholders::_1));
    } else {
      odom_sub.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, "/odom"));
      points_sub.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, "/velodyne_points"));
      sync.reset(new message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>(*odom_sub, *points_sub, 20));
      sync->registerCallback(std::bind(&HdlPeopleDetectionNode::callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  virtual ~HdlPeopleDetectionNode() {}

private:
  /**
   * @brief initialize_params
   */
  void initialize_params() {
    double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
    backsub_resolution_ = this->declare_parameter<double>("backsub_resolution", 0.2);
    backsub_occupancy_thresh_ = this->declare_parameter<int>("backsub_occupancy_thresh", 2);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(this->get_logger(), "create people detector");
    detector.reset(new hdl_people_detection::PeopleDetector(this));
  }

  /**
   * @brief in case the sensor is fixed
   * @param points_msg
   */
  void callback_static(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Callback is running! Input cloud size: %d", points_msg->width * points_msg->height);
    if(!globalmap) {
      RCLCPP_INFO(this->get_logger(), "constructing globalmap from a points msg");
      globalmap_callback(points_msg);
      RCLCPP_INFO(this->get_logger(), "done");
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    if(cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "cloud is empty!!");
      return;
    }

    // downsampling
    pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*downsampled);
    downsampled->header = cloud->header;
    cloud = downsampled;

    // background subtraction and people detection
    auto filtered = backsub->filter(cloud);
    auto clusters = detector->detect(filtered);

    publish_msgs(points_msg->header.stamp, filtered, clusters);
  }

  /**
   * @brief callback
   * @param odom_msg    sensor pose
   * @param points_msg  point cloud
   */
  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points_msg) {
    if(!globalmap) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "globalmap has not been received!!");
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    if(cloud->empty()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "cloud is empty!!");
      return;
    }

    // downsampling
    pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*downsampled);
    downsampled->header = cloud->header;
    cloud = downsampled;

    // transform #cloud into the globalmap space
    const auto& position = odom_msg->pose.pose.position;
    const auto& orientation = odom_msg->pose.pose.orientation;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 1>(0, 3) = Eigen::Vector3f(position.x, position.y, position.z);
    transform.block<3, 3>(0, 0) = Eigen::Quaternionf(orientation.w, orientation.x, orientation.y, orientation.z).toRotationMatrix();
    pcl::transformPointCloud(*cloud, *cloud, transform);
    cloud->header.frame_id = globalmap->header.frame_id;

    // background subtraction and people detection
    auto filtered = backsub->filter(cloud);
    auto clusters = detector->detect(filtered);

    publish_msgs(points_msg->header.stamp, filtered, clusters);
  }

  void globalmap_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    RCLCPP_INFO(this->get_logger(), "globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    // RCLCPP_INFO(this->get_logger(), "background subtractor constructed");
    // double backsub_resolution;
    // if (!this->has_parameter("backsub_resolution")) {
    //   backsub_resolution = this->declare_parameter<double>("backsub_resolution", 0.2);
    // } else {
    //   this->get_parameter("backsub_resolution", backsub_resolution);
    // }

    // int backsub_occupancy_thresh;
    // if (!this->has_parameter("backsub_occupancy_thresh")) {
    //   backsub_occupancy_thresh = this->declare_parameter<int>("backsub_occupancy_thresh", 2);
    // } else {
    //   this->get_parameter("backsub_occupancy_thresh", backsub_occupancy_thresh);
    // }

    backsub.reset(new hdl_people_detection::BackgroundSubtractor());
    backsub->setVoxelSize(backsub_resolution_, backsub_resolution_, backsub_resolution_);
    backsub->setOccupancyThresh(backsub_occupancy_thresh_);
    backsub->setBackgroundCloud(globalmap);

    backsub_voxel_markers_pub->publish(*backsub->create_voxel_marker());
    
    sensor_msgs::msg::PointCloud2 voxel_ros_cloud;
    pcl::toROSMsg(*backsub->voxels(), voxel_ros_cloud);
    voxel_ros_cloud.header.frame_id = globalmap->header.frame_id;
    voxel_ros_cloud.header.stamp = this->now();
    backsub_voxel_points_pub->publish(voxel_ros_cloud);
  }

private:
  /**
   * @brief publish messages
   * @param stamp
   * @param filtered
   * @param clusters
   */
  void publish_msgs(const rclcpp::Time& stamp, const pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered, const std::vector<hdl_people_detection::Cluster::Ptr>& clusters) const {
    // if(clusters_pub->get_subscription_count() > 0) {
      hdl_people_tracking::msg::ClusterArray clusters_msg;
      clusters_msg.header.frame_id = globalmap->header.frame_id;
      clusters_msg.header.stamp = stamp;

      clusters_msg.clusters.resize(clusters.size());
      for(size_t i=0; i<clusters.size(); i++) {
        auto& cluster_msg = clusters_msg.clusters[i];
        cluster_msg.is_human = clusters[i]->is_human;
        cluster_msg.min_pt.x = clusters[i]->min_pt.x();
        cluster_msg.min_pt.y = clusters[i]->min_pt.y();
        cluster_msg.min_pt.z = clusters[i]->min_pt.z();

        cluster_msg.max_pt.x = clusters[i]->max_pt.x();
        cluster_msg.max_pt.y = clusters[i]->max_pt.y();
        cluster_msg.max_pt.z = clusters[i]->max_pt.z();

        cluster_msg.size.x = clusters[i]->size.x();
        cluster_msg.size.y = clusters[i]->size.y();
        cluster_msg.size.z = clusters[i]->size.z();

        cluster_msg.centroid.x = clusters[i]->centroid.x();
        cluster_msg.centroid.y = clusters[i]->centroid.y();
        cluster_msg.centroid.z = clusters[i]->centroid.z();
      }

      clusters_pub->publish(clusters_msg);
    // }

    // if(backsub_points_pub->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 ros_cloud;
      pcl::toROSMsg(*filtered, ros_cloud);
      ros_cloud.header.stamp = stamp;
      ros_cloud.header.frame_id = globalmap->header.frame_id;
      backsub_points_pub->publish(ros_cloud);
    // }

    // if(cluster_points_pub->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr accum(new pcl::PointCloud<pcl::PointXYZI>());
      for(const auto& cluster : clusters) {
        std::copy(cluster->cloud->begin(), cluster->cloud->end(), std::back_inserter(accum->points));
      }
      accum->width = accum->size();
      accum->height = 1;
      accum->is_dense = false;

      // sensor_msgs::msg::PointCloud2 ros_cloud;
      // pcl::toROSMsg(*accum, ros_cloud);
      ros_cloud.header.stamp = stamp;
      ros_cloud.header.frame_id = globalmap->header.frame_id;
      cluster_points_pub->publish(ros_cloud);
    // }

    // if(human_points_pub->get_subscription_count() > 0) {
      // pcl::PointCloud<pcl::PointXYZI>::Ptr accum(new pcl::PointCloud<pcl::PointXYZI>());
      for(const auto& cluster : clusters) {
        if(cluster->is_human){
          std::copy(cluster->cloud->begin(), cluster->cloud->end(), std::back_inserter(accum->points));
        }
      }
      accum->width = accum->size();
      accum->height = 1;
      accum->is_dense = false;

      // sensor_msgs::msg::PointCloud2 ros_cloud;
      // pcl::toROSMsg(*accum, ros_cloud);
      ros_cloud.header.stamp = stamp;
      ros_cloud.header.frame_id = globalmap->header.frame_id;
      human_points_pub->publish(ros_cloud);
    // }

    // if(detection_markers_pub->get_subscription_count() > 0) {
      detection_markers_pub->publish(create_markers(stamp, clusters));
    // }
  }

  visualization_msgs::msg::MarkerArray create_markers(const rclcpp::Time& stamp, const std::vector<hdl_people_detection::Cluster::Ptr>& clusters) const {
    visualization_msgs::msg::MarkerArray markers;
    
    for(size_t i=0; i<clusters.size(); i++) {
      if(!clusters[i]->is_human) {
        continue;
      }

      visualization_msgs::msg::Marker cluster_marker;
      cluster_marker.header.stamp = stamp;
      cluster_marker.header.frame_id = globalmap->header.frame_id;
      cluster_marker.action = visualization_msgs::msg::Marker::ADD;
      cluster_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      cluster_marker.ns = (boost::format("cluster%d") % i).str();
      cluster_marker.type = visualization_msgs::msg::Marker::CUBE;

      cluster_marker.pose.position.x = clusters[i]->centroid.x();
      cluster_marker.pose.position.y = clusters[i]->centroid.y();
      cluster_marker.pose.position.z = clusters[i]->centroid.z();
      cluster_marker.pose.orientation.w = 1.0;

      cluster_marker.color.r = 0.0;
      cluster_marker.color.g = 0.0;
      cluster_marker.color.b = 1.0;
      cluster_marker.color.a = 0.4;

      cluster_marker.scale.x = clusters[i]->size.x();
      cluster_marker.scale.y = clusters[i]->size.y();
      cluster_marker.scale.z = clusters[i]->size.z();

      markers.markers.push_back(cluster_marker);
    }

    return markers;
  }

private:
  // subscribers
  std::unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> points_sub;
  std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>> sync;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_sub;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr backsub_points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr backsub_voxel_points_pub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_points_pub;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detection_markers_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr backsub_voxel_markers_pub;

  rclcpp::Publisher<hdl_people_tracking::msg::ClusterArray>::SharedPtr clusters_pub;

  // global map
  pcl::PointCloud<PointT>::Ptr globalmap;

  pcl::Filter<PointT>::Ptr downsample_filter;
  std::unique_ptr<hdl_people_detection::BackgroundSubtractor> backsub;
  std::unique_ptr<hdl_people_detection::PeopleDetector> detector;

  double backsub_resolution_;
  int backsub_occupancy_thresh_;

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_people_tracking::HdlPeopleDetectionNode)
