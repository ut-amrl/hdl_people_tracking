#include <hdl_people_detection/people_detector.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <hdl_people_detection/cluster_detector.hpp>
#include <hdl_people_detection/marcel_people_detector.hpp>

namespace hdl_people_detection {

PeopleDetector::PeopleDetector(rclcpp::Node* node) {
  min_pts = node->declare_parameter<int>("cluster_min_pts", 10);
  max_pts = node->declare_parameter<int>("cluster_max_pts", 8192);
  min_size.x() = node->declare_parameter<double>("cluster_min_size_x", 0.2);
  min_size.y() = node->declare_parameter<double>("cluster_min_size_y", 0.2);
  min_size.z() = node->declare_parameter<double>("cluster_min_size_z", 0.3);
  max_size.x() = node->declare_parameter<double>("cluster_max_size_x", 1.0);
  max_size.y() = node->declare_parameter<double>("cluster_max_size_y", 1.0);
  max_size.z() = node->declare_parameter<double>("cluster_max_size_z", 2.0);

  if(node->declare_parameter<bool>("enable_classification", true)) {
    try {
      std::string package_path = ament_index_cpp::get_package_share_directory("hdl_people_tracking");
      // Note: In ROS 1 getPath returned the source directory for devel/source, but share/pkg for install.
      // In ROS 2 get_package_share_directory returns share/pkg.
      // I need to ensure the data files are installed to share/pkg/data.
      // The CMakeLists.txt I wrote earlier did NOT install data folder. I missed that.
      // I will need to update CMakeLists.txt to install the data folder.
      classifier.reset(new KidonoHumanClassifier(package_path + "/data/boost_kidono.model", package_path + "/data/boost_kidono.scale"));
    } catch (std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "failed to find package path: %s", e.what());
    }
  }
}

PeopleDetector::~PeopleDetector() {

}

std::vector<Cluster::Ptr> PeopleDetector::detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
  MarcelPeopleDetector marcel(min_pts, max_pts, min_size, max_size);
  auto clusters = marcel.detect(cloud);

  for(auto& cluster : clusters) {
    cluster->is_human = !classifier || classifier->predict(cluster->cloud);
  }

  return clusters;
}

}
