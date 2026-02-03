#include <mutex>
#include <memory>
#include <iostream>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/filters/filter.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <hdl_people_tracking/msg/track_array.hpp>
#include <hdl_people_tracking/msg/cluster_array.hpp>

#include <kkl/cvk/cvutils.hpp>
#include <hdl_people_tracking/people_tracker.hpp>

namespace hdl_people_tracking {

class HdlPeopleTrackingNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlPeopleTrackingNode(const rclcpp::NodeOptions& options) : Node("hdl_people_tracking_node", options) {
    tracker.reset(new PeopleTracker(this));
    // kkl::cvk::create_color_palette might need checking. 
    // Assuming kkl is available as it was for classifier.
    color_palette = cvk::create_color_palette(16);

    tracks_pub = this->create_publisher<hdl_people_tracking::msg::TrackArray>("tracks", 10);
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
    clusters_sub = this->create_subscription<hdl_people_tracking::msg::ClusterArray>(
      "clusters", 1, std::bind(&HdlPeopleTrackingNode::callback, this, std::placeholders::_1));
  }

  virtual ~HdlPeopleTrackingNode() {}

private:

  void callback(const hdl_people_tracking::msg::ClusterArray::ConstSharedPtr clusters_msg) {
    // Filter non-human clusters
    std::vector<hdl_people_tracking::msg::Cluster> clusters;
    clusters.reserve(clusters_msg->clusters.size());
    for(const auto& cluster : clusters_msg->clusters) {
      if(cluster.is_human) {
        clusters.push_back(cluster);
      }
    }

    // update people tracker
    tracker->predict(clusters_msg->header.stamp);
    tracker->correct(clusters_msg->header.stamp, clusters);

    // publish tracks msg
    if(tracks_pub->get_subscription_count() > 0) {
      tracks_pub->publish(create_tracks_msg(clusters_msg->header));
    }

    // publish rviz markers
    if(marker_pub->get_subscription_count() > 0) {
      marker_pub->publish(create_tracked_people_marker(clusters_msg->header));
    }
  }

  hdl_people_tracking::msg::TrackArray create_tracks_msg(const std_msgs::msg::Header& header) const {
    hdl_people_tracking::msg::TrackArray tracks_msg;
    tracks_msg.header = header;

    tracks_msg.tracks.resize(tracker->people.size());
    for(int i=0; i<tracker->people.size(); i++) {
      const auto& track = tracker->people[i];
      auto& track_msg = tracks_msg.tracks[i];

      track_msg.id = track->id();
      // track->age returns rclcpp::Duration, toSec() -> seconds()
      track_msg.age = track->age(header.stamp).seconds();
      track_msg.pos.x = track->position().x();
      track_msg.pos.y = track->position().y();
      track_msg.pos.z = track->position().z();
      track_msg.vel.x = track->velocity().x();
      track_msg.vel.y = track->velocity().y();
      track_msg.vel.z = track->velocity().z();

      Eigen::Matrix3d pos_cov = track->positionCov();
      for(int k=0; k<3; k++) {
        for(int j=0; j<3; j++) {
          track_msg.pos_cov[k*3 + j] = pos_cov(k, j);
        }
      }

      Eigen::Matrix3d vel_cov = track->velocityCov();
      for(int k=0; k<3; k++) {
        for(int j=0; j<3; j++) {
          track_msg.vel_cov[k*3 + j] = vel_cov(k, j);
        }
      }

      const hdl_people_tracking::msg::Cluster* associated = boost::any_cast<hdl_people_tracking::msg::Cluster>(&track->lastAssociated());
      if(associated) {
        track_msg.associated.resize(1);
        track_msg.associated[0] = (*associated);
      }
    }

    return tracks_msg;
  }

  visualization_msgs::msg::MarkerArray create_tracked_people_marker(const std_msgs::msg::Header& header) const {
    visualization_msgs::msg::MarkerArray markers;

    if (tracker->people.empty()) {
        return markers;
    }

    visualization_msgs::msg::Marker boxes;
    boxes.header = header;
    boxes.action = visualization_msgs::msg::Marker::ADD;
    boxes.lifetime = rclcpp::Duration::from_seconds(1.0);

    boxes.ns = "boxes";
    boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    boxes.colors.reserve(tracker->people.size());
    boxes.points.reserve(tracker->people.size());

    boxes.pose.position.z = 0.0f;
    boxes.pose.orientation.w = 1.0f;

    boxes.scale.x = 0.5;
    boxes.scale.y = 0.5;
    boxes.scale.z = 1.2;

    for(int i=0; i<tracker->people.size(); i++) {
      const auto& person = tracker->people[i];
      const auto& color = color_palette[person->id() % color_palette.size()];

      if(person->correctionCount() < 5) {
        continue;
      }

      std_msgs::msg::ColorRGBA rgba;
      rgba.r = color[2] / 255.0;
      rgba.g = color[1] / 255.0;
      rgba.b = color[0] / 255.0;
      rgba.a = 0.6f;
      boxes.colors.push_back(rgba);

      geometry_msgs::msg::Point point;
      point.x = person->position().x();
      point.y = person->position().y();
      point.z = person->position().z();
      boxes.points.push_back(point);

      visualization_msgs::msg::Marker text;
      text.header = header;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.lifetime = rclcpp::Duration::from_seconds(1.0);

      text.ns = (boost::format("text%d") % person->id()).str();
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.scale.z = 0.5;

      text.pose.position = point;
      text.pose.position.z += 0.7;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
      text.text = (boost::format("id:%d") % person->id()).str();

      markers.markers.push_back(text);
    }
    
    if (!boxes.points.empty()) {
       markers.markers.insert(markers.markers.begin(), boxes);
    }

    return markers;
  }

private:
  rclcpp::Publisher<hdl_people_tracking::msg::TrackArray>::SharedPtr tracks_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  rclcpp::Subscription<hdl_people_tracking::msg::ClusterArray>::SharedPtr clusters_sub;

  boost::circular_buffer<cv::Scalar> color_palette;

  std::unique_ptr<PeopleTracker> tracker;
};

}


RCLCPP_COMPONENTS_REGISTER_NODE(hdl_people_tracking::HdlPeopleTrackingNode)
