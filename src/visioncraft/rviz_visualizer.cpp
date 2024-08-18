#include "rviz_visualizer.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

namespace visioncraft {

RVizVisualizer::RVizVisualizer(ros::NodeHandle& nh)
    : marker_id_(0),
      visual_tools_("map", "/rviz_visual_markers") { // Initialize visual_tools_
    visual_tools_.loadMarkerPub(); // Load marker publisher

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1);
}

visualization_msgs::Marker RVizVisualizer::createMarker() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "rviz_visualizer";
    marker.id = marker_id_++;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    return marker;
}

void RVizVisualizer::publishPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud) {
    sensor_msgs::PointCloud2 ros_point_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;

    for (const auto& point : point_cloud->points_) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        pcl_point_cloud.push_back(pcl_point);
    }

    pcl::toROSMsg(pcl_point_cloud, ros_point_cloud);
    ros_point_cloud.header.frame_id = "map";
    point_cloud_pub_.publish(ros_point_cloud);
}

void RVizVisualizer::publishOctomap(const std::shared_ptr<octomap::ColorOcTree>& octomap) {
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::fullMapToMsg(*octomap, octomap_msg);
    octomap_msg.header.frame_id = "map";
    octomap_pub_.publish(octomap_msg);
}

void RVizVisualizer::publishViewpoints(const std::vector<Viewpoint>& viewpoints, bool show_frustum) {
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;

    for (const auto& viewpoint : viewpoints) {
        // Publish axis for each viewpoint using visual_tools_ (No issues here)
        Eigen::Quaterniond quaternion = viewpoint.getOrientationQuaternion().normalized();
        geometry_msgs::Pose pose;
        pose.position.x = viewpoint.getPosition().x();
        pose.position.y = viewpoint.getPosition().y();
        pose.position.z = viewpoint.getPosition().z();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        visual_tools_.publishAxisLabeled(pose, "Viewpoint");

        // Add frustum visualization if show_frustum is true
        if (show_frustum) {
            visualization_msgs::Marker frustum_marker;
            frustum_marker.header.frame_id = "map";
            frustum_marker.header.stamp = ros::Time::now();
            frustum_marker.ns = "camera_frustums";
            frustum_marker.id = id++;
            frustum_marker.type = visualization_msgs::Marker::LINE_LIST;
            frustum_marker.action = visualization_msgs::Marker::ADD;
            frustum_marker.scale.x = 0.01;  // Line width
            frustum_marker.color.r = 1.0f;
            frustum_marker.color.g = 1.0f;
            frustum_marker.color.b = 1.0f;
            frustum_marker.color.a = 1.0;

            // Get frustum corners
            std::vector<Eigen::Vector3d> frustum_corners = viewpoint.getFrustumCorners();
            std::vector<geometry_msgs::Point> frustum_points(8);

            // Convert Eigen::Vector3d to geometry_msgs::Point
            for (int i = 0; i < 8; ++i) {
                frustum_points[i].x = frustum_corners[i].x();
                frustum_points[i].y = frustum_corners[i].y();
                frustum_points[i].z = frustum_corners[i].z();
            }

            // Connect the corners to form the frustum
            for (int i = 0; i < 4; ++i) {
                frustum_marker.points.push_back(frustum_points[i]);
                frustum_marker.points.push_back(frustum_points[(i + 1) % 4]);  // Near plane
                frustum_marker.points.push_back(frustum_points[i + 4]);
                frustum_marker.points.push_back(frustum_points[4 + (i + 1) % 4]);  // Far plane
                frustum_marker.points.push_back(frustum_points[i]);
                frustum_marker.points.push_back(frustum_points[i + 4]);  // Connecting lines
            }

            // Directly set the frustum marker's pose to identity
            frustum_marker.pose.orientation.w = 1.0;
            frustum_marker.pose.orientation.x = 0.0;
            frustum_marker.pose.orientation.y = 0.0;
            frustum_marker.pose.orientation.z = 0.0;

            marker_array.markers.push_back(frustum_marker);
        }
    }

    // Publish the new markers
    marker_pub_.publish(marker_array);

    std::cout << "Published frustum markers on /viewpoints topic." << std::endl;
}

} // namespace visioncraft
