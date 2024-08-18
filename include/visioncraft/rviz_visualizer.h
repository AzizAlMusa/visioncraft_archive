#ifndef RVIZ_VISUALIZER_H
#define RVIZ_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <vector>
#include "visioncraft/viewpoint.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace visioncraft {

/**
 * @brief Class for visualizing 3D structures in RViz.
 * 
 * This class provides functionality to publish point clouds, octomaps, and viewpoints
 * for visualization in RViz.
 */
class RVizVisualizer {
public:
    /**
     * @brief Constructor for RVizVisualizer class.
     * 
     * @param nh ROS node handle.
     */
    RVizVisualizer(ros::NodeHandle& nh);

    /**
     * @brief Publish a point cloud to RViz.
     * 
     * @param point_cloud Shared pointer to the Open3D point cloud.
     */
    void publishPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud);

    /**
     * @brief Publish an octomap to RViz.
     * 
     * @param octomap Shared pointer to the colored octomap.
     */
    void publishOctomap(const std::shared_ptr<octomap::ColorOcTree>& octomap);

    /**
     * @brief Publish viewpoints to RViz as arrows.
     * 
     * @param viewpoints Vector of viewpoints.
     */
    void publishViewpoints(const std::vector<Viewpoint>& viewpoints, bool show_frustum = false);

private:
    ros::Publisher point_cloud_pub_; ///< ROS publisher for point clouds.
    ros::Publisher octomap_pub_; ///< ROS publisher for octomaps.
    ros::Publisher marker_pub_; ///< ROS publisher for markers.
    size_t marker_id_; ///< Marker ID for uniquely identifying markers.

    rviz_visual_tools::RvizVisualTools visual_tools_;  // Added
    /**
     * @brief Create a visualization marker.
     * 
     * @return A configured visualization marker.
     */
    visualization_msgs::Marker createMarker();
};

} // namespace visioncraft

#endif // RVIZ_VISUALIZER_H
