#include <ros/ros.h>
#include "visioncraft/model_loader.h" // Include your ModelLoader header
#include "visioncraft/rviz_visualizer.h" // Include your RVizVisualizer header
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <open3d/Open3D.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "rviz_visualizer_tests");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Check if the file path argument is provided
    if (argc != 2) {
        ROS_ERROR("Usage: rviz_visualizer_tests <file_path>");
        return 1;
    }

    // Get the file path from the command-line argument
    std::string filePath = argv[1];
    ROS_INFO("File path: %s", filePath.c_str());

    // Create an instance of the ModelLoader
    visioncraft::ModelLoader modelLoader;

    // Load the mesh
    if (!modelLoader.loadMesh(filePath)) return 1;

    // Initialize the raycasting scene
    if (!modelLoader.initializeRaycastingScene()) return 1;

    // Generate the point cloud
    if (!modelLoader.generatePointCloud(10000)) return 1;

    // Generate the voxel grid
    if (!modelLoader.generateVoxelGrid(0.0)) return 1;

    // Generate the exploration map
    octomap::point3d min_bound = modelLoader.getMinBound();
    octomap::point3d max_bound = modelLoader.getMaxBound();
    if (!modelLoader.generateExplorationMap(32, min_bound, max_bound)) return 1;

    // Generate the octomap
    double resolution = modelLoader.getExplorationMapResolution();
    if (!modelLoader.generateOctoMap(resolution)) return 1;

    // Generate the volumetric point cloud
    if (!modelLoader.generateVolumetricPointCloud()) return 1;

    // Generate the volumetric octomap
    double volumetric_resolution = modelLoader.getAverageSpacing();
    if (!modelLoader.generateVolumetricOctoMap(volumetric_resolution)) return 1;

    // Generate the surface shell octomap
    if (!modelLoader.generateSurfaceShellOctomap()) return 1;

    // Get the data for visualization
    std::shared_ptr<open3d::geometry::PointCloud> pointCloud = modelLoader.getPointCloud();
    std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid = modelLoader.getVoxelGrid();
    std::shared_ptr<octomap::ColorOcTree> octomap = modelLoader.getOctomap();
    std::shared_ptr<octomap::ColorOcTree> volumetricOctomap = modelLoader.getVolumetricOctomap();
    std::shared_ptr<octomap::ColorOcTree> surfaceShellOctomap = modelLoader.getSurfaceShellOctomap();
    std::shared_ptr<octomap::ColorOcTree> explorationMap = modelLoader.getExplorationMap();

    // Create an instance of the RVizVisualizer
    visioncraft::RVizVisualizer visualizer(nh);

    // Set the publishing rate (e.g., 1 Hz)
    ros::Rate rate(1.0); // 1 Hz

    // Create a viewpoint using lookAt
    Eigen::Vector3d position(1, 0, 0);
    Eigen::Vector3d position2(0, 1, 0);
    Eigen::Vector3d position3(0, 0, 1);
    Eigen::Vector3d position4(1, 1, 1);
    Eigen::Vector3d position5(1, -1, 1);
    Eigen::Vector3d lookAt(0, 0, 0);
    Eigen::Vector3d up(0.0, 0.0, -1);   // Define the up direction as negative Y to point down
   
    
    while (ros::ok()) {
        // Publish the point cloud
        visualizer.publishPointCloud(pointCloud);

        // Publish the octomap
        visualizer.publishOctomap(octomap);

        // Generate and publish the viewpoints as axis frames (example viewpoints)
        std::vector<visioncraft::Viewpoint> viewpoints = {
            visioncraft::Viewpoint(position, lookAt, up, 0.1, 0.5),
            visioncraft::Viewpoint(position2, lookAt, up, 0.1, 0.5),
            visioncraft::Viewpoint(position3, lookAt, up, 0.1, 0.5),
            visioncraft::Viewpoint(position4, lookAt, up, 0.1, 0.5),
            visioncraft::Viewpoint(position5, lookAt, up, 0.1, 0.5)
        };


        visualizer.publishViewpoints(viewpoints, true);

        // Spin once to handle callbacks
        ros::spinOnce();

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    return 0;
}
