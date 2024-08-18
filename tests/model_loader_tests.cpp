#include <ros/ros.h>
#include "visioncraft/model_loader.h" // Include your ModelLoader header
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <open3d/Open3D.h>

// Helper function to visualize a point cloud
void visualizePointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& pointCloud, const std::string& title) {
    if (pointCloud && !pointCloud->points_.empty()) {
        open3d::visualization::DrawGeometries({pointCloud}, title, 1600, 900, 50, 50, true);
    } else {
        ROS_ERROR("No point cloud data to visualize.");
    }
}

// Helper function to visualize multiple point clouds with toggle visibility
void visualizePointCloudsWithToggle(const std::shared_ptr<open3d::geometry::PointCloud>& volumetricPointCloud, 
                                    const std::shared_ptr<open3d::geometry::PointCloud>& surfaceShellPointCloud, 
                                    const std::string& title) {
    open3d::visualization::VisualizerWithKeyCallback visualizer;
    visualizer.CreateVisualizerWindow(title, 1600, 900);

    visualizer.AddGeometry(volumetricPointCloud);
    visualizer.AddGeometry(surfaceShellPointCloud);

    bool surfaceShellVisible = true;
    visualizer.RegisterKeyCallback(GLFW_KEY_T, [&](open3d::visualization::Visualizer* vis) {
        surfaceShellVisible = !surfaceShellVisible;
        surfaceShellPointCloud->Clear();
        if (surfaceShellVisible) {
            // Re-add the points to the surfaceShellPointCloud
            // Assuming the points and colors were previously stored
            for (const auto& point : surfaceShellPointCloud->points_) {
                surfaceShellPointCloud->points_.push_back(point);
            }
            for (const auto& color : surfaceShellPointCloud->colors_) {
                surfaceShellPointCloud->colors_.push_back(color);
            }
        }
        vis->UpdateGeometry();
        return true;
    });

    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}

// Helper function to visualize a voxel grid
void visualizeVoxelGrid(const std::shared_ptr<open3d::geometry::VoxelGrid>& voxelGrid, const std::string& title) {
    if (voxelGrid && voxelGrid->voxels_.size() > 0) {
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow(title, 1600, 900);
        visualizer.AddGeometry(voxelGrid);
        visualizer.GetRenderOption().ToggleLightOn();
        visualizer.GetRenderOption().mesh_shade_option_ = open3d::visualization::RenderOption::MeshShadeOption::FlatShade;
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();
    } else {
        ROS_ERROR("No voxel grid data to visualize.");
    }
}

// Helper function to publish an octomap
void publishOctomap(const std::shared_ptr<octomap::ColorOcTree>& octomap, const ros::Publisher& publisher, const std::string& frame_id) {
    if (octomap) {
        octomap_msgs::Octomap octomap_msg;
        octomap_msgs::fullMapToMsg(*octomap, octomap_msg);
        octomap_msg.header.frame_id = "map";
        octomap_msg.header.stamp = ros::Time::now();
        publisher.publish(octomap_msg);
        ROS_INFO("%s published to topic.", frame_id.c_str());
    } else {
        ROS_ERROR("Failed to publish %s.", frame_id.c_str());
    }
}

// Function to convert an OctoMap to an Open3D point cloud
std::shared_ptr<open3d::geometry::PointCloud> ConvertOctomapToPointCloud(const std::shared_ptr<octomap::ColorOcTree>& octomap, const open3d::utility::optional<Eigen::Vector3d>& color = {}) {
    auto pointCloud = std::make_shared<open3d::geometry::PointCloud>();
    for (auto it = octomap->begin_leafs(); it != octomap->end_leafs(); ++it) {
        if (it->getOccupancy() > 0.5) {
            pointCloud->points_.emplace_back(it.getX(), it.getY(), it.getZ());
            if (color.has_value()) {
                pointCloud->colors_.emplace_back(color.value());
            } else {
                pointCloud->colors_.emplace_back(it->getColor().r / 255.0, it->getColor().g / 255.0, it->getColor().b / 255.0);
            }
        }
    }
    return pointCloud;
}

// Function to load the mesh
bool loadMesh(visioncraft::ModelLoader& modelLoader, const std::string& filePath) {
    if (!modelLoader.loadMesh(filePath)) {
        ROS_ERROR("Failed to load mesh.");
        return false;
    }
    ROS_INFO("Mesh loaded successfully.");
    return true;
}

// Function to generate the point cloud
bool generatePointCloud(visioncraft::ModelLoader& modelLoader) {
    if (!modelLoader.generatePointCloud(10000)) {
        ROS_ERROR("Failed to generate point cloud.");
        return false;
    }
    ROS_INFO("Point cloud generated successfully.");
    return true;
}

// Function to generate the voxel grid
bool generateVoxelGrid(visioncraft::ModelLoader& modelLoader) {
    if (!modelLoader.generateVoxelGrid(0.0)) {
        ROS_ERROR("Failed to generate voxel grid.");
        return false;
    }
    ROS_INFO("Voxel grid generated successfully.");
    return true;
}

// Function to generate the exploration map
bool generateExplorationMap(visioncraft::ModelLoader& modelLoader) {
    octomap::point3d min_bound = modelLoader.getMinBound();
    octomap::point3d max_bound = modelLoader.getMaxBound();
    if (!modelLoader.generateExplorationMap(32, min_bound, max_bound)) {
        ROS_ERROR("Failed to generate exploration map.");
        return false;
    }
    ROS_INFO("Exploration map generated successfully.");
    return true;
}

// Function to generate the octomap
bool generateOctomap(visioncraft::ModelLoader& modelLoader) {
    double resolution = modelLoader.getExplorationMapResolution();
    if (!modelLoader.generateOctoMap(resolution)) {
        ROS_ERROR("Failed to generate octomap.");
        return false;
    }
    ROS_INFO("Octomap generated successfully.");
    return true;
}

// Function to generate the volumetric octomap
bool generateVolumetricOctomap(visioncraft::ModelLoader& modelLoader) {
    double resolution = modelLoader.getAverageSpacing();
    if (!modelLoader.generateVolumetricOctoMap(resolution)) {
        ROS_ERROR("Failed to generate volumetric octomap.");
        return false;
    }
    ROS_INFO("Volumetric octomap generated successfully.");
    return true;
}

// Function to generate the surface shell octomap
bool generateSurfaceShellOctomap(visioncraft::ModelLoader& modelLoader) {
    if (!modelLoader.generateSurfaceShellOctomap()) {
        ROS_ERROR("Failed to generate surface shell octomap.");
        return false;
    }
    ROS_INFO("Surface shell octomap generated successfully.");
    return true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "model_loader_tests");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create publishers for the octomap, volumetric octomap, exploration map, and surface shell octomap
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_topic", 1, true);
    ros::Publisher volumetric_octomap_pub = nh.advertise<octomap_msgs::Octomap>("volumetric_octomap_topic", 1, true);
    ros::Publisher exploration_map_pub = nh.advertise<octomap_msgs::Octomap>("exploration_map_topic", 1, true);
    ros::Publisher surface_shell_octomap_pub = nh.advertise<octomap_msgs::Octomap>("surface_shell_octomap_topic", 1, true);

    // Check if the file path argument is provided
    if (argc != 2) {
        ROS_ERROR("Usage: model_loader_tests <file_path>");
        return 1;
    }

    // Get the file path from the command-line argument
    std::string filePath = argv[1];
    ROS_INFO("File path: %s", filePath.c_str());

    // Create an instance of the ModelLoader
    visioncraft::ModelLoader modelLoader;

    // Load the mesh
    if (!loadMesh(modelLoader, filePath)) return 1;

    // Initialize the raycasting scene
    if (!modelLoader.initializeRaycastingScene()) return 1;

    // Generate the point cloud
    if (!generatePointCloud(modelLoader)) return 1;

    // Visualize the point cloud
    std::shared_ptr<open3d::geometry::PointCloud> pointCloud = modelLoader.getPointCloud();
    visualizePointCloud(pointCloud, "Point Cloud with Normals");

    // Generate the voxel grid
    if (!generateVoxelGrid(modelLoader)) return 1;

    // Visualize the voxel grid
    std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid = modelLoader.getVoxelGrid();
    visualizeVoxelGrid(voxelGrid, "Voxel Grid Visualization");

    // Generate the exploration map
    if (!generateExplorationMap(modelLoader)) return 1;

    // Generate the octomap
    if (!generateOctomap(modelLoader)) return 1;

    // Generate the volumetric point cloud
    if (modelLoader.generateVolumetricPointCloud()) {
        std::shared_ptr<open3d::geometry::PointCloud> volumetricPointCloud = modelLoader.getVolumetricPointCloud();
        visualizePointCloud(volumetricPointCloud, "Volumetric Point Cloud");

        // Generate the volumetric octomap
        if (!generateVolumetricOctomap(modelLoader)) return 1;
    } else {
        ROS_ERROR("Failed to create volumetric point cloud.");
        return 1;
    }

    // Generate the surface shell octomap
    if (!generateSurfaceShellOctomap(modelLoader)) return 1;

    // Get the octomaps
    std::shared_ptr<octomap::ColorOcTree> octomap = modelLoader.getOctomap();
    std::shared_ptr<octomap::ColorOcTree> explorationMap = modelLoader.getExplorationMap();
    std::shared_ptr<octomap::ColorOcTree> volumetricOctomap = modelLoader.getVolumetricOctomap();
    std::shared_ptr<octomap::ColorOcTree> surfaceShellOctomap = modelLoader.getSurfaceShellOctomap();

    // Print bounding box of octomap
    double min_x, min_y, min_z, max_x, max_y, max_z;
    double octomap_resolution = octomap->getResolution();
    auto tree_depth = octomap->getTreeDepth();
    octomap->getMetricMin(min_x, min_y, min_z);
    octomap->getMetricMax(max_x, max_y, max_z);
    octomap::point3d bounding_box = octomap->getBBXBounds();
    ROS_INFO("Octomap Bounding Box:");
    ROS_INFO("Min: (%f, %f, %f)", min_x, min_y, min_z);
    ROS_INFO("Max: (%f, %f, %f)", max_x, max_y, max_z);
    ROS_INFO("Resolution: %f", octomap_resolution);
    ROS_INFO("Tree Depth: %d", tree_depth);
    ROS_INFO("Bounding Box: (%f, %f, %f)", bounding_box.x(), bounding_box.y(), bounding_box.z());


    // Convert OctoMaps to Open3D Point Clouds for Visualization with specified colors and opacity
    auto volumetricOctomapPointCloud = ConvertOctomapToPointCloud(volumetricOctomap, Eigen::Vector3d(1.0, 0.0, 0.0)); // Red color
    auto surfaceShellOctomapPointCloud = ConvertOctomapToPointCloud(surfaceShellOctomap, Eigen::Vector3d(0.0, 0.0, 1.0)); // Blue color

    // Apply 50% opacity to both point clouds
    for (auto& color : volumetricOctomapPointCloud->colors_) {
        color *= 0.5;
    }
    for (auto& color : surfaceShellOctomapPointCloud->colors_) {
        color *= 0.5;
    }

    // Visualize both point clouds together with toggle visibility
    visualizePointCloudsWithToggle(volumetricOctomapPointCloud, surfaceShellOctomapPointCloud, "Volumetric and Surface Shell OctoMaps");

    // Set the publishing rate (e.g., 1 Hz)
    ros::Rate rate(1.0); // 1 Hz

    while (ros::ok()) {
        // Publish the octomap
        publishOctomap(octomap, octomap_pub, "octomap");

        // Publish the exploration map
        publishOctomap(explorationMap, exploration_map_pub, "exploration_map");

        // Publish the volumetric octomap
        publishOctomap(volumetricOctomap, volumetric_octomap_pub, "volumetric_octomap");

        // Publish the surface shell octomap
        publishOctomap(surfaceShellOctomap, surface_shell_octomap_pub, "surface_shell_octomap");

        // Spin once to handle callbacks
        ros::spinOnce();

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    return 0;
}
