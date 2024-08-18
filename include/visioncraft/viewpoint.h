#ifndef VISIONCRAFT_VIEWPOINT_H
#define VISIONCRAFT_VIEWPOINT_H

#include <Eigen/Dense>
#include <memory>
#include <open3d/Open3D.h>
#include <octomap/ColorOcTree.h>

namespace visioncraft {

/**
 * @brief Class representing a viewpoint for depth camera positioning and orientation.
 * 
 * This class provides functionality to define and manipulate the position and orientation
 * of a depth camera viewpoint in a 3D space. It also includes camera properties like near
 * and far planes, resolution, and downsampling for simulation purposes.
 */
class Viewpoint {
public:
    /**
     * @brief Default constructor for Viewpoint class.
     * 
     * Initializes the viewpoint with default position, orientation, and camera properties.
     */
    Viewpoint();

    /**
     * @brief Constructor with position, orientation, and camera properties.
     * 
     * @param position The position of the viewpoint as an Eigen::Vector3d.
     * @param orientation The orientation of the viewpoint as a rotation matrix (Eigen::Matrix3d).
     * @param near The near plane distance of the camera frustum in millimeters (default: 350.0).
     * @param far The far plane distance of the camera frustum in millimeters (default: 900.0).
     * @param resolution_width The width resolution of the camera (default: 2448).
     * @param resolution_height The height resolution of the camera (default: 2048).
     * @param hfov The horizontal field of view of the camera in degrees (default: 44.8).
     * @param vfov The vertical field of view of the camera in degrees (default: 42.6).
     */
    Viewpoint(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, 
              double near = 350.0, double far = 900.0, 
              int resolution_width = 2448, int resolution_height = 2048,
              double hfov = 44.8, double vfov = 42.6);

    /**
     * @brief Constructor with position and lookAt point.
     * 
     * @param position The position of the viewpoint as an Eigen::Vector3d.
     * @param lookAt The target point the viewpoint is oriented towards.
     * @param up The up direction for the viewpoint (default: Eigen::Vector3d::UnitY()).
     * @param near The near plane distance of the camera frustum in millimeters (default: 350.0).
     * @param far The far plane distance of the camera frustum in millimeters (default: 900.0).
     * @param resolution_width The width resolution of the camera (default: 2448).
     * @param resolution_height The height resolution of the camera (default: 2048).
     * @param hfov The horizontal field of view of the camera in degrees (default: 44.8).
     * @param vfov The vertical field of view of the camera in degrees (default: 42.6).
     */
    Viewpoint(const Eigen::Vector3d& position, const Eigen::Vector3d& lookAt, 
              const Eigen::Vector3d& up = Eigen::Vector3d::UnitY(),
              double near = 350.0, double far = 900.0, 
              int resolution_width = 2448, int resolution_height = 2048,
              double hfov = 44.8, double vfov = 42.6);

    /**
     * @brief Destructor for Viewpoint class.
     */
    ~Viewpoint();

    // Position and Orientation Methods

    /**
     * @brief Set the position of the viewpoint.
     * 
     * @param position The position of the viewpoint as an Eigen::Vector3d.
     */
    void setPosition(const Eigen::Vector3d& position);

    /**
     * @brief Get the position of the viewpoint.
     * 
     * @return The position of the viewpoint as an Eigen::Vector3d.
     */
    Eigen::Vector3d getPosition() const;

    /**
     * @brief Set the orientation of the viewpoint using a rotation matrix.
     * 
     * @param orientation The orientation of the viewpoint as a rotation matrix (Eigen::Matrix3d).
     */
    void setOrientation(const Eigen::Matrix3d& orientation);

    /**
     * @brief Set the orientation of the viewpoint using quaternions.
     * 
     * @param quaternion The orientation of the viewpoint as an Eigen::Quaterniond.
     */
    void setOrientation(const Eigen::Quaterniond& quaternion);

    /**
     * @brief Set the orientation of the viewpoint using a lookAt target point.
     * 
     * @param lookAt The target point the viewpoint is oriented towards.
     * @param up The up direction for the viewpoint (default: Eigen::Vector3d::UnitY()).
     */
    void setLookAt(const Eigen::Vector3d& lookAt, const Eigen::Vector3d& up = -Eigen::Vector3d::UnitZ());

    /**
     * @brief Get the orientation of the viewpoint as a rotation matrix.
     * 
     * @return The orientation of the viewpoint as a rotation matrix (Eigen::Matrix3d).
     */
    Eigen::Matrix3d getOrientationMatrix() const;

    /**
     * @brief Get the orientation of the viewpoint as quaternions.
     * 
     * @return The orientation of the viewpoint as an Eigen::Quaterniond.
     */
    Eigen::Quaterniond getOrientationQuaternion() const;

    /**
     * @brief Compute the transformation matrix for the viewpoint.
     * 
     * @return The transformation matrix as an Eigen::Matrix4d.
     */
    Eigen::Matrix4d getTransformationMatrix() const;

    /**
     * @brief Convert the orientation to Euler angles.
     * 
     * @return The orientation of the viewpoint as Euler angles (Eigen::Vector3d).
     */
    Eigen::Vector3d getOrientationEuler() const;

    // Camera Properties Methods

    /**
     * @brief Set the near plane distance of the camera frustum.
     * 
     * @param near The near plane distance in millimeters.
     */
    void setNearPlane(double near);

    /**
     * @brief Get the near plane distance of the camera frustum.
     * 
     * @return The near plane distance in millimeters.
     */
    double getNearPlane() const;

    /**
     * @brief Set the far plane distance of the camera frustum.
     * 
     * @param far The far plane distance in millimeters.
     */
    void setFarPlane(double far);

    /**
     * @brief Get the far plane distance of the camera frustum.
     * 
     * @return The far plane distance in millimeters.
     */
    double getFarPlane() const;

    /**
     * @brief Set the resolution of the camera.
     * 
     * @param width The width resolution of the camera.
     * @param height The height resolution of the camera.
     */
    void setResolution(int width, int height);

    /**
     * @brief Get the resolution of the camera.
     * 
     * @return A pair representing the width and height resolution of the camera.
     */
    std::pair<int, int> getResolution() const;

    /**
     * @brief Set the downsample factor for simulation.
     * 
     * @param factor The factor by which to downsample the resolution.
     */
    void setDownsampleFactor(double factor);

    /**
     * @brief Get the downsample factor for simulation.
     * 
     * @return The downsample factor.
     */
    double getDownsampleFactor() const;

    /**
     * @brief Get the downsampled resolution of the camera.
     * 
     * @return A pair representing the downsampled width and height resolution of the camera.
     */
    std::pair<int, int> getDownsampledResolution() const;

    /**
     * @brief Set the horizontal field of view (hFOV) of the camera.
     * 
     * @param hfov The horizontal field of view in degrees.
     */
    void setHorizontalFieldOfView(double hfov);

    /**
     * @brief Get the horizontal field of view (hFOV) of the camera.
     * 
     * @return The horizontal field of view in degrees.
     */
    double getHorizontalFieldOfView() const;

    /**
     * @brief Set the vertical field of view (vFOV) of the camera.
     * 
     * @param vfov The vertical field of view in degrees.
     */
    void setVerticalFieldOfView(double vfov);

    /**
     * @brief Get the vertical field of view (vFOV) of the camera.
     * 
     * @return The vertical field of view in degrees.
     */
    double getVerticalFieldOfView() const;

    /**
     * @brief Compute and return the frustum corners of the camera.
     * 
     * @return A vector of Eigen::Vector3d representing the frustum corners.
     */
    std::vector<Eigen::Vector3d> getFrustumCorners() const;


    /**
     * @brief Get the generated rays for the viewpoint.
     * 
     * @return A vector of Eigen::Vector3d representing the rays' end points from the viewpoint.
     */
    std::vector<Eigen::Vector3d> getRays() const {return rays_;}

    /**
     * @brief Perform raycasting with the generated rays on the provided octomap.
     * 
     * This method checks whether the rays generated from the viewpoint hit any occupied voxel in the octomap.
     * 
     * @param octomap The octomap to perform raycasting on.
     * @return A vector of booleans indicating whether each ray hit an occupied voxel (true) or not (false).
     */
    std::vector<std::pair<bool, Eigen::Vector3d>>  performRaycasting(const std::shared_ptr<octomap::ColorOcTree>& octomap, bool parallelize = false);


    /**
     * @brief Get the hit results of the raycasting operation.
     * 
     * @return A vector of pairs indicating whether each ray hit an occupied voxel (true) or not (false) and the hit point.
     */
    std::vector<std::pair<bool, Eigen::Vector3d>> getHitResults() const {return hits_;}

    /**
     * @brief Generate rays for each pixel in the image plane.
     * 
     * This function calculates the direction of rays for each pixel in the image plane
     * based on the camera's intrinsic and extrinsic parameters.
     * 
     * @return A vector of Eigen::Vector3d representing the direction of rays from the camera's position.
     */
    std::vector<Eigen::Vector3d> generateRays();


private:




    // Viewpoint pose
    Eigen::Vector3d position_; ///< The position of the viewpoint.
    Eigen::Matrix3d orientation_matrix_; ///< The orientation of the viewpoint as a rotation matrix.
    Eigen::Quaterniond quaternion_; ///< The orientation of the viewpoint as a quaternion.

    // Camera properties
    double near_; ///< The near plane distance of the camera frustum in millimeters.
    double far_; ///< The far plane distance of the camera frustum in millimeters.
    int resolution_width_; ///< The width resolution of the camera.
    int resolution_height_; ///< The height resolution of the camera.
    double downsample_factor_; ///< The factor by which to downsample the resolution for simulation.
    double hfov_; ///< The horizontal field of view of the camera in degrees.
    double vfov_; ///< The vertical field of view of the camera in degrees.

    // Rays
    std::vector<Eigen::Vector3d> rays_; ///< Stores the generated rays for the viewpoint.
    std::vector<std::pair<bool, Eigen::Vector3d>> hits_; ///< Stores the hit results of the raycasting operation.
};

} // namespace visioncraft

#endif // VISIONCRAFT_VIEWPOINT_H