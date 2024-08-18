#include "visioncraft/utils/poisson_sampler.h" // Include the PoissonSampler header file

#include <iostream>
#include <string>
#include <numeric>
#include <cmath>
#include <random>
#include <vector>
#include <queue>
#include <unordered_set>
#include <limits>
#include <algorithm>

#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkTriangle.h>
#include <pcl/kdtree/kdtree_flann.h> // Include the KDTree header

// Helper function to calculate Euclidean distance between two points
double euclideanDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

namespace visioncraft {
namespace utils {

// Constructor implementation for PoissonSampler
PoissonSampler::PoissonSampler() {
    // Constructor logic here
}

// Compute the surface area of a mesh data
double PoissonSampler::computeSurfaceArea(const vtkSmartPointer<vtkPolyData>& meshData) {
    double totalArea = 0.0;
    vtkSmartPointer<vtkCellArray> triangles = meshData->GetPolys();
    vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();

    // Iterate through each triangle in the mesh data and calculate its area
    for (triangles->InitTraversal(); triangles->GetNextCell(idList); ) {
        if (idList->GetNumberOfIds() == 3) { // Ensure it's a triangle
            double p0[3], p1[3], p2[3];
            meshData->GetPoint(idList->GetId(0), p0);
            meshData->GetPoint(idList->GetId(1), p1);
            meshData->GetPoint(idList->GetId(2), p2);
            totalArea += vtkTriangle::TriangleArea(p0, p1, p2);
        }
    }
    return totalArea;
}

// Generate a random point on the mesh data (optimized)
void PoissonSampler::generateRandomPoint(const vtkSmartPointer<vtkPolyData>& meshData) {

    if (triangleIndexToVertexIDs_.empty()) {
        std::cerr << "Error: Triangle mapping not initialized." << std::endl;
        return;
    }

    // Select a random triangle
    int randomIndex = rand() % triangleIndexToVertexIDs_.size();
    const auto& vertexIDs = triangleIndexToVertexIDs_[randomIndex];

    // Generate a random point within the selected triangle
    double p0[3], p1[3], p2[3];
    meshData->GetPoint(vertexIDs[0], p0);
    meshData->GetPoint(vertexIDs[1], p1);
    meshData->GetPoint(vertexIDs[2], p2);
    double r1 = sqrt(((double) rand() / (RAND_MAX)));
    double r2 = ((double) rand() / (RAND_MAX));
    double x = (1 - r1) * p0[0] + (r1 * (1 - r2)) * p1[0] + (r1 * r2) * p2[0];
    double y = (1 - r1) * p0[1] + (r1 * (1 - r2)) * p1[1] + (r1 * r2) * p2[1];
    double z = (1 - r1) * p0[2] + (r1 * (1 - r2)) * p1[2] + (r1 * r2) * p2[2];

    pointCloud_->push_back(pcl::PointXYZ(x, y, z));
}

// Select a triangle index based on the areas vector
int PoissonSampler::selectTriangleIndex(const std::vector<double>& areas) {
    double totalArea = std::accumulate(areas.begin(), areas.end(), 0.0);
    std::vector<double> cumulativeAreas(areas.size());
    std::partial_sum(areas.begin(), areas.end(), cumulativeAreas.begin());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, totalArea);
    double value = dis(gen);
    auto it = std::upper_bound(cumulativeAreas.begin(), cumulativeAreas.end(), value);
    return std::distance(cumulativeAreas.begin(), it);
}

void PoissonSampler::generateUniformSample(const vtkSmartPointer<vtkPolyData>& meshData, int numSamples){
       // Initialize point cloud and generate initial samples
    pointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

     // Precompute the map from idList to triangles
      vtkSmartPointer<vtkCellArray> triangles = meshData->GetPolys();
     vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();

        triangleIndexToVertexIDs_.clear();
        triangleIndexToVertexIDs_.reserve(triangles->GetNumberOfCells());

        for (triangles->InitTraversal(); triangles->GetNextCell(idList);) {
            std::array<int, 3> vertexIDs = {
                idList->GetId(0),
                idList->GetId(1),
                idList->GetId(2)
            };
            triangleIndexToVertexIDs_.push_back(vertexIDs);
        }
 
    // Generate initial samples
    for (int i = 0; i < numSamples; ++i) {
        generateRandomPoint(meshData);
    }

    // Print initial points have been generated
    std::cout << "Initial points generated: " << pointCloud_->size() << std::endl;
}

// Generate Poisson disk samples on the mesh data (elimination based)
void PoissonSampler::generatePoissonDiskSample(const vtkSmartPointer<vtkPolyData>& meshData, int numSamples) {
    double surfaceArea = computeSurfaceArea(meshData);
    double r_max = calculateRMax(surfaceArea, numSamples);
    int init_factor = 5;
    generateUniformSample(meshData, init_factor * numSamples);

    // Set Poisson disk sampling parameters
    double alpha = 8;    // Constant defined in paper
    double beta = 0.65;  // Constant defined in paper
    double gamma = 1.5;  // Constant defined in paper
    double r_min = r_max * beta * (1 - pow(double(numSamples) / double(pointCloud_->size()), gamma));

    // Initialize KD-tree for efficient neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pointCloud_);

    // Initialize data structures for sample elimination
    std::vector<double> weights(pointCloud_->size());
    std::vector<bool> deleted(pointCloud_->size(), false);

    // Compute weights using a priority queue for efficient computation
    std::priority_queue<std::pair<double, int>> weightQueue; // (weight, index)
    for (int i = 0; i < pointCloud_->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree.radiusSearch(pointCloud_->at(i), r_max, indices, distances);

        double weight = 0;
        for (int j : indices) {
            if (i != j && !deleted[j]) {
                double dist = euclideanDistance(pointCloud_->at(i), pointCloud_->at(j));
                if (dist < r_min) {
                    weight += pow(1 - dist / r_max, alpha);
                }
            }
        }

        weights[i] = weight;
        weightQueue.push({ weight, i });
    }

    // Perform sample elimination efficiently using the priority queue
    std::unordered_set<int> validIndices;
    while (!weightQueue.empty() && validIndices.size() < init_factor * numSamples) {
        int idx = weightQueue.top().second;
        weightQueue.pop();
        if (!deleted[idx]) {
            validIndices.insert(idx);
            std::vector<int> indices;
            std::vector<float> distances;
            kdtree.radiusSearch(pointCloud_->at(idx), r_max, indices, distances);
            for (int j : indices) {
                if (idx != j && !deleted[j]) {
                    double dist = euclideanDistance(pointCloud_->at(idx), pointCloud_->at(j));
                    if (dist < r_max) {
                        deleted[j] = true;
                        // Update weight of deleted point
                        weightQueue.push({ weights[j], j });
                    }
                }
            }
        }
    }

    // Update point cloud with valid samples
    pcl::PointCloud<pcl::PointXYZ>::Ptr updatedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i : validIndices) {
        updatedPointCloud->push_back(pointCloud_->at(i));
    }
    pointCloud_ = updatedPointCloud;
}

// Calculate the maximum radius for Poisson disk sampling
double PoissonSampler::calculateRMax(double surfaceArea, int numPoints) {
    return  sqrt(surfaceArea / (numPoints * 2 * sqrt(3)));
}

// Get the generated point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr PoissonSampler::getPointCloud() const {
    return pointCloud_;
}

} // namespace utils
} // namespace visioncraft
