#pragma once

#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

namespace MeshGeometricDeviation {

// Forward declarations
struct Vertex;
struct Triangle;
struct Mesh;

// Vector3 utility structure - using double precision
struct Vector3
{
    double x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(double s) const { return Vector3(x * s, y * s, z * s); }
    Vector3 operator/(double s) const { return Vector3(x / s, y / s, z / s); }

    double dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3 cross(const Vector3& v) const {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    double lengthSquared() const { return x * x + y * y + z * z; }
    double length() const { return std::sqrt(lengthSquared()); }

    Vector3 normalized() const {
        double len = length();
        return len > 0 ? (*this) / len : Vector3(0, 0, 0);
    }
};

// Bounding box for spatial partitioning
struct BoundingBox
{
    Vector3 min;
    Vector3 max;

    BoundingBox() 
        : min(std::numeric_limits<float>::max(), 
              std::numeric_limits<float>::max(), 
              std::numeric_limits<float>::max()),
          max(-std::numeric_limits<float>::max(), 
              -std::numeric_limits<float>::max(), 
              -std::numeric_limits<float>::max()) {}

    BoundingBox(const Vector3& min_, const Vector3& max_) : min(min_), max(max_) {}

    void expand(const Vector3& point);
    void expand(const BoundingBox& box);
    Vector3 center() const;
    Vector3 extents() const;
    double surfaceArea() const;
    double distanceSquared(const Vector3& point) const;
};

// Triangle data for spatial database
struct TriangleData
{
    unsigned int index;      // Original triangle index in mesh
    Vector3 v0, v1, v2;      // Triangle vertices
    Vector3 normal;          // Triangle normal (normalized)
    BoundingBox bounds;      // Triangle bounding box

    TriangleData();
    TriangleData(unsigned int idx, const Vector3& a, const Vector3& b, const Vector3& c);
    Vector3 center() const;
};

// KD-Tree node for spatial partitioning
struct KdNode
{
    BoundingBox bounds;
    std::vector<TriangleData> triangles;
    int splitAxis;           // 0=x, 1=y, 2=z, -1=leaf
    double splitPos;
    KdNode* left;
    KdNode* right;

    KdNode();
    ~KdNode();
    bool isLeaf() const;
};

// Main Spatial Database class
class SpatialDb
{
public:
    // Statistics structure
    struct Stats {
        int totalNodes;
        int leafNodes;
        int totalTriangles;
        int maxDepth;
        int minTrisPerLeaf;
        int maxTrisPerLeaf;
        float avgTrisPerLeaf;
    };

    // Result structure for normal-filtered queries
    struct ClosestPointResult {
        Vector3 point;              // The closest point found
        bool matchedNormalConstraint; // True if the result matched the normal constraint
        unsigned int triangleIndex;  // Index of the triangle containing the closest point
        double baryU, baryV, baryW;  // Barycentric coordinates of the closest point
        
        ClosestPointResult();
        ClosestPointResult(const Vector3& p, bool matched, unsigned int triIdx = 0, 
                          double u = 0, double v = 0, double w = 0);
    };

    SpatialDb();
    SpatialDb(const Mesh& mesh);
    ~SpatialDb();

    // Build the spatial database from a mesh
    void build(const Mesh& mesh);

    // Query methods
    Vector3 getClosestPointOnMeshSurface(const Vector3& queryPoint) const;
    ClosestPointResult getClosestPointOnMeshSurface(const Vector3& queryPoint, const Vector3& desiredNormal, 
                                                     double maxAngleDegrees = 180.0) const;
    ClosestPointResult getClosestPointDetailed(const Vector3& queryPoint) const;
    ClosestPointResult getClosestPointDetailedWithNormal(const Vector3& queryPoint, const Vector3& desiredNormal,
                                                         double maxAngleDegrees = 180.0) const;
    double getDistanceToSurface(const Vector3& queryPoint) const;

    // Statistics
    Stats getStats() const;

private:
    KdNode* root;
    const Mesh* meshPtr;
    std::vector<TriangleData> allTriangles;

    static constexpr int MAX_TRIANGLES_PER_LEAF = 4;  // Balanced for cache vs search cost
    static constexpr int MAX_DEPTH = 16;  // Moderate depth for good performance

    KdNode* buildTree(std::vector<TriangleData>& triangles, int depth);
    Vector3 closestPointOnTriangle(const Vector3& p, const TriangleData& tri, double& distSq) const;
    Vector3 closestPointOnTriangleWithBary(const Vector3& p, const TriangleData& tri, double& distSq, 
                                           double& baryU, double& baryV, double& baryW) const;
    void searchNode(const KdNode* node, const Vector3& query, Vector3& closestPoint, double& minDistSq) const;
    void searchNodeDetailed(const KdNode* node, const Vector3& query, ClosestPointResult& result, double& minDistSq) const;
    void searchNodeWithNormal(const KdNode* node, const Vector3& query, const Vector3& desiredNormal, 
                             double minDotProduct, Vector3& closestPoint, double& minDistSq) const;
    void searchNodeWithNormalDetailed(const KdNode* node, const Vector3& query, const Vector3& desiredNormal,
                                     double minDotProduct, ClosestPointResult& result, double& minDistSq) const;
    void collectStats(const KdNode* node, Stats& stats, int depth) const;
};

} // namespace MeshGeometricDeviation

