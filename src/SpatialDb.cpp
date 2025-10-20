#include "MeshGeometricDeviation/SpatialDb.h"
#include "MeshGeometricDeviation/MeshComparison.h"

namespace MeshGeometricDeviation
{

// Helper function to convert Vertex to Vector3
inline Vector3 vertexToVector3(const Vertex& v) { return Vector3(v.x, v.y, v.z); }

// BoundingBox method implementations
void BoundingBox::expand(const Vector3& point)
{
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
}

void BoundingBox::expand(const BoundingBox& box)
{
    expand(box.min);
    expand(box.max);
}

Vector3 BoundingBox::center() const { return (min + max) * 0.5; }

Vector3 BoundingBox::extents() const { return (max - min) * 0.5; }

double BoundingBox::surfaceArea() const
{
    Vector3 d = max - min;
    return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

double BoundingBox::distanceSquared(const Vector3& point) const
{
    double dx = std::max(std::max(min.x - point.x, point.x - max.x), 0.0);
    double dy = std::max(std::max(min.y - point.y, point.y - max.y), 0.0);
    double dz = std::max(std::max(min.z - point.z, point.z - max.z), 0.0);
    return dx * dx + dy * dy + dz * dz;
}

// TriangleData method implementations
TriangleData::TriangleData()
    : index(0)
{
}

TriangleData::TriangleData(unsigned int idx, const Vector3& a, const Vector3& b, const Vector3& c)
    : index(idx)
    , v0(a)
    , v1(b)
    , v2(c)
{
    bounds.expand(v0);
    bounds.expand(v1);
    bounds.expand(v2);

    // Compute triangle normal
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    normal = edge1.cross(edge2).normalized();
}

Vector3 TriangleData::center() const { return (v0 + v1 + v2) * (1.0 / 3.0); }

// KdNode method implementations
KdNode::KdNode()
    : splitAxis(-1)
    , splitPos(0.0)
    , left(nullptr)
    , right(nullptr)
{
}

KdNode::~KdNode()
{
    delete left;
    delete right;
}

bool KdNode::isLeaf() const { return splitAxis == -1; }

// SpatialDb method implementations
SpatialDb::SpatialDb()
    : root(nullptr)
    , meshPtr(nullptr)
{
}

SpatialDb::SpatialDb(const Mesh& mesh)
    : root(nullptr)
    , meshPtr(&mesh)
{
    build(mesh);
}

SpatialDb::~SpatialDb() { delete root; }

SpatialDb::ClosestPointResult::ClosestPointResult()
    : point(0, 0, 0)
    , matchedNormalConstraint(false)
    , triangleIndex(0)
    , baryU(0)
    , baryV(0)
    , baryW(0)
{
}

SpatialDb::ClosestPointResult::ClosestPointResult(const Vector3& p, bool matched, unsigned int triIdx, double u, double v, double w)
    : point(p)
    , matchedNormalConstraint(matched)
    , triangleIndex(triIdx)
    , baryU(u)
    , baryV(v)
    , baryW(w)
{
}

void SpatialDb::build(const Mesh& mesh)
{
    meshPtr = &mesh;

    // Clear existing tree
    delete root;
    root = nullptr;
    allTriangles.clear();

    // Convert mesh triangles to TriangleData
    allTriangles.reserve(mesh.tris.size());

    for (size_t i = 0; i < mesh.tris.size(); i++)
    {
        const Triangle& tri = mesh.tris[i];

        // Get triangle vertices
        const Vector3 v0 = vertexToVector3(mesh.verts[tri.i0]);
        const Vector3 v1 = vertexToVector3(mesh.verts[tri.i1]);
        const Vector3 v2 = vertexToVector3(mesh.verts[tri.i2]);

        allTriangles.emplace_back(static_cast<unsigned int>(i), v0, v1, v2);
    }

    // Build KD-tree
    if (!allTriangles.empty())
    {
        root = buildTree(allTriangles, 0);
    }
}

KdNode* SpatialDb::buildTree(std::vector<TriangleData>& triangles, int depth)
{
    if (triangles.empty())
    {
        return nullptr;
    }

    KdNode* node = new KdNode();

    for (const auto& tri : triangles)
    {
        node->bounds.expand(tri.bounds);
    }

    if (triangles.size() <= MAX_TRIANGLES_PER_LEAF || depth >= MAX_DEPTH)
    {
        node->triangles = triangles;
        return node;
    }

    Vector3 extent = node->bounds.extents();
    int axis = 0;
    if (extent.y > extent.x)
        axis = 1;
    if (extent.z > ((axis == 0) ? extent.x : extent.y))
        axis = 2;

    node->splitAxis = axis;

    std::sort(triangles.begin(), triangles.end(),
              [axis](const TriangleData& a, const TriangleData& b)
              {
                  Vector3 ca = a.center();
                  Vector3 cb = b.center();
                  return (axis == 0) ? (ca.x < cb.x) : ((axis == 1) ? (ca.y < cb.y) : (ca.z < cb.z));
              });

    size_t mid = triangles.size() / 2;
    Vector3 splitCenter = triangles[mid].center();
    node->splitPos = (axis == 0) ? splitCenter.x : ((axis == 1) ? splitCenter.y : splitCenter.z);

    std::vector<TriangleData> leftTris(triangles.begin(), triangles.begin() + mid);
    std::vector<TriangleData> rightTris(triangles.begin() + mid, triangles.end());

    node->left = buildTree(leftTris, depth + 1);
    node->right = buildTree(rightTris, depth + 1);

    return node;
}

Vector3 SpatialDb::closestPointOnTriangle(const Vector3& p, const TriangleData& tri, double& distSq) const
{
    const Vector3& a = tri.v0;
    const Vector3& b = tri.v1;
    const Vector3& c = tri.v2;

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;

    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
        distSq = ap.lengthSquared();
        return a;
    }

    Vector3 bp = p - b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
    {
        distSq = bp.lengthSquared();
        return b;
    }

    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        double v = d1 / (d1 - d3);
        Vector3 closest = a + ab * v;
        distSq = (p - closest).lengthSquared();
        return closest;
    }

    Vector3 cp = p - c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
    {
        distSq = cp.lengthSquared();
        return c;
    }

    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        double w = d2 / (d2 - d6);
        Vector3 closest = a + ac * w;
        distSq = (p - closest).lengthSquared();
        return closest;
    }

    double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        Vector3 closest = b + (c - b) * w;
        distSq = (p - closest).lengthSquared();
        return closest;
    }

    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    Vector3 closest = a + ab * v + ac * w;
    distSq = (p - closest).lengthSquared();
    return closest;
}

Vector3 SpatialDb::closestPointOnTriangleWithBary(const Vector3& p, const TriangleData& tri, double& distSq, double& baryU, double& baryV,
                                                  double& baryW) const
{
    const Vector3& a = tri.v0;
    const Vector3& b = tri.v1;
    const Vector3& c = tri.v2;

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;

    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
        distSq = ap.lengthSquared();
        baryU = 1.0;
        baryV = 0.0;
        baryW = 0.0;
        return a;
    }

    Vector3 bp = p - b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
    {
        distSq = bp.lengthSquared();
        baryU = 0.0;
        baryV = 1.0;
        baryW = 0.0;
        return b;
    }

    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        double v = d1 / (d1 - d3);
        Vector3 closest = a + ab * v;
        distSq = (p - closest).lengthSquared();
        baryU = 1.0 - v;
        baryV = v;
        baryW = 0.0;
        return closest;
    }

    Vector3 cp = p - c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
    {
        distSq = cp.lengthSquared();
        baryU = 0.0;
        baryV = 0.0;
        baryW = 1.0;
        return c;
    }

    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        double w = d2 / (d2 - d6);
        Vector3 closest = a + ac * w;
        distSq = (p - closest).lengthSquared();
        baryU = 1.0 - w;
        baryV = 0.0;
        baryW = w;
        return closest;
    }

    double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        Vector3 closest = b + (c - b) * w;
        distSq = (p - closest).lengthSquared();
        baryU = 0.0;
        baryV = 1.0 - w;
        baryW = w;
        return closest;
    }

    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    double u = 1.0 - v - w;
    Vector3 closest = a + ab * v + ac * w;
    distSq = (p - closest).lengthSquared();
    baryU = u;
    baryV = v;
    baryW = w;
    return closest;
}

void SpatialDb::searchNode(const KdNode* node, const Vector3& query, Vector3& closestPoint, double& minDistSq) const
{
    if (!node)
        return;

    double boxDistSq = node->bounds.distanceSquared(query);
    if (boxDistSq >= minDistSq)
    {
        return;
    }

    if (node->isLeaf())
    {
        for (const auto& tri : node->triangles)
        {
            double distSq;
            Vector3 candidate = closestPointOnTriangle(query, tri, distSq);
            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                closestPoint = candidate;
            }
        }
    }
    else
    {
        double queryPos = (node->splitAxis == 0) ? query.x : ((node->splitAxis == 1) ? query.y : query.z);

        KdNode* nearChild = (queryPos < node->splitPos) ? node->left : node->right;
        KdNode* farChild = (queryPos < node->splitPos) ? node->right : node->left;

        searchNode(nearChild, query, closestPoint, minDistSq);
        searchNode(farChild, query, closestPoint, minDistSq);
    }
}

void SpatialDb::searchNodeDetailed(const KdNode* node, const Vector3& query, ClosestPointResult& result, double& minDistSq) const
{
    if (!node)
        return;

    double boxDistSq = node->bounds.distanceSquared(query);
    if (boxDistSq >= minDistSq)
    {
        return;
    }

    if (node->isLeaf())
    {
        for (const auto& tri : node->triangles)
        {
            double distSq;
            double baryU, baryV, baryW;
            Vector3 candidate = closestPointOnTriangleWithBary(query, tri, distSq, baryU, baryV, baryW);
            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                result.point = candidate;
                result.triangleIndex = tri.index;
                result.baryU = baryU;
                result.baryV = baryV;
                result.baryW = baryW;
            }
        }
    }
    else
    {
        double queryPos = (node->splitAxis == 0) ? query.x : ((node->splitAxis == 1) ? query.y : query.z);

        KdNode* nearChild = (queryPos < node->splitPos) ? node->left : node->right;
        KdNode* farChild = (queryPos < node->splitPos) ? node->right : node->left;

        searchNodeDetailed(nearChild, query, result, minDistSq);
        searchNodeDetailed(farChild, query, result, minDistSq);
    }
}

void SpatialDb::searchNodeWithNormal(const KdNode* node, const Vector3& query, const Vector3& desiredNormal, double minDotProduct,
                                     Vector3& closestPoint, double& minDistSq) const
{
    if (!node)
        return;

    double boxDistSq = node->bounds.distanceSquared(query);
    if (boxDistSq >= minDistSq)
    {
        return;
    }

    if (node->isLeaf())
    {
        for (const auto& tri : node->triangles)
        {
            double dotProduct = tri.normal.dot(desiredNormal);
            if (dotProduct >= minDotProduct)
            {
                double distSq;
                Vector3 candidate = closestPointOnTriangle(query, tri, distSq);
                if (distSq < minDistSq)
                {
                    minDistSq = distSq;
                    closestPoint = candidate;
                }
            }
        }
    }
    else
    {
        double queryPos = (node->splitAxis == 0) ? query.x : ((node->splitAxis == 1) ? query.y : query.z);

        KdNode* nearChild = (queryPos < node->splitPos) ? node->left : node->right;
        KdNode* farChild = (queryPos < node->splitPos) ? node->right : node->left;

        searchNodeWithNormal(nearChild, query, desiredNormal, minDotProduct, closestPoint, minDistSq);
        searchNodeWithNormal(farChild, query, desiredNormal, minDotProduct, closestPoint, minDistSq);
    }
}

void SpatialDb::searchNodeWithNormalDetailed(const KdNode* node, const Vector3& query, const Vector3& desiredNormal, double minDotProduct,
                                             ClosestPointResult& result, double& minDistSq) const
{
    if (!node)
        return;

    double boxDistSq = node->bounds.distanceSquared(query);
    if (boxDistSq >= minDistSq)
    {
        return;
    }

    if (node->isLeaf())
    {
        for (const auto& tri : node->triangles)
        {
            double dotProduct = tri.normal.dot(desiredNormal);
            if (dotProduct >= minDotProduct)
            {
                double distSq;
                double baryU, baryV, baryW;
                Vector3 candidate = closestPointOnTriangleWithBary(query, tri, distSq, baryU, baryV, baryW);
                if (distSq < minDistSq)
                {
                    minDistSq = distSq;
                    result.point = candidate;
                    result.triangleIndex = tri.index;
                    result.baryU = baryU;
                    result.baryV = baryV;
                    result.baryW = baryW;
                }
            }
        }
    }
    else
    {
        double queryPos = (node->splitAxis == 0) ? query.x : ((node->splitAxis == 1) ? query.y : query.z);

        KdNode* nearChild = (queryPos < node->splitPos) ? node->left : node->right;
        KdNode* farChild = (queryPos < node->splitPos) ? node->right : node->left;

        searchNodeWithNormalDetailed(nearChild, query, desiredNormal, minDotProduct, result, minDistSq);
        searchNodeWithNormalDetailed(farChild, query, desiredNormal, minDotProduct, result, minDistSq);
    }
}

Vector3 SpatialDb::getClosestPointOnMeshSurface(const Vector3& queryPoint) const
{
    if (!root)
    {
        return Vector3(0, 0, 0);
    }

    Vector3 closestPoint;
    double minDistSq = std::numeric_limits<double>::max();

    searchNode(root, queryPoint, closestPoint, minDistSq);

    return closestPoint;
}

SpatialDb::ClosestPointResult SpatialDb::getClosestPointOnMeshSurface(const Vector3& queryPoint, const Vector3& desiredNormal,
                                                                      double maxAngleDegrees) const
{
    if (!root)
    {
        return ClosestPointResult(Vector3(0, 0, 0), false);
    }

    Vector3 normalizedDesiredNormal = desiredNormal.normalized();

    double maxAngleRadians = maxAngleDegrees * 3.14159265358979323846 / 180.0;
    double minDotProduct = std::cos(maxAngleRadians);

    Vector3 closestPoint;
    double minDistSq = std::numeric_limits<double>::max();

    searchNodeWithNormal(root, queryPoint, normalizedDesiredNormal, minDotProduct, closestPoint, minDistSq);

    if (minDistSq == std::numeric_limits<double>::max())
    {
        searchNode(root, queryPoint, closestPoint, minDistSq);
        return ClosestPointResult(closestPoint, false);
    }

    return ClosestPointResult(closestPoint, true);
}

double SpatialDb::getDistanceToSurface(const Vector3& queryPoint) const
{
    Vector3 closest = getClosestPointOnMeshSurface(queryPoint);
    return (queryPoint - closest).length();
}

SpatialDb::ClosestPointResult SpatialDb::getClosestPointDetailed(const Vector3& queryPoint) const
{
    if (!root)
    {
        return ClosestPointResult(Vector3(0, 0, 0), false);
    }

    ClosestPointResult result;
    result.matchedNormalConstraint = false; // Not using normal filtering
    double minDistSq = std::numeric_limits<double>::max();

    searchNodeDetailed(root, queryPoint, result, minDistSq);

    return result;
}

SpatialDb::ClosestPointResult SpatialDb::getClosestPointDetailedWithNormal(const Vector3& queryPoint, const Vector3& desiredNormal,
                                                                           double maxAngleDegrees) const
{
    if (!root)
    {
        return ClosestPointResult(Vector3(0, 0, 0), false);
    }

    Vector3 normalizedDesiredNormal = desiredNormal.normalized();

    double maxAngleRadians = maxAngleDegrees * 3.14159265358979323846 / 180.0;
    double minDotProduct = std::cos(maxAngleRadians);

    ClosestPointResult result;
    double minDistSq = std::numeric_limits<double>::max();

    searchNodeWithNormalDetailed(root, queryPoint, normalizedDesiredNormal, minDotProduct, result, minDistSq);

    if (minDistSq == std::numeric_limits<double>::max())
    {
        // Fallback to search without normal filtering
        searchNodeDetailed(root, queryPoint, result, minDistSq);
        result.matchedNormalConstraint = false;
    }
    else
    {
        result.matchedNormalConstraint = true;
    }

    return result;
}

SpatialDb::Stats SpatialDb::getStats() const
{
    Stats stats = {0, 0, 0, 0, std::numeric_limits<int>::max(), 0, 0.0f};

    if (root)
    {
        collectStats(root, stats, 0);
        if (stats.leafNodes > 0)
        {
            stats.avgTrisPerLeaf = static_cast<float>(stats.totalTriangles) / stats.leafNodes;
        }
    }

    return stats;
}

void SpatialDb::collectStats(const KdNode* node, SpatialDb::Stats& stats, int depth) const
{
    if (!node)
        return;

    stats.totalNodes++;
    stats.maxDepth = std::max(stats.maxDepth, depth);

    if (node->isLeaf())
    {
        stats.leafNodes++;
        int triCount = static_cast<int>(node->triangles.size());
        stats.totalTriangles += triCount;
        stats.minTrisPerLeaf = std::min(stats.minTrisPerLeaf, triCount);
        stats.maxTrisPerLeaf = std::max(stats.maxTrisPerLeaf, triCount);
    }
    else
    {
        collectStats(node->left, stats, depth + 1);
        collectStats(node->right, stats, depth + 1);
    }
}

} // namespace MeshGeometricDeviation
