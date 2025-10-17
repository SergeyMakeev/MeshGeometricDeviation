#include "SpatialDb.h"

// These structures are defined in main.cpp but we need them here
// In a proper project structure, these would be in a shared header

struct Vertex
{
    float x;
    float y;
    float z;
};

struct Triangle
{
    unsigned int i0, i1, i2;
};

struct Mesh
{
    std::vector<Vertex> verts;
    std::vector<Triangle> tris;
};

// Helper function to convert Vertex to Vector3
inline Vector3 vertexToVector3(const Vertex& v) {
    return Vector3(v.x, v.y, v.z);
}

void SpatialDb::build(const Mesh& mesh) {
    meshPtr = &mesh;
    
    // Clear existing tree
    delete root;
    root = nullptr;
    allTriangles.clear();

    // Convert mesh triangles to TriangleData
    allTriangles.reserve(mesh.tris.size());
    
    for (size_t i = 0; i < mesh.tris.size(); i++) {
        const Triangle& tri = mesh.tris[i];
        
        // Get triangle vertices
        const Vector3 v0 = vertexToVector3(mesh.verts[tri.i0]);
        const Vector3 v1 = vertexToVector3(mesh.verts[tri.i1]);
        const Vector3 v2 = vertexToVector3(mesh.verts[tri.i2]);
        
        allTriangles.emplace_back(static_cast<unsigned int>(i), v0, v1, v2);
    }

    // Build KD-tree
    if (!allTriangles.empty()) {
        root = buildTree(allTriangles, 0);
    }
}

