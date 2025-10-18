#pragma once

#include "SpatialDb.h"
#include <string>
#include <vector>

namespace MeshGeometricDeviation {

// Mesh structures
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
    std::vector<Vector3> vertexNormals;  // Per-vertex normals (computed from adjacent faces)
    std::string name;
};

// Deviance statistics
struct DevianceStats {
    double minDeviance;
    double maxDeviance;
    double averageDeviance;
    double medianDeviance;
    double rmsd;  // Root Mean Square Deviation
    int totalSamples;
    int normalMatchedCount;
    int fallbackCount;
    int largeDevianceCount;
    
    // Normal variance metrics
    double minNormalAngleDeg;        // Minimum angle between normals (degrees)
    double maxNormalAngleDeg;        // Maximum angle between normals (degrees)
    double averageNormalAngleDeg;    // Average angle between normals (degrees)
    double medianNormalAngleDeg;     // Median angle between normals (degrees)
    int largeNormalDevianceCount;    // Count of samples with angle > 15 degrees
};

// Bidirectional comparison results
struct BidirectionalDevianceStats {
    DevianceStats aToB;  // Reference mesh A to test mesh B
    DevianceStats bToA;  // Test mesh B to reference mesh A
    double minDeviance;  // min(aToB.minDeviance, bToA.minDeviance)
    double maxDeviance;  // max(aToB.maxDeviance, bToA.maxDeviance)
    double averageDeviance; // average of both directions
    double rmsd;  // combined RMSD from both directions
    
    // Asymmetry indicators
    double asymmetryRatio;  // ratio of aToB.maxDeviance / bToA.maxDeviance (or inverse if < 1)
    bool isAsymmetric;      // true if there's significant asymmetry
};

// Mesh loading
bool loadObjFile(const std::string& filename, Mesh& mesh);

// Mesh analysis
double computeMeshSurfaceArea(const Mesh& mesh);
int computeNumSamples(const Mesh& mesh, double samplesPerUnitArea);
void computeVertexNormals(Mesh& mesh);  // Compute per-vertex normals from adjacent faces

// Mesh comparison
DevianceStats compareMeshes(const Mesh& meshA, const Mesh& meshB, int numSamples, 
                            double maxAngleDegrees = 45.0, bool useAreaWeighting = true, 
                            bool useNormalFiltering = true, unsigned int seed = 42);

BidirectionalDevianceStats compareMeshesBidirectional(const Mesh& meshA, const Mesh& meshB, 
                                                       int numSamplesA, int numSamplesB,
                                                       double maxAngleDegrees = 45.0,
                                                       bool useAreaWeighting = true, 
                                                       bool useNormalFiltering = true,
                                                       unsigned int baseSeed = 42);

// Output functions
void printDevianceStats(const DevianceStats& stats, bool showNormalStats = true);
void printBidirectionalStats(const BidirectionalDevianceStats& biStats);

} // namespace MeshGeometricDeviation

