#pragma once

#include "SpatialDb.h"
#include <string>
#include <vector>
#include <functional>

namespace MeshGeometricDeviation {

// Log severity levels
enum class LogLevel {
    Debug = 0,    // Detailed diagnostic information
    Info = 1,     // General informational messages
    Warning = 2,  // Warning messages
    Error = 3,    // Error messages
    Silent = 4    // No logging
};

// Logging function type - printf-style callback with severity level
typedef void (*LogCallback)(LogLevel level, const char* message);

// Set custom log callback (pass nullptr to use default stdout logging)
void setLogCallback(LogCallback callback);

// Set minimum log level (messages below this level are filtered out)
void setLogLevel(LogLevel level);

// Get current log level
LogLevel getLogLevel();

// Internal log function (used by library, printf-style)
void log(LogLevel level, const char* format, ...);

// Mesh structures
struct Vertex
{
    float x;
    float y;
    float z;
};

struct Triangle
{
    unsigned int i0, i1, i2;  // Vertex indices
    unsigned int t0, t1, t2;  // Texture coordinate indices (0 if no UVs)
};

struct UV
{
    float u;
    float v;
};

struct Mesh
{
    std::vector<Vertex> verts;
    std::vector<Triangle> tris;
    std::vector<UV> uvs;                 // Texture coordinates
    std::vector<Vector3> vertexNormals;  // Per-vertex normals (computed from adjacent faces)
    std::string name;
};

// Extreme case data for debug visualization
struct ExtremeCase {
    Vector3 samplePosition;       // Position of the sample point
    Vector3 closestPosition;      // Position of the closest point found
    unsigned int sampleTriIndex;  // Triangle index in source mesh
    unsigned int closestTriIndex; // Triangle index in target mesh
    double sampleBaryU, sampleBaryV, sampleBaryW;   // Barycentric coords of sample
    double closestBaryU, closestBaryV, closestBaryW; // Barycentric coords of closest point
    double value;                 // The actual value (distance, angle, etc.)
    
    ExtremeCase() : samplePosition(0,0,0), closestPosition(0,0,0), 
                   sampleTriIndex(0), closestTriIndex(0),
                   sampleBaryU(0), sampleBaryV(0), sampleBaryW(0),
                   closestBaryU(0), closestBaryV(0), closestBaryW(0),
                   value(0) {}
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
    
    // UV variance metrics
    double minUVDistance;            // Minimum distance between UV coordinates
    double maxUVDistance;            // Maximum distance between UV coordinates
    double averageUVDistance;        // Average distance between UV coordinates
    double medianUVDistance;         // Median distance between UV coordinates
    int largeUVDevianceCount;        // Count of samples with UV distance > 0.1
    
    // Extreme cases for debug visualization
    ExtremeCase maxDistanceCase;     // Case with maximum geometric distance
    ExtremeCase maxNormalAngleCase;  // Case with maximum normal angle deviation
    ExtremeCase maxUVDistanceCase;   // Case with maximum UV distance
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
                            double maxAngleDegrees = 180.0, bool useAreaWeighting = true, 
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

// Debug/Visualization functions
void exportDebugVisualization(const std::string& filename,
                              const Mesh& meshA, const Mesh& meshB,
                              const BidirectionalDevianceStats& biStats);

} // namespace MeshGeometricDeviation

