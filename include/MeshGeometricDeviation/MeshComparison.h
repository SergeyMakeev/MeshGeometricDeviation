#pragma once

#include "SpatialDb.h"
#include <functional>
#include <string>
#include <vector>

namespace MeshGeometricDeviation
{

// Log severity levels
enum class LogLevel
{
    Debug = 0,   // Detailed diagnostic information
    Info = 1,    // General informational messages
    Warning = 2, // Warning messages
    Error = 3,   // Error messages
    Silent = 4   // No logging
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
    unsigned int i0, i1, i2; // Vertex indices
    unsigned int t0, t1, t2; // Texture coordinate indices (0 if no UVs)
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
    std::vector<UV> uvs;                // Texture coordinates
    std::vector<Vector3> vertexNormals; // Per-vertex normals (computed from adjacent faces)
    std::string name;
};

// Extreme case data for debug visualization
struct ExtremeCase
{
    Vector3 samplePosition;                          // Position of the sample point
    Vector3 closestPosition;                         // Position of the closest point found
    unsigned int sampleTriIndex;                     // Triangle index in source mesh
    unsigned int closestTriIndex;                    // Triangle index in target mesh
    double sampleBaryU, sampleBaryV, sampleBaryW;    // Barycentric coords of sample
    double closestBaryU, closestBaryV, closestBaryW; // Barycentric coords of closest point
    double value;                                    // The actual value (distance, angle, etc.)

    ExtremeCase()
        : samplePosition(0, 0, 0)
        , closestPosition(0, 0, 0)
        , sampleTriIndex(0)
        , closestTriIndex(0)
        , sampleBaryU(0)
        , sampleBaryV(0)
        , sampleBaryW(0)
        , closestBaryU(0)
        , closestBaryV(0)
        , closestBaryW(0)
        , value(0)
    {
    }
};

// Deviance statistics
struct DevianceStats
{
    double minDeviance;
    double maxDeviance;
    double averageDeviance;
    double medianDeviance;
    double rmsd; // Root Mean Square Deviation
    int totalSamples;
    int normalMatchedCount;
    int fallbackCount;
    int largeDevianceCount;

    // Normal variance metrics
    double minNormalAngleDeg;     // Minimum angle between normals (degrees)
    double maxNormalAngleDeg;     // Maximum angle between normals (degrees)
    double averageNormalAngleDeg; // Average angle between normals (degrees)
    double medianNormalAngleDeg;  // Median angle between normals (degrees)
    int largeNormalDevianceCount; // Count of samples with angle > 15 degrees

    // UV variance metrics
    double minUVDistance;     // Minimum distance between UV coordinates
    double maxUVDistance;     // Maximum distance between UV coordinates
    double averageUVDistance; // Average distance between UV coordinates
    double medianUVDistance;  // Median distance between UV coordinates
    int largeUVDevianceCount; // Count of samples with UV distance > 0.1

    // Extreme cases for debug visualization
    ExtremeCase maxDistanceCase;    // Case with maximum geometric distance
    ExtremeCase maxNormalAngleCase; // Case with maximum normal angle deviation
    ExtremeCase maxUVDistanceCase;  // Case with maximum UV distance
};

// Bidirectional comparison results
struct BidirectionalDevianceStats
{
    DevianceStats aToB;     // Reference mesh A to test mesh B
    DevianceStats bToA;     // Test mesh B to reference mesh A
    double minDeviance;     // min(aToB.minDeviance, bToA.minDeviance)
    double maxDeviance;     // max(aToB.maxDeviance, bToA.maxDeviance)
    double averageDeviance; // average of both directions
    double rmsd;            // combined RMSD from both directions

    // Asymmetry indicators
    double asymmetryRatio; // ratio of aToB.maxDeviance / bToA.maxDeviance (or inverse if < 1)
    bool isAsymmetric;     // true if there's significant asymmetry
};

// ============================================================================
// Mesh Loading
// ============================================================================

/**
 * Load a mesh from an OBJ file
 *
 * Automatically triangulates polygons and loads vertices, UVs, and normals.
 * Note: fast_obj library adds dummy entries at index 0 for positions, UVs, and normals.
 *
 * @param filename Path to the OBJ file
 * @param mesh Output mesh structure
 * @return true if loading succeeded, false otherwise
 */
bool loadObjFile(const std::string& filename, Mesh& mesh);

// ============================================================================
// Mesh Analysis
// ============================================================================

/**
 * Compute total surface area of a mesh
 *
 * @param mesh Input mesh
 * @return Total surface area in square units
 */
double computeMeshSurfaceArea(const Mesh& mesh);

/**
 * Compute number of samples based on surface area and density
 *
 * Ensures at least one sample per triangle.
 *
 * @param mesh Input mesh
 * @param samplesPerUnitArea Desired sample density (samples per square unit)
 * @return Number of samples to generate
 */
int computeNumSamples(const Mesh& mesh, double samplesPerUnitArea);

/**
 * Compute per-vertex normals from adjacent faces
 *
 * Uses area-weighted averaging of adjacent face normals.
 * Required for normal variance analysis.
 *
 * @param mesh Mesh to compute normals for (modified in place)
 */
void computeVertexNormals(Mesh& mesh);

// ============================================================================
// Mesh Comparison
// ============================================================================

/**
 * Compare two meshes unidirectionally (A -> B)
 *
 * Samples points on meshA and finds their closest points on meshB.
 * Measures geometric distance, normal variance (if vertex normals present),
 * and UV variance (if UVs present).
 *
 * @param meshA Reference mesh to sample from
 * @param meshB Target mesh to find closest points on
 * @param numSamples Number of points to sample on meshA
 * @param maxAngleDegrees Maximum angle in degrees for normal-aware queries (default: 180.0)
 *                        180.0 effectively disables normal filtering
 * @param useAreaWeighting If true, distributes samples by triangle area (default: true)
 * @param useNormalFiltering If true, prefers triangles with similar normals (default: true)
 * @param seed Random seed for reproducible sampling (default: 42)
 * @return DevianceStats containing distance, normal, and UV statistics
 */
DevianceStats compareMeshes(const Mesh& meshA, const Mesh& meshB, int numSamples, double maxAngleDegrees = 180.0,
                            bool useAreaWeighting = true, bool useNormalFiltering = true, unsigned int seed = 42);

/**
 * Compare two meshes bidirectionally (A -> B and B -> A)
 *
 * Performs comparison in both directions to detect both missing geometry (holes)
 * and extra geometry. Computes asymmetry metrics and combined statistics.
 *
 * @param meshA Reference mesh
 * @param meshB Test mesh
 * @param numSamplesA Number of samples to take on meshA
 * @param numSamplesB Number of samples to take on meshB
 * @param maxAngleDegrees Maximum angle in degrees for normal-aware queries (default: 45.0)
 *                        Note: Different default than compareMeshes() for stricter matching
 * @param useAreaWeighting If true, distributes samples by triangle area (default: true)
 * @param useNormalFiltering If true, prefers triangles with similar normals (default: true)
 * @param baseSeed Base random seed (uses baseSeed for A->B, baseSeed+1 for B->A) (default: 42)
 * @return BidirectionalDevianceStats with results for both directions and combined metrics
 */
BidirectionalDevianceStats compareMeshesBidirectional(const Mesh& meshA, const Mesh& meshB, int numSamplesA, int numSamplesB,
                                                      double maxAngleDegrees = 45.0, bool useAreaWeighting = true,
                                                      bool useNormalFiltering = true, unsigned int baseSeed = 42);

// ============================================================================
// Output Functions
// ============================================================================

/**
 * Print deviance statistics to console
 *
 * Displays min/max/average/median distance, RMSD, normal variance, UV variance,
 * and large deviation counts.
 *
 * @param stats Deviance statistics to print
 * @param showNormalStats If true, shows normal constraint statistics (default: true)
 */
void printDevianceStats(const DevianceStats& stats, bool showNormalStats = true);

/**
 * Print bidirectional comparison statistics to console
 *
 * Displays results for both directions (A->B and B->A), combined metrics,
 * and asymmetry analysis.
 *
 * @param biStats Bidirectional deviance statistics to print
 */
void printBidirectionalStats(const BidirectionalDevianceStats& biStats);

// ============================================================================
// Debug/Visualization Functions
// ============================================================================

/**
 * Export debug visualization to OBJ file
 *
 * Creates an OBJ file containing:
 * - Both input meshes with normals and UVs
 * - Sphere markers at extreme deviation points (max distance, normal angle, UV distance)
 * - Large spheres = sample points, small spheres = closest points
 *
 * @param filename Output OBJ file path
 * @param meshA Reference mesh
 * @param meshB Test mesh
 * @param biStats Bidirectional statistics containing extreme case information
 */
void exportDebugVisualization(const std::string& filename, const Mesh& meshA, const Mesh& meshB, const BidirectionalDevianceStats& biStats);

/**
 * Export a single mesh to OBJ file
 *
 * For debugging/testing mesh loading and processing.
 * Writes vertices, normals, UVs, and faces in standard OBJ format.
 *
 * @param filename Output OBJ file path
 * @param mesh Mesh to export
 * @return true if export succeeded, false otherwise
 */
bool exportMeshToObj(const std::string& filename, const Mesh& mesh);

} // namespace MeshGeometricDeviation
