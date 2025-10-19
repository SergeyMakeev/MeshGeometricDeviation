#include "MeshGeometricDeviation/MeshComparison.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdarg>
#include <cstdio>

#define FAST_OBJ_IMPLEMENTATION
#include "fast_obj.h"

namespace MeshGeometricDeviation {

// Global log callback and log level settings
static LogCallback g_logCallback = nullptr;
static LogLevel g_logLevel = LogLevel::Info;

// Default log function (prints to stdout with level prefix)
static void defaultLogCallback(LogLevel level, const char* message) {
    const char* levelStr = "";
    switch (level) {
        case LogLevel::Debug:   levelStr = "[DEBUG] "; break;
        case LogLevel::Info:    levelStr = ""; break;  // No prefix for info
        case LogLevel::Warning: levelStr = "[WARNING] "; break;
        case LogLevel::Error:   levelStr = "[ERROR] "; break;
        default: break;
    }
    std::printf("%s%s", levelStr, message);
}

// Set custom log callback
void setLogCallback(LogCallback callback) {
    g_logCallback = callback;
}

// Set minimum log level
void setLogLevel(LogLevel level) {
    g_logLevel = level;
}

// Get current log level
LogLevel getLogLevel() {
    return g_logLevel;
}

// Internal log function (printf-style with level filtering)
void log(LogLevel level, const char* format, ...) {
    // Filter out messages below current log level
    if (level < g_logLevel) {
        return;
    }
    
    if (g_logCallback == nullptr) {
        g_logCallback = defaultLogCallback;
    }
    
    char buffer[4096];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    g_logCallback(level, buffer);
}

// Helper to convert Vertex to Vector3
inline Vector3 vertexToVector3(const Vertex& v) {
    return Vector3(v.x, v.y, v.z);
}

// Surface sample (internal structure)
struct SurfaceSample {
    Vector3 position;
    Vector3 normal;
    unsigned int triangleIndex;
    double baryU, baryV, baryW;  // Barycentric coordinates
};

// Sample a random point on a triangle using barycentric coordinates
static SurfaceSample samplePointOnTriangle(const Mesh& mesh, unsigned int triIdx, std::mt19937& rng) {
    const Triangle& tri = mesh.tris[triIdx];
    const Vector3 v0 = vertexToVector3(mesh.verts[tri.i0]);
    const Vector3 v1 = vertexToVector3(mesh.verts[tri.i1]);
    const Vector3 v2 = vertexToVector3(mesh.verts[tri.i2]);
    
    // Generate random barycentric coordinates using uniform distribution
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double r1 = dist(rng);
    double r2 = dist(rng);
    
    // Convert to barycentric coordinates (ensures uniform distribution)
    double sqrtR1 = std::sqrt(r1);
    double u = 1.0 - sqrtR1;
    double v = sqrtR1 * (1.0 - r2);
    double w = sqrtR1 * r2;
    
    // Compute position
    Vector3 position = v0 * u + v1 * v + v2 * w;
    
    // Compute normal
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 normal = edge1.cross(edge2).normalized();
    
    SurfaceSample sample;
    sample.position = position;
    sample.normal = normal;
    sample.triangleIndex = triIdx;
    sample.baryU = u;
    sample.baryV = v;
    sample.baryW = w;
    
    return sample;
}

// Precompute triangle areas (cache for performance)
static std::vector<double> computeTriangleAreas(const Mesh& mesh, double& totalArea) {
    std::vector<double> triangleAreas;
    triangleAreas.reserve(mesh.tris.size());
    
    totalArea = 0.0;
    for (const Triangle& tri : mesh.tris) {
        const Vector3 v0 = vertexToVector3(mesh.verts[tri.i0]);
        const Vector3 v1 = vertexToVector3(mesh.verts[tri.i1]);
        const Vector3 v2 = vertexToVector3(mesh.verts[tri.i2]);
        
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        double area = edge1.cross(edge2).length() * 0.5;
        
        triangleAreas.push_back(area);
        totalArea += area;
    }
    
    return triangleAreas;
}

// Generate area-weighted random samples
// Ensures each triangle gets at least one sample
static std::vector<SurfaceSample> generateAreaWeightedSamples(const Mesh& mesh, int numSamples, unsigned int seed) {
    std::vector<SurfaceSample> samples;
    samples.reserve(numSamples);
    
    // Compute triangle areas once
    double totalArea = 0.0;
    std::vector<double> triangleAreas = computeTriangleAreas(mesh, totalArea);
    
    std::mt19937 rng(seed);
    
    // First pass: Give each triangle at least one sample
    int samplesGenerated = 0;
    for (size_t triIdx = 0; triIdx < mesh.tris.size(); triIdx++) {
        samples.push_back(samplePointOnTriangle(mesh, static_cast<unsigned int>(triIdx), rng));
        samplesGenerated++;
    }
    
    // Second pass: Distribute remaining samples based on area
    int remainingSamples = numSamples - samplesGenerated;
    if (remainingSamples > 0) {
        double samplesPerUnitArea = remainingSamples / totalArea;
        double fractionalRemainder = 0.0;
        
        for (size_t triIdx = 0; triIdx < mesh.tris.size(); triIdx++) {
            // Calculate exact number of additional samples for this triangle
            double exactSamples = triangleAreas[triIdx] * samplesPerUnitArea + fractionalRemainder;
            int samplesForThisTriangle = static_cast<int>(exactSamples);
            fractionalRemainder = exactSamples - samplesForThisTriangle;
            
            // Generate additional samples for this triangle
            for (int i = 0; i < samplesForThisTriangle; i++) {
                samples.push_back(samplePointOnTriangle(mesh, static_cast<unsigned int>(triIdx), rng));
                samplesGenerated++;
            }
        }
        
        // Handle any final remaining samples due to rounding
        if (samplesGenerated < numSamples) {
            // Create a list of triangle indices sorted by area (largest first)
            std::vector<unsigned int> trianglesByArea(mesh.tris.size());
            for (size_t i = 0; i < trianglesByArea.size(); i++) {
                trianglesByArea[i] = static_cast<unsigned int>(i);
            }
            std::sort(trianglesByArea.begin(), trianglesByArea.end(), 
                      [&triangleAreas](unsigned int a, unsigned int b) {
                          return triangleAreas[a] > triangleAreas[b];
                      });
            
            // Add final remaining samples to largest triangles
            int finalRemaining = numSamples - samplesGenerated;
            for (int i = 0; i < finalRemaining; i++) {
                unsigned int triIdx = trianglesByArea[i % trianglesByArea.size()];
                samples.push_back(samplePointOnTriangle(mesh, triIdx, rng));
            }
        }
    }
    
    return samples;
}

// Load OBJ file using fast_obj
bool loadObjFile(const std::string& filename, Mesh& mesh) {
    fastObjMesh* obj = fast_obj_read(filename.c_str());
    if (!obj) {
        log(LogLevel::Error, "Failed to load OBJ file: %s\n", filename.c_str());
        return false;
    }

    mesh.name = filename;
    
    // Reserve space
    mesh.verts.reserve(obj->position_count);
    mesh.uvs.reserve(obj->texcoord_count + 1);  // +1 for index 0 (no UV)
    
    // Add a default UV at index 0 for faces without UVs
    UV defaultUV;
    defaultUV.u = 0.0f;
    defaultUV.v = 0.0f;
    mesh.uvs.push_back(defaultUV);
    
    // Load vertices
    for (unsigned int i = 0; i < obj->position_count; i++) {
        Vertex v;
        v.x = obj->positions[3 * i + 0];
        v.y = obj->positions[3 * i + 1];
        v.z = obj->positions[3 * i + 2];
        mesh.verts.push_back(v);
    }
    
    // Load texture coordinates
    for (unsigned int i = 0; i < obj->texcoord_count; i++) {
        UV uv;
        uv.u = obj->texcoords[2 * i + 0];
        uv.v = obj->texcoords[2 * i + 1];
        mesh.uvs.push_back(uv);
    }
    
    // Load normals from OBJ file if available
    // OBJ normals are per-corner (indexed), we need to convert to per-vertex
    // Note: fast_obj adds a dummy normal at index 0, so normal_count > 1 means actual normals exist
    bool hasNormals = (obj->normal_count > 1);
    if (hasNormals) {
        // Initialize per-vertex normal accumulation
        std::vector<Vector3> normalAccum(obj->position_count, Vector3(0, 0, 0));
        std::vector<int> normalCount(obj->position_count, 0);
        
        // Accumulate normals for each vertex from all its uses
        unsigned int faceVertexOffset = 0;
        for (unsigned int faceIdx = 0; faceIdx < obj->face_count; faceIdx++) {
            unsigned int vertCount = obj->face_vertices[faceIdx];
            
            for (unsigned int i = 0; i < vertCount; i++) {
                unsigned int posIdx = obj->indices[faceVertexOffset + i].p;
                unsigned int normIdx = obj->indices[faceVertexOffset + i].n;
                
                // If this vertex has a normal specified
                if (normIdx > 0 && normIdx <= obj->normal_count) {
                    Vector3 normal(
                        obj->normals[3 * (normIdx - 1) + 0],
                        obj->normals[3 * (normIdx - 1) + 1],
                        obj->normals[3 * (normIdx - 1) + 2]
                    );
                    normalAccum[posIdx] = normalAccum[posIdx] + normal;
                    normalCount[posIdx]++;
                }
            }
            
            faceVertexOffset += vertCount;
        }
        
        // Average and normalize the accumulated normals
        mesh.vertexNormals.reserve(obj->position_count);
        bool hasInvalidNormals = false;
        for (unsigned int i = 0; i < obj->position_count; i++) {
            if (normalCount[i] > 0) {
                Vector3 avgNormal = normalAccum[i] * (1.0 / normalCount[i]);
                mesh.vertexNormals.push_back(avgNormal.normalized());
            } else {
                // Vertex had no normal specified
                // Skip vertex 0 in the check (it's a dummy vertex added by fast_obj)
                if (i > 0) {
                    hasInvalidNormals = true;
                }
                mesh.vertexNormals.push_back(Vector3(0, 0, 0));
            }
        }
        
        // If any actual vertices (not the dummy at index 0) are missing normals,
        // the OBJ file has incomplete normal data - clear them so they'll be computed
        if (hasInvalidNormals) {
            log(LogLevel::Info, "  Warning: Some vertices missing normals, will compute from geometry\n");
            mesh.vertexNormals.clear();
            hasNormals = false;
        }
    }
    
    // Load triangles
    unsigned int faceVertexOffset = 0;
    for (unsigned int faceIdx = 0; faceIdx < obj->face_count; faceIdx++) {
        unsigned int vertCount = obj->face_vertices[faceIdx];
        
        if (vertCount == 3) {
            Triangle tri;
            tri.i0 = obj->indices[faceVertexOffset + 0].p;
            tri.i1 = obj->indices[faceVertexOffset + 1].p;
            tri.i2 = obj->indices[faceVertexOffset + 2].p;
            tri.t0 = obj->indices[faceVertexOffset + 0].t;
            tri.t1 = obj->indices[faceVertexOffset + 1].t;
            tri.t2 = obj->indices[faceVertexOffset + 2].t;
            mesh.tris.push_back(tri);
        } else if (vertCount > 3) {
            // Triangulate polygon (simple fan triangulation)
            for (unsigned int i = 2; i < vertCount; i++) {
                Triangle tri;
                tri.i0 = obj->indices[faceVertexOffset + 0].p;
                tri.i1 = obj->indices[faceVertexOffset + i - 1].p;
                tri.i2 = obj->indices[faceVertexOffset + i].p;
                tri.t0 = obj->indices[faceVertexOffset + 0].t;
                tri.t1 = obj->indices[faceVertexOffset + i - 1].t;
                tri.t2 = obj->indices[faceVertexOffset + i].t;
                mesh.tris.push_back(tri);
            }
        }
        
        faceVertexOffset += vertCount;
    }
    
    fast_obj_destroy(obj);
    
    log(LogLevel::Info, "Loaded mesh: %s\n", filename.c_str());
    log(LogLevel::Info, "  Vertices: %zu\n", mesh.verts.size());
    log(LogLevel::Info, "  Triangles: %zu\n", mesh.tris.size());
    log(LogLevel::Debug, "  UVs: %zu\n", mesh.uvs.size() - 1);  // -1 to exclude default UV
    if (hasNormals) {
        log(LogLevel::Info, "  Vertex normals: %zu (loaded from file)\n", mesh.vertexNormals.size());
    } else {
        log(LogLevel::Info, "  Vertex normals: none (will be computed if needed)\n");
    }
    
    return true;
}

// Compute total surface area of mesh
double computeMeshSurfaceArea(const Mesh& mesh) {
    double totalArea = 0.0;
    computeTriangleAreas(mesh, totalArea);
    return totalArea;
}

// Compute per-vertex normals from adjacent faces (area-weighted average)
void computeVertexNormals(Mesh& mesh) {
    // Initialize vertex normals to zero
    mesh.vertexNormals.clear();
    mesh.vertexNormals.resize(mesh.verts.size(), Vector3(0, 0, 0));
    
    // Accumulate face normals weighted by face area
    for (const Triangle& tri : mesh.tris) {
        const Vector3 v0 = vertexToVector3(mesh.verts[tri.i0]);
        const Vector3 v1 = vertexToVector3(mesh.verts[tri.i1]);
        const Vector3 v2 = vertexToVector3(mesh.verts[tri.i2]);
        
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 faceNormal = edge1.cross(edge2);  // Unnormalized (weighted by area)
        
        // Add to each vertex of the triangle
        mesh.vertexNormals[tri.i0] = mesh.vertexNormals[tri.i0] + faceNormal;
        mesh.vertexNormals[tri.i1] = mesh.vertexNormals[tri.i1] + faceNormal;
        mesh.vertexNormals[tri.i2] = mesh.vertexNormals[tri.i2] + faceNormal;
    }
    
    // Normalize all vertex normals
    for (Vector3& normal : mesh.vertexNormals) {
        normal = normal.normalized();
    }
}

// Compute number of samples based on surface area and sample density
// Ensures at least one sample per triangle
int computeNumSamples(const Mesh& mesh, double samplesPerUnitArea) {
    double totalArea = computeMeshSurfaceArea(mesh);
    int numSamples = static_cast<int>(std::ceil(totalArea * samplesPerUnitArea));
    
    // Ensure at least one sample per triangle
    int minSamples = static_cast<int>(mesh.tris.size());
    if (numSamples < minSamples) {
        numSamples = minSamples;
    }
    
    return numSamples;
}

// Compare two meshes
DevianceStats compareMeshes(const Mesh& meshA, const Mesh& meshB, int numSamples, 
                            double maxAngleDegrees, bool useAreaWeighting, 
                            bool useNormalFiltering, unsigned int seed) {
    log(LogLevel::Info, "\n=== Comparing Meshes ===\n");
    log(LogLevel::Info, "Reference mesh: %s (%zu triangles)\n", meshA.name.c_str(), meshA.tris.size());
    log(LogLevel::Info, "Test mesh: %s (%zu triangles)\n", meshB.name.c_str(), meshB.tris.size());
    log(LogLevel::Info, "Samples: %d\n", numSamples);
    log(LogLevel::Info, "Normal filtering: %s\n", useNormalFiltering ? "enabled" : "disabled");
    if (useNormalFiltering) {
        log(LogLevel::Debug, "Normal angle threshold: %.1f degrees\n", maxAngleDegrees);
    }
    log(LogLevel::Debug, "Area weighting: %s\n", useAreaWeighting ? "enabled" : "disabled");
    log(LogLevel::Debug, "Random seed: %u (for reproducibility)\n", seed);
    
    // Build spatial database for meshB
    log(LogLevel::Info, "\nBuilding spatial database for test mesh...\n");
    auto t_start_db = std::chrono::high_resolution_clock::now();
    SpatialDb spatialDb(meshB);
    auto t_end_db = std::chrono::high_resolution_clock::now();
    double time_db = std::chrono::duration<double>(t_end_db - t_start_db).count();
    
    auto stats = spatialDb.getStats();
    log(LogLevel::Debug, "Spatial database stats:\n");
    log(LogLevel::Debug, "  Total nodes: %d\n", stats.totalNodes);
    log(LogLevel::Debug, "  Leaf nodes: %d\n", stats.leafNodes);
    log(LogLevel::Debug, "  Max depth: %d\n", stats.maxDepth);
    log(LogLevel::Debug, "  Avg triangles per leaf: %.2f\n", stats.avgTrisPerLeaf);
    log(LogLevel::Debug, "  Build time: %.6f seconds\n", time_db);
    
    // Generate samples on meshA
    log(LogLevel::Info, "\nGenerating surface samples on reference mesh...\n");
    auto t_start_sample = std::chrono::high_resolution_clock::now();
    std::vector<SurfaceSample> samples = generateAreaWeightedSamples(meshA, numSamples, seed);
    auto t_end_sample = std::chrono::high_resolution_clock::now();
    double time_sample = std::chrono::duration<double>(t_end_sample - t_start_sample).count();
    log(LogLevel::Debug, "  Sampling time: %.6f seconds\n", time_sample);
    
    // Check if both meshes have vertex normals for normal variance computation
    bool hasVertexNormals = (!meshA.vertexNormals.empty() && !meshB.vertexNormals.empty());
    
    // Check if both meshes have UVs for UV variance computation
    bool hasUVs = (meshA.uvs.size() > 1 && meshB.uvs.size() > 1);
    
    // Measure distances
    log(LogLevel::Info, "\nMeasuring distances");
    if (hasVertexNormals) log(LogLevel::Info, " + normal variance");
    if (hasUVs) log(LogLevel::Info, " + UV variance");
    log(LogLevel::Info, "...\n");
    
    auto t_start_measure = std::chrono::high_resolution_clock::now();
    
    std::vector<double> distances;
    std::vector<double> normalAngles;  // Angles between interpolated vertex normals (degrees)
    std::vector<double> uvDistances;   // Distances between interpolated UV coordinates
    distances.reserve(numSamples);
    if (hasVertexNormals) {
        normalAngles.reserve(numSamples);
    }
    if (hasUVs) {
        uvDistances.reserve(numSamples);
    }
    
    int normalMatchedCount = 0;
    int fallbackCount = 0;
    int largeDevianceCount = 0;
    int largeNormalDevianceCount = 0;
    int largeUVDevianceCount = 0;
    const double largeDevianceThreshold = 1e-4; // 0.0001 units
    const double largeNormalAngleThreshold = 15.0; // 15 degrees
    const double largeUVThreshold = 0.1; // 0.1 UV units
    
    // Track extreme cases for debug visualization
    ExtremeCase maxDistanceCase;
    ExtremeCase maxNormalAngleCase;
    ExtremeCase maxUVDistanceCase;
    
    for (const SurfaceSample& sample : samples) {
        double distance;
        SpatialDb::ClosestPointResult result;
        
        if (useNormalFiltering) {
            result = spatialDb.getClosestPointDetailedWithNormal(
                sample.position, sample.normal, maxAngleDegrees);
            
            distance = (result.point - sample.position).length();
            
            if (result.matchedNormalConstraint) {
                normalMatchedCount++;
            } else {
                fallbackCount++;
            }
        } else {
            // Use simple closest point query without normal filtering
            result = spatialDb.getClosestPointDetailed(sample.position);
            distance = (result.point - sample.position).length();
            normalMatchedCount++; // Not applicable, but count for consistency
        }
        
        distances.push_back(distance);
        
        if (distance > largeDevianceThreshold) {
            largeDevianceCount++;
        }
        
        // Track max distance case
        if (distance > maxDistanceCase.value) {
            maxDistanceCase.samplePosition = sample.position;
            maxDistanceCase.closestPosition = result.point;
            maxDistanceCase.sampleTriIndex = sample.triangleIndex;
            maxDistanceCase.closestTriIndex = result.triangleIndex;
            maxDistanceCase.sampleBaryU = sample.baryU;
            maxDistanceCase.sampleBaryV = sample.baryV;
            maxDistanceCase.sampleBaryW = sample.baryW;
            maxDistanceCase.closestBaryU = result.baryU;
            maxDistanceCase.closestBaryV = result.baryV;
            maxDistanceCase.closestBaryW = result.baryW;
            maxDistanceCase.value = distance;
        }
        
        // Compute normal variance if both meshes have vertex normals
        // Cache triangle lookups to improve performance
        if (hasVertexNormals) {
            const Triangle& sampleTri = meshA.tris[sample.triangleIndex];
            const Triangle& closestTri = meshB.tris[result.triangleIndex];
            
            // Interpolate normals using barycentric coordinates
            Vector3 sampleVertexNormal = 
                meshA.vertexNormals[sampleTri.i0] * sample.baryU +
                meshA.vertexNormals[sampleTri.i1] * sample.baryV +
                meshA.vertexNormals[sampleTri.i2] * sample.baryW;
            sampleVertexNormal = sampleVertexNormal.normalized();
            
            Vector3 closestVertexNormal = 
                meshB.vertexNormals[closestTri.i0] * result.baryU +
                meshB.vertexNormals[closestTri.i1] * result.baryV +
                meshB.vertexNormals[closestTri.i2] * result.baryW;
            closestVertexNormal = closestVertexNormal.normalized();
            
            // Compute angle between normals (optimized)
            double dotProduct = std::max(-1.0, std::min(1.0, sampleVertexNormal.dot(closestVertexNormal)));
            double angleDegrees = std::acos(dotProduct) * 57.2957795131;  // Rad to deg constant
            
            normalAngles.push_back(angleDegrees);
            largeNormalDevianceCount += (angleDegrees > largeNormalAngleThreshold) ? 1 : 0;
            
            // Track max normal angle case
            if (angleDegrees > maxNormalAngleCase.value) {
                maxNormalAngleCase.samplePosition = sample.position;
                maxNormalAngleCase.closestPosition = result.point;
                maxNormalAngleCase.sampleTriIndex = sample.triangleIndex;
                maxNormalAngleCase.closestTriIndex = result.triangleIndex;
                maxNormalAngleCase.sampleBaryU = sample.baryU;
                maxNormalAngleCase.sampleBaryV = sample.baryV;
                maxNormalAngleCase.sampleBaryW = sample.baryW;
                maxNormalAngleCase.closestBaryU = result.baryU;
                maxNormalAngleCase.closestBaryV = result.baryV;
                maxNormalAngleCase.closestBaryW = result.baryW;
                maxNormalAngleCase.value = angleDegrees;
            }
        }
        
        // Compute UV variance if both meshes have UVs
        // Use cached triangle references from normal variance if available
        if (hasUVs) {
            const Triangle& sampleTri = meshA.tris[sample.triangleIndex];
            const Triangle& closestTri = meshB.tris[result.triangleIndex];
            
            // Check if both triangles have valid UV indices
            if (sampleTri.t0 > 0 && sampleTri.t1 > 0 && sampleTri.t2 > 0 &&
                closestTri.t0 > 0 && closestTri.t1 > 0 && closestTri.t2 > 0) {
                
                // Interpolate UVs directly (optimized)
                const UV& uv0_s = meshA.uvs[sampleTri.t0];
                const UV& uv1_s = meshA.uvs[sampleTri.t1];
                const UV& uv2_s = meshA.uvs[sampleTri.t2];
                
                double sampleU = uv0_s.u * sample.baryU + uv1_s.u * sample.baryV + uv2_s.u * sample.baryW;
                double sampleV = uv0_s.v * sample.baryU + uv1_s.v * sample.baryV + uv2_s.v * sample.baryW;
                
                const UV& uv0_c = meshB.uvs[closestTri.t0];
                const UV& uv1_c = meshB.uvs[closestTri.t1];
                const UV& uv2_c = meshB.uvs[closestTri.t2];
                
                double closestU = uv0_c.u * result.baryU + uv1_c.u * result.baryV + uv2_c.u * result.baryW;
                double closestV = uv0_c.v * result.baryU + uv1_c.v * result.baryV + uv2_c.v * result.baryW;
                
                // Compute UV distance (optimized)
                double du = sampleU - closestU;
                double dv = sampleV - closestV;
                double uvDistance = std::sqrt(du * du + dv * dv);
                
                uvDistances.push_back(uvDistance);
                largeUVDevianceCount += (uvDistance > largeUVThreshold) ? 1 : 0;
                
                // Track max UV distance case
                if (uvDistance > maxUVDistanceCase.value) {
                    maxUVDistanceCase.samplePosition = sample.position;
                    maxUVDistanceCase.closestPosition = result.point;
                    maxUVDistanceCase.sampleTriIndex = sample.triangleIndex;
                    maxUVDistanceCase.closestTriIndex = result.triangleIndex;
                    maxUVDistanceCase.sampleBaryU = sample.baryU;
                    maxUVDistanceCase.sampleBaryV = sample.baryV;
                    maxUVDistanceCase.sampleBaryW = sample.baryW;
                    maxUVDistanceCase.closestBaryU = result.baryU;
                    maxUVDistanceCase.closestBaryV = result.baryV;
                    maxUVDistanceCase.closestBaryW = result.baryW;
                    maxUVDistanceCase.value = uvDistance;
                }
            }
        }
    }
    
    auto t_end_measure = std::chrono::high_resolution_clock::now();
    double time_measure = std::chrono::duration<double>(t_end_measure - t_start_measure).count();
    
    log(LogLevel::Debug, "  Measurement time: %.6f seconds\n", time_measure);
    log(LogLevel::Debug, "  Samples/second: %.0f\n", (numSamples / time_measure));
    
    // Compute statistics
    auto t_start_stats = std::chrono::high_resolution_clock::now();
    
    DevianceStats devianceStats;
    devianceStats.totalSamples = numSamples;
    devianceStats.normalMatchedCount = normalMatchedCount;
    devianceStats.fallbackCount = fallbackCount;
    devianceStats.largeDevianceCount = largeDevianceCount;
    devianceStats.largeNormalDevianceCount = largeNormalDevianceCount;
    
    // Min and Max deviance
    devianceStats.minDeviance = *std::min_element(distances.begin(), distances.end());
    devianceStats.maxDeviance = *std::max_element(distances.begin(), distances.end());
    
    // Average deviance and RMSD
    double sum = 0.0;
    double sumSquares = 0.0;
    for (double d : distances) {
        sum += d;
        sumSquares += d * d;
    }
    devianceStats.averageDeviance = sum / numSamples;
    devianceStats.rmsd = std::sqrt(sumSquares / numSamples);
    
    // Median deviance
    std::vector<double> sortedDistances = distances;
    std::sort(sortedDistances.begin(), sortedDistances.end());
    if (numSamples % 2 == 0) {
        devianceStats.medianDeviance = (sortedDistances[numSamples / 2 - 1] + 
                                        sortedDistances[numSamples / 2]) * 0.5;
    } else {
        devianceStats.medianDeviance = sortedDistances[numSamples / 2];
    }
    
    // Compute normal variance statistics if available
    if (hasVertexNormals && !normalAngles.empty()) {
        devianceStats.minNormalAngleDeg = *std::min_element(normalAngles.begin(), normalAngles.end());
        devianceStats.maxNormalAngleDeg = *std::max_element(normalAngles.begin(), normalAngles.end());
        
        double angleSum = 0.0;
        for (double angle : normalAngles) {
            angleSum += angle;
        }
        devianceStats.averageNormalAngleDeg = angleSum / normalAngles.size();
        
        std::vector<double> sortedAngles = normalAngles;
        std::sort(sortedAngles.begin(), sortedAngles.end());
        if (normalAngles.size() % 2 == 0) {
            devianceStats.medianNormalAngleDeg = (sortedAngles[normalAngles.size() / 2 - 1] + 
                                                  sortedAngles[normalAngles.size() / 2]) * 0.5;
        } else {
            devianceStats.medianNormalAngleDeg = sortedAngles[normalAngles.size() / 2];
        }
    } else {
        // No vertex normals available
        devianceStats.minNormalAngleDeg = 0.0;
        devianceStats.maxNormalAngleDeg = 0.0;
        devianceStats.averageNormalAngleDeg = 0.0;
        devianceStats.medianNormalAngleDeg = 0.0;
    }
    
    // Compute UV variance statistics if available
    devianceStats.largeUVDevianceCount = largeUVDevianceCount;
    if (hasUVs && !uvDistances.empty()) {
        devianceStats.minUVDistance = *std::min_element(uvDistances.begin(), uvDistances.end());
        devianceStats.maxUVDistance = *std::max_element(uvDistances.begin(), uvDistances.end());
        
        double uvSum = 0.0;
        for (double uvDist : uvDistances) {
            uvSum += uvDist;
        }
        devianceStats.averageUVDistance = uvSum / uvDistances.size();
        
        std::vector<double> sortedUVs = uvDistances;
        std::sort(sortedUVs.begin(), sortedUVs.end());
        if (sortedUVs.size() % 2 == 0) {
            devianceStats.medianUVDistance = (sortedUVs[sortedUVs.size() / 2 - 1] + 
                                              sortedUVs[sortedUVs.size() / 2]) * 0.5;
        } else {
            devianceStats.medianUVDistance = sortedUVs[sortedUVs.size() / 2];
        }
    } else {
        // No UVs available
        devianceStats.minUVDistance = 0.0;
        devianceStats.maxUVDistance = 0.0;
        devianceStats.averageUVDistance = 0.0;
        devianceStats.medianUVDistance = 0.0;
    }
    
    // Store extreme cases for debug visualization
    devianceStats.maxDistanceCase = maxDistanceCase;
    devianceStats.maxNormalAngleCase = maxNormalAngleCase;
    devianceStats.maxUVDistanceCase = maxUVDistanceCase;
    
    auto t_end_stats = std::chrono::high_resolution_clock::now();
    double time_stats = std::chrono::duration<double>(t_end_stats - t_start_stats).count();
    
    log(LogLevel::Debug, "  Statistics time: %.6f seconds\n", time_stats);
    
    // Total time
    double time_total = time_db + time_sample + time_measure + time_stats;
    log(LogLevel::Info, "\nTotal comparison time: %.6f seconds\n", time_total);
    log(LogLevel::Debug, "  KD-tree build: %.2f%%\n", (time_db / time_total * 100));
    log(LogLevel::Debug, "  Sampling:      %.2f%%\n", (time_sample / time_total * 100));
    log(LogLevel::Debug, "  Measurement:   %.2f%%\n", (time_measure / time_total * 100));
    log(LogLevel::Debug, "  Statistics:    %.2f%%\n", (time_stats / time_total * 100));
    
    return devianceStats;
}

// Bidirectional mesh comparison
BidirectionalDevianceStats compareMeshesBidirectional(const Mesh& meshA, const Mesh& meshB, 
                                                       int numSamplesA, int numSamplesB,
                                                       double maxAngleDegrees,
                                                       bool useAreaWeighting, bool useNormalFiltering,
                                                       unsigned int baseSeed) {
    BidirectionalDevianceStats biStats;
    
    log(LogLevel::Info, "\n=== Bidirectional Mesh Comparison ===\n");
    log(LogLevel::Info, "This compares in both directions to detect missing/extra geometry\n");
    
    // Direction 1: A -> B (reference to test)
    log(LogLevel::Info, "\n--- Direction 1: Reference (A) -> Test (B) ---\n");
    biStats.aToB = compareMeshes(meshA, meshB, numSamplesA, maxAngleDegrees, useAreaWeighting, useNormalFiltering, baseSeed);
    
    // Direction 2: B -> A (test to reference)
    log(LogLevel::Info, "\n--- Direction 2: Test (B) -> Reference (A) ---\n");
    biStats.bToA = compareMeshes(meshB, meshA, numSamplesB, maxAngleDegrees, useAreaWeighting, useNormalFiltering, baseSeed + 1);
    
    // Compute overall statistics
    biStats.minDeviance = std::min(biStats.aToB.minDeviance, biStats.bToA.minDeviance);
    biStats.maxDeviance = std::max(biStats.aToB.maxDeviance, biStats.bToA.maxDeviance);
    biStats.averageDeviance = (biStats.aToB.averageDeviance + biStats.bToA.averageDeviance) * 0.5;
    
    // Combined RMSD: sqrt((RMSD_A^2 + RMSD_B^2) / 2)
    double rmsdA_squared = biStats.aToB.rmsd * biStats.aToB.rmsd;
    double rmsdB_squared = biStats.bToA.rmsd * biStats.bToA.rmsd;
    biStats.rmsd = std::sqrt((rmsdA_squared + rmsdB_squared) * 0.5);
    
    // Compute asymmetry
    double maxA = biStats.aToB.maxDeviance;
    double maxB = biStats.bToA.maxDeviance;
    
    if (maxA > maxB) {
        biStats.asymmetryRatio = maxA / (maxB + 1e-15); // Add epsilon to avoid division by zero
    } else {
        biStats.asymmetryRatio = maxB / (maxA + 1e-15);
    }
    
    // Consider asymmetric if ratio > 1.5 (50% difference)
    biStats.isAsymmetric = (biStats.asymmetryRatio > 1.5);
    
    return biStats;
}

// Print deviance statistics
void printDevianceStats(const DevianceStats& stats, bool showNormalStats) {
    log(LogLevel::Info, "\n=== Deviance Statistics ===\n");
    log(LogLevel::Info, "Min deviance:     %g\n", stats.minDeviance);
    log(LogLevel::Info, "Max deviance:     %g\n", stats.maxDeviance);
    log(LogLevel::Info, "Average deviance: %g\n", stats.averageDeviance);
    log(LogLevel::Info, "Median deviance:  %g\n", stats.medianDeviance);
    log(LogLevel::Info, "RMSD:             %g\n", stats.rmsd);
    
    if (showNormalStats && stats.fallbackCount >= 0) {
        log(LogLevel::Info, "\nNormal constraint:\n");
        log(LogLevel::Debug, "  Matched:  %d samples (%.2f%%)\n", stats.normalMatchedCount,
            100.0f * stats.normalMatchedCount / stats.totalSamples);
        log(LogLevel::Debug, "  Fallback: %d samples (%.2f%%)\n", stats.fallbackCount,
            100.0f * stats.fallbackCount / stats.totalSamples);
    }
    
    log(LogLevel::Info, "\nPrecision analysis:\n");
    log(LogLevel::Debug, "  Large deviations (>0.0001): %d samples (%.2f%%)\n", stats.largeDevianceCount,
        100.0f * stats.largeDevianceCount / stats.totalSamples);
    
    // Display normal variance if available
    if (stats.maxNormalAngleDeg > 0.0) {
        log(LogLevel::Info, "\nVertex Normal Variance (interpolated):\n");
        log(LogLevel::Info, "  Min angle:     %g degrees\n", stats.minNormalAngleDeg);
        log(LogLevel::Info, "  Max angle:     %g degrees\n", stats.maxNormalAngleDeg);
        log(LogLevel::Info, "  Average angle: %g degrees\n", stats.averageNormalAngleDeg);
        log(LogLevel::Info, "  Median angle:  %g degrees\n", stats.medianNormalAngleDeg);
        log(LogLevel::Info, "  Large normal deviations (>15 deg): %d samples (%.2f%%)\n", 
            stats.largeNormalDevianceCount,
            100.0f * stats.largeNormalDevianceCount / stats.totalSamples);
    }
    
    // Warn about self-comparison issues
    if (stats.averageDeviance > 1e-12 && stats.fallbackCount == 0) {
        log(LogLevel::Info, "\nNote: If comparing identical meshes, non-zero deviance indicates numerical\n");
        log(LogLevel::Warning, "      precision issues in the spatial query. This is expected due to floating-\n");
        log(LogLevel::Info, "      point arithmetic, especially near edges/vertices where multiple triangles meet.\n");
    }
}

// Print bidirectional comparison statistics
void printBidirectionalStats(const BidirectionalDevianceStats& biStats) {
    log(LogLevel::Info, "\n=== Bidirectional Comparison Results ===\n");
    log(LogLevel::Info, "\nReference -> Test (A -> B):\n");
    log(LogLevel::Info, "  Min deviance:     %g\n", biStats.aToB.minDeviance);
    log(LogLevel::Info, "  Max deviance:     %g\n", biStats.aToB.maxDeviance);
    log(LogLevel::Info, "  Average deviance: %g\n", biStats.aToB.averageDeviance);
    log(LogLevel::Info, "  Median deviance:  %g\n", biStats.aToB.medianDeviance);
    log(LogLevel::Info, "  RMSD:             %g\n", biStats.aToB.rmsd);
    
    log(LogLevel::Info, "\nTest -> Reference (B -> A):\n");
    log(LogLevel::Info, "  Min deviance:     %g\n", biStats.bToA.minDeviance);
    log(LogLevel::Info, "  Max deviance:     %g\n", biStats.bToA.maxDeviance);
    log(LogLevel::Info, "  Average deviance: %g\n", biStats.bToA.averageDeviance);
    log(LogLevel::Info, "  Median deviance:  %g\n", biStats.bToA.medianDeviance);
    log(LogLevel::Info, "  RMSD:             %g\n", biStats.bToA.rmsd);
    
    log(LogLevel::Info, "\nOverall (Symmetric Hausdorff-like):\n");
    log(LogLevel::Info, "  Min deviance:     %g\n", biStats.minDeviance);
    log(LogLevel::Info, "  Max deviance:     %g\n", biStats.maxDeviance);
    log(LogLevel::Info, "  Average deviance: %g\n", biStats.averageDeviance);
    log(LogLevel::Info, "  RMSD:             %g\n", biStats.rmsd);
    
    log(LogLevel::Info, "\nAsymmetry Analysis:\n");
    if (biStats.isAsymmetric) {
        log(LogLevel::Info, "  Status: ASYMMETRIC (ratio: %gx)\n", biStats.asymmetryRatio);
        if (biStats.aToB.maxDeviance > biStats.bToA.maxDeviance * 1.5) {
            log(LogLevel::Info, "  -> Reference mesh has points far from test mesh\n");
            log(LogLevel::Info, "     (Test mesh may have missing geometry/holes)\n");
        } else if (biStats.bToA.maxDeviance > biStats.aToB.maxDeviance * 1.5) {
            log(LogLevel::Info, "  -> Test mesh has points far from reference mesh\n");
            log(LogLevel::Info, "     (Test mesh may have extra geometry)\n");
        }
    } else {
        log(LogLevel::Info, "  Status: Symmetric (ratio: %gx)\n", biStats.asymmetryRatio);
        log(LogLevel::Info, "  -> Deviations are similar in both directions\n");
    }
    
    // Detail on large deviations
    log(LogLevel::Info, "\nLarge deviations (>0.0001) per direction:\n");
    log(LogLevel::Debug, "  A -> B: %d samples (%.2f%%)\n", biStats.aToB.largeDevianceCount,
        100.0 * biStats.aToB.largeDevianceCount / biStats.aToB.totalSamples);
    log(LogLevel::Debug, "  B -> A: %d samples (%.2f%%)\n", biStats.bToA.largeDevianceCount,
        100.0 * biStats.bToA.largeDevianceCount / biStats.bToA.totalSamples);
    
    // Display normal variance if available
    if (biStats.aToB.maxNormalAngleDeg > 0.0 || biStats.bToA.maxNormalAngleDeg > 0.0) {
        log(LogLevel::Info, "\n=== Vertex Normal Variance (Interpolated) ===\n");
        
        if (biStats.aToB.maxNormalAngleDeg > 0.0) {
            log(LogLevel::Info, "\nReference -> Test (A -> B):\n");
            log(LogLevel::Info, "  Min angle:     %g degrees\n", biStats.aToB.minNormalAngleDeg);
            log(LogLevel::Info, "  Max angle:     %g degrees\n", biStats.aToB.maxNormalAngleDeg);
            log(LogLevel::Info, "  Average angle: %g degrees\n", biStats.aToB.averageNormalAngleDeg);
            log(LogLevel::Info, "  Median angle:  %g degrees\n", biStats.aToB.medianNormalAngleDeg);
            log(LogLevel::Debug, "  Large deviations (>15 deg): %d samples (%.2f%%)\n",
                biStats.aToB.largeNormalDevianceCount,
                100.0 * biStats.aToB.largeNormalDevianceCount / biStats.aToB.totalSamples);
        }
        
        if (biStats.bToA.maxNormalAngleDeg > 0.0) {
            log(LogLevel::Info, "\nTest -> Reference (B -> A):\n");
            log(LogLevel::Info, "  Min angle:     %g degrees\n", biStats.bToA.minNormalAngleDeg);
            log(LogLevel::Info, "  Max angle:     %g degrees\n", biStats.bToA.maxNormalAngleDeg);
            log(LogLevel::Info, "  Average angle: %g degrees\n", biStats.bToA.averageNormalAngleDeg);
            log(LogLevel::Info, "  Median angle:  %g degrees\n", biStats.bToA.medianNormalAngleDeg);
            log(LogLevel::Debug, "  Large deviations (>15 deg): %d samples (%.2f%%)\n",
                biStats.bToA.largeNormalDevianceCount,
                100.0 * biStats.bToA.largeNormalDevianceCount / biStats.bToA.totalSamples);
        }
    }
    
    // Display UV variance if available
    if (biStats.aToB.maxUVDistance > 0.0 || biStats.bToA.maxUVDistance > 0.0) {
        log(LogLevel::Info, "\n=== UV Coordinate Variance (Interpolated) ===\n");
        
        if (biStats.aToB.maxUVDistance > 0.0) {
            log(LogLevel::Info, "\nReference -> Test (A -> B):\n");
            log(LogLevel::Info, "  Min UV distance:     %g\n", biStats.aToB.minUVDistance);
            log(LogLevel::Info, "  Max UV distance:     %g\n", biStats.aToB.maxUVDistance);
            log(LogLevel::Info, "  Average UV distance: %g\n", biStats.aToB.averageUVDistance);
            log(LogLevel::Info, "  Median UV distance:  %g\n", biStats.aToB.medianUVDistance);
            log(LogLevel::Debug, "  Large deviations (>0.1): %d samples (%.2f%%)\n",
                biStats.aToB.largeUVDevianceCount,
                100.0 * biStats.aToB.largeUVDevianceCount / biStats.aToB.totalSamples);
        }
        
        if (biStats.bToA.maxUVDistance > 0.0) {
            log(LogLevel::Info, "\nTest -> Reference (B -> A):\n");
            log(LogLevel::Info, "  Min UV distance:     %g\n", biStats.bToA.minUVDistance);
            log(LogLevel::Info, "  Max UV distance:     %g\n", biStats.bToA.maxUVDistance);
            log(LogLevel::Info, "  Average UV distance: %g\n", biStats.bToA.averageUVDistance);
            log(LogLevel::Info, "  Median UV distance:  %g\n", biStats.bToA.medianUVDistance);
            log(LogLevel::Debug, "  Large deviations (>0.1): %d samples (%.2f%%)\n",
                biStats.bToA.largeUVDevianceCount,
                100.0 * biStats.bToA.largeUVDevianceCount / biStats.bToA.totalSamples);
        }
    }
}

// Helper function to generate a sphere mesh
static void generateSphere(std::ofstream& out, const Vector3& center, double radius, int& vertexOffset, const std::string& objName) {
    const int segments = 8;  // Low-poly sphere for visibility
    const int rings = 6;
    
    out << "\no " << objName << "\n";
    
    int startVertex = vertexOffset + 1;
    
    // Generate vertices
    for (int ring = 0; ring <= rings; ring++) {
        double phi = 3.14159265358979323846 * ring / rings;
        double sinPhi = std::sin(phi);
        double cosPhi = std::cos(phi);
        
        for (int seg = 0; seg < segments; seg++) {
            double theta = 2.0 * 3.14159265358979323846 * seg / segments;
            double sinTheta = std::sin(theta);
            double cosTheta = std::cos(theta);
            
            double x = center.x + radius * sinPhi * cosTheta;
            double y = center.y + radius * sinPhi * sinTheta;
            double z = center.z + radius * cosPhi;
            
            out << "v " << x << " " << y << " " << z << "\n";
            vertexOffset++;
        }
    }
    
    // Generate faces
    for (int ring = 0; ring < rings; ring++) {
        for (int seg = 0; seg < segments; seg++) {
            int current = startVertex + ring * segments + seg;
            int next = startVertex + ring * segments + ((seg + 1) % segments);
            int currentNext = startVertex + (ring + 1) * segments + seg;
            int nextNext = startVertex + (ring + 1) * segments + ((seg + 1) % segments);
            
            // Create two triangles for this quad
            out << "f " << current << " " << currentNext << " " << next << "\n";
            out << "f " << next << " " << currentNext << " " << nextNext << "\n";
        }
    }
}

// Export debug visualization showing meshes and extreme deviation points
void exportDebugVisualization(const std::string& filename,
                              const Mesh& meshA, const Mesh& meshB,
                              const BidirectionalDevianceStats& biStats) {
    
    log(LogLevel::Info, "\n=== Generating Debug Visualization ===\n");
    log(LogLevel::Info, "Using pre-computed comparison results (Direction A->B)\n");
    
    // Get extreme cases from A->B comparison
    const ExtremeCase& maxDistanceCase = biStats.aToB.maxDistanceCase;
    const ExtremeCase& maxNormalAngleCase = biStats.aToB.maxNormalAngleCase;
    const ExtremeCase& maxUVDistanceCase = biStats.aToB.maxUVDistanceCase;
    
    bool hasVertexNormals = (!meshA.vertexNormals.empty() && !meshB.vertexNormals.empty());
    bool hasUVs = (meshA.uvs.size() > 1 && meshB.uvs.size() > 1);
    
    // Print diagnostic information from pre-computed extreme cases
    log(LogLevel::Info, "\n=== Extreme Cases (from comparison) ===\n");
    
    log(LogLevel::Info, "\nMax Distance: %g\n", maxDistanceCase.value);
    log(LogLevel::Debug, "  Sample position: (%g, %g, %g)\n", 
        maxDistanceCase.samplePosition.x, maxDistanceCase.samplePosition.y, maxDistanceCase.samplePosition.z);
    log(LogLevel::Debug, "  Closest position: (%g, %g, %g)\n", 
        maxDistanceCase.closestPosition.x, maxDistanceCase.closestPosition.y, maxDistanceCase.closestPosition.z);
    
    if (hasVertexNormals && maxNormalAngleCase.value > 0.0) {
        log(LogLevel::Info, "\nMax Normal Angle: %g degrees\n", maxNormalAngleCase.value);
        log(LogLevel::Debug, "  Sample position: (%g, %g, %g)\n", 
            maxNormalAngleCase.samplePosition.x, maxNormalAngleCase.samplePosition.y, maxNormalAngleCase.samplePosition.z);
        log(LogLevel::Debug, "  Closest position: (%g, %g, %g)\n", 
            maxNormalAngleCase.closestPosition.x, maxNormalAngleCase.closestPosition.y, maxNormalAngleCase.closestPosition.z);
    }
    
    if (hasUVs && maxUVDistanceCase.value > 0.0) {
        log(LogLevel::Info, "\nMax UV Distance: %g\n", maxUVDistanceCase.value);
        log(LogLevel::Debug, "  Sample position: (%g, %g, %g)\n", 
            maxUVDistanceCase.samplePosition.x, maxUVDistanceCase.samplePosition.y, maxUVDistanceCase.samplePosition.z);
        log(LogLevel::Debug, "  Closest position: (%g, %g, %g)\n", 
            maxUVDistanceCase.closestPosition.x, maxUVDistanceCase.closestPosition.y, maxUVDistanceCase.closestPosition.z);
    }
    
    // Export to OBJ file
    std::ofstream out(filename);
    if (!out.is_open()) {
        log(LogLevel::Error, "Failed to open output file: %s\n", filename.c_str());
        return;
    }
    
    out << "# Debug visualization for MeshGeometricDeviation\n";
    out << "# Reference mesh (A), Test mesh (B), and extreme deviation spheres\n";
    out << "# Load in a 3D viewer to inspect problem areas\n\n";
    
    int vertexOffset = 0;
    
    // Export reference mesh A with normals and UVs
    out << "# ===== Reference Mesh A =====\n";
    out << "g reference_mesh\n";
    out << "o mesh_a\n";
    
    // Write vertices
    for (const Vertex& v : meshA.verts) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    
    // Write vertex normals if available
    // Skip any invalid (0,0,0) normals and replace with a default
    if (hasVertexNormals) {
        for (const Vector3& n : meshA.vertexNormals) {
            // Check if normal is valid (not zero length)
            double len = n.length();
            if (len > 1e-10) {
                out << "vn " << n.x << " " << n.y << " " << n.z << "\n";
            } else {
                // Use a default normal for invalid normals
                out << "vn 0 0 1\n";
            }
        }
    }
    
    // Write UVs if available
    if (hasUVs) {
        for (size_t i = 1; i < meshA.uvs.size(); i++) {  // Skip index 0 (default UV)
            const UV& uv = meshA.uvs[i];
            out << "vt " << uv.u << " " << uv.v << "\n";
        }
    }
    
    // Write faces with appropriate format
    int meshAVertCount = static_cast<int>(meshA.verts.size());
    for (const Triangle& tri : meshA.tris) {
        out << "f";
        if (hasUVs && hasVertexNormals) {
            // v/vt/vn format
            out << " " << (tri.i0 + 1) << "/" << tri.t0 << "/" << (tri.i0 + 1);
            out << " " << (tri.i1 + 1) << "/" << tri.t1 << "/" << (tri.i1 + 1);
            out << " " << (tri.i2 + 1) << "/" << tri.t2 << "/" << (tri.i2 + 1);
        } else if (hasVertexNormals) {
            // v//vn format
            out << " " << (tri.i0 + 1) << "//" << (tri.i0 + 1);
            out << " " << (tri.i1 + 1) << "//" << (tri.i1 + 1);
            out << " " << (tri.i2 + 1) << "//" << (tri.i2 + 1);
        } else if (hasUVs) {
            // v/vt format
            out << " " << (tri.i0 + 1) << "/" << tri.t0;
            out << " " << (tri.i1 + 1) << "/" << tri.t1;
            out << " " << (tri.i2 + 1) << "/" << tri.t2;
        } else {
            // v format
            out << " " << (tri.i0 + 1);
            out << " " << (tri.i1 + 1);
            out << " " << (tri.i2 + 1);
        }
        out << "\n";
    }
    vertexOffset = meshAVertCount;
    int normalOffset = hasVertexNormals ? static_cast<int>(meshA.vertexNormals.size()) : 0;
    int uvOffset = hasUVs ? static_cast<int>(meshA.uvs.size() - 1) : 0;
    
    // Export test mesh B with normals and UVs
    out << "\n# ===== Test Mesh B =====\n";
    out << "g test_mesh\n";
    out << "o mesh_b\n";
    
    // Write vertices
    for (const Vertex& v : meshB.verts) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    
    // Write vertex normals if available
    // Skip any invalid (0,0,0) normals and replace with a default
    if (hasVertexNormals) {
        for (const Vector3& n : meshB.vertexNormals) {
            // Check if normal is valid (not zero length)
            double len = n.length();
            if (len > 1e-10) {
                out << "vn " << n.x << " " << n.y << " " << n.z << "\n";
            } else {
                // Use a default normal for invalid normals
                out << "vn 0 0 1\n";
            }
        }
    }
    
    // Write UVs if available
    if (hasUVs) {
        for (size_t i = 1; i < meshB.uvs.size(); i++) {  // Skip index 0 (default UV)
            const UV& uv = meshB.uvs[i];
            out << "vt " << uv.u << " " << uv.v << "\n";
        }
    }
    
    // Write faces with appropriate format
    int meshBVertCount = static_cast<int>(meshB.verts.size());
    for (const Triangle& tri : meshB.tris) {
        out << "f";
        if (hasUVs && hasVertexNormals) {
            // v/vt/vn format
            out << " " << (tri.i0 + 1 + vertexOffset) << "/" << (tri.t0 > 0 ? tri.t0 + uvOffset : 0) << "/" << (tri.i0 + 1 + normalOffset);
            out << " " << (tri.i1 + 1 + vertexOffset) << "/" << (tri.t1 > 0 ? tri.t1 + uvOffset : 0) << "/" << (tri.i1 + 1 + normalOffset);
            out << " " << (tri.i2 + 1 + vertexOffset) << "/" << (tri.t2 > 0 ? tri.t2 + uvOffset : 0) << "/" << (tri.i2 + 1 + normalOffset);
        } else if (hasVertexNormals) {
            // v//vn format
            out << " " << (tri.i0 + 1 + vertexOffset) << "//" << (tri.i0 + 1 + normalOffset);
            out << " " << (tri.i1 + 1 + vertexOffset) << "//" << (tri.i1 + 1 + normalOffset);
            out << " " << (tri.i2 + 1 + vertexOffset) << "//" << (tri.i2 + 1 + normalOffset);
        } else if (hasUVs) {
            // v/vt format
            out << " " << (tri.i0 + 1 + vertexOffset) << "/" << (tri.t0 > 0 ? tri.t0 + uvOffset : 0);
            out << " " << (tri.i1 + 1 + vertexOffset) << "/" << (tri.t1 > 0 ? tri.t1 + uvOffset : 0);
            out << " " << (tri.i2 + 1 + vertexOffset) << "/" << (tri.t2 > 0 ? tri.t2 + uvOffset : 0);
        } else {
            // v format
            out << " " << (tri.i0 + 1 + vertexOffset);
            out << " " << (tri.i1 + 1 + vertexOffset);
            out << " " << (tri.i2 + 1 + vertexOffset);
        }
        out << "\n";
    }
    vertexOffset += meshBVertCount;
    
    // Compute sphere radius based on bounding box size for better scaling
    Vector3 bboxMin(1e10, 1e10, 1e10);
    Vector3 bboxMax(-1e10, -1e10, -1e10);
    for (const Vertex& v : meshA.verts) {
        bboxMin.x = std::min(bboxMin.x, static_cast<double>(v.x));
        bboxMin.y = std::min(bboxMin.y, static_cast<double>(v.y));
        bboxMin.z = std::min(bboxMin.z, static_cast<double>(v.z));
        bboxMax.x = std::max(bboxMax.x, static_cast<double>(v.x));
        bboxMax.y = std::max(bboxMax.y, static_cast<double>(v.y));
        bboxMax.z = std::max(bboxMax.z, static_cast<double>(v.z));
    }
    Vector3 bboxExtent = bboxMax - bboxMin;
    double bboxDiagonal = bboxExtent.length();
    double baseSphereRadius = bboxDiagonal * 0.0005;  // 0.05% of bounding box diagonal
    if (baseSphereRadius < 0.1) baseSphereRadius = 0.1;
    
    // Export max distance spheres (sample point = larger, surface = smaller, separate groups)
    out << "\n# ===== Max Distance Deviation =====\n";
    out << "# Sample point (larger sphere)\n";
    out << "g max_distance_sample\n";
    generateSphere(out, maxDistanceCase.samplePosition, baseSphereRadius * 3.0, vertexOffset, "max_distance_sample");
    
    out << "# Closest surface point (smaller sphere)\n";
    out << "g max_distance_surface\n";
    generateSphere(out, maxDistanceCase.closestPosition, baseSphereRadius, vertexOffset, "max_distance_surface");
    
    if (hasVertexNormals && maxNormalAngleCase.value > 0.0) {
        out << "\n# ===== Max Normal Angle Deviation =====\n";
        out << "# Sample point (larger sphere)\n";
        out << "g max_normal_sample\n";
        generateSphere(out, maxNormalAngleCase.samplePosition, baseSphereRadius * 3.0, vertexOffset, "max_normal_sample");
        
        out << "# Closest surface point (smaller sphere)\n";
        out << "g max_normal_surface\n";
        generateSphere(out, maxNormalAngleCase.closestPosition, baseSphereRadius, vertexOffset, "max_normal_surface");
    }
    
    if (hasUVs && maxUVDistanceCase.value > 0.0) {
        out << "\n# ===== Max UV Distance Deviation =====\n";
        out << "# Sample point (larger sphere)\n";
        out << "g max_uv_sample\n";
        generateSphere(out, maxUVDistanceCase.samplePosition, baseSphereRadius * 3.0, vertexOffset, "max_uv_sample");
        
        out << "# Closest surface point (smaller sphere)\n";
        out << "g max_uv_surface\n";
        generateSphere(out, maxUVDistanceCase.closestPosition, baseSphereRadius, vertexOffset, "max_uv_surface");
    } else if (hasUVs) {
        out << "\n# No valid UV comparisons found (meshes may lack UV data on compared triangles)\n";
    }
    
    out.close();
    
    log(LogLevel::Info, "\nDebug visualization exported to: %s\n", filename.c_str());
    log(LogLevel::Info, "\nExported content:\n");
    log(LogLevel::Info, "  - Reference mesh (A) with normals and UVs\n");
    log(LogLevel::Info, "  - Test mesh (B) with normals and UVs\n");
    log(LogLevel::Info, "  - Max distance deviation markers (%.4g)\n", maxDistanceCase.value);
    if (hasVertexNormals && maxNormalAngleCase.value > 0.0) {
        log(LogLevel::Info, "  - Max normal angle deviation markers (%.2f degrees)\n", maxNormalAngleCase.value);
    }
    if (hasUVs && maxUVDistanceCase.value > 0.0) {
        log(LogLevel::Info, "  - Max UV distance deviation markers (%.4g)\n", maxUVDistanceCase.value);
    }
    log(LogLevel::Info, "\nVisualization shows Direction A->B extreme cases\n");
    log(LogLevel::Info, "Large spheres = sample points on A, small spheres = closest points on B\n");
}

} // namespace MeshGeometricDeviation

