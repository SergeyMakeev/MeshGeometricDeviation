#include "MeshGeometricDeviation/MeshComparison.h"
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

#define FAST_OBJ_IMPLEMENTATION
#include "fast_obj.h"

namespace MeshGeometricDeviation {

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

// Generate area-weighted random samples
// Ensures each triangle gets at least one sample
static std::vector<SurfaceSample> generateAreaWeightedSamples(const Mesh& mesh, int numSamples, unsigned int seed) {
    std::vector<SurfaceSample> samples;
    samples.reserve(numSamples);
    
    // Compute individual triangle areas
    std::vector<double> triangleAreas;
    triangleAreas.reserve(mesh.tris.size());
    
    double totalArea = 0.0;
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
        std::cerr << "Failed to load OBJ file: " << filename << std::endl;
        return false;
    }

    mesh.name = filename;
    
    // Reserve space
    mesh.verts.reserve(obj->position_count);
    
    // Load vertices
    for (unsigned int i = 0; i < obj->position_count; i++) {
        Vertex v;
        v.x = obj->positions[3 * i + 0];
        v.y = obj->positions[3 * i + 1];
        v.z = obj->positions[3 * i + 2];
        mesh.verts.push_back(v);
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
            mesh.tris.push_back(tri);
        } else if (vertCount > 3) {
            // Triangulate polygon (simple fan triangulation)
            for (unsigned int i = 2; i < vertCount; i++) {
                Triangle tri;
                tri.i0 = obj->indices[faceVertexOffset + 0].p;
                tri.i1 = obj->indices[faceVertexOffset + i - 1].p;
                tri.i2 = obj->indices[faceVertexOffset + i].p;
                mesh.tris.push_back(tri);
            }
        }
        
        faceVertexOffset += vertCount;
    }
    
    fast_obj_destroy(obj);
    
    std::cout << "Loaded mesh: " << filename << std::endl;
    std::cout << "  Vertices: " << mesh.verts.size() << std::endl;
    std::cout << "  Triangles: " << mesh.tris.size() << std::endl;
    
    return true;
}

// Compute total surface area of mesh
double computeMeshSurfaceArea(const Mesh& mesh) {
    double totalArea = 0.0;
    for (const Triangle& tri : mesh.tris) {
        const Vector3 v0 = vertexToVector3(mesh.verts[tri.i0]);
        const Vector3 v1 = vertexToVector3(mesh.verts[tri.i1]);
        const Vector3 v2 = vertexToVector3(mesh.verts[tri.i2]);
        
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        double area = edge1.cross(edge2).length() * 0.5;
        
        totalArea += area;
    }
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
    std::cout << "\n=== Comparing Meshes ===" << std::endl;
    std::cout << "Reference mesh: " << meshA.name << " (" << meshA.tris.size() << " triangles)" << std::endl;
    std::cout << "Test mesh: " << meshB.name << " (" << meshB.tris.size() << " triangles)" << std::endl;
    std::cout << "Samples: " << numSamples << std::endl;
    std::cout << "Normal filtering: " << (useNormalFiltering ? "enabled" : "disabled") << std::endl;
    if (useNormalFiltering) {
        std::cout << "Normal angle threshold: " << maxAngleDegrees << " degrees" << std::endl;
    }
    std::cout << "Area weighting: " << (useAreaWeighting ? "enabled" : "disabled") << std::endl;
    std::cout << "Random seed: " << seed << " (for reproducibility)" << std::endl;
    
    // Build spatial database for meshB
    std::cout << "\nBuilding spatial database for test mesh..." << std::endl;
    SpatialDb spatialDb(meshB);
    
    auto stats = spatialDb.getStats();
    std::cout << "Spatial database stats:" << std::endl;
    std::cout << "  Total nodes: " << stats.totalNodes << std::endl;
    std::cout << "  Leaf nodes: " << stats.leafNodes << std::endl;
    std::cout << "  Max depth: " << stats.maxDepth << std::endl;
    std::cout << "  Avg triangles per leaf: " << stats.avgTrisPerLeaf << std::endl;
    
    // Generate samples on meshA
    std::cout << "\nGenerating surface samples on reference mesh..." << std::endl;
    std::vector<SurfaceSample> samples = generateAreaWeightedSamples(meshA, numSamples, seed);
    
    // Check if both meshes have vertex normals for normal variance computation
    bool hasVertexNormals = (!meshA.vertexNormals.empty() && !meshB.vertexNormals.empty());
    if (hasVertexNormals) {
        std::cout << "Computing vertex normal variance..." << std::endl;
    }
    
    // Measure distances
    std::cout << "Measuring distances..." << std::endl;
    std::vector<double> distances;
    std::vector<double> normalAngles;  // Angles between interpolated vertex normals (degrees)
    distances.reserve(numSamples);
    if (hasVertexNormals) {
        normalAngles.reserve(numSamples);
    }
    
    int normalMatchedCount = 0;
    int fallbackCount = 0;
    int largeDevianceCount = 0;
    int largeNormalDevianceCount = 0;
    const double largeDevianceThreshold = 1e-4; // 0.0001 units
    const double largeNormalAngleThreshold = 15.0; // 15 degrees
    
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
        
        // Compute normal variance if both meshes have vertex normals
        if (hasVertexNormals) {
            // Get sample point's interpolated vertex normal
            const Triangle& sampleTri = meshA.tris[sample.triangleIndex];
            Vector3 sampleVertexNormal = 
                meshA.vertexNormals[sampleTri.i0] * sample.baryU +
                meshA.vertexNormals[sampleTri.i1] * sample.baryV +
                meshA.vertexNormals[sampleTri.i2] * sample.baryW;
            sampleVertexNormal = sampleVertexNormal.normalized();
            
            // Get closest point's interpolated vertex normal
            const Triangle& closestTri = meshB.tris[result.triangleIndex];
            Vector3 closestVertexNormal = 
                meshB.vertexNormals[closestTri.i0] * result.baryU +
                meshB.vertexNormals[closestTri.i1] * result.baryV +
                meshB.vertexNormals[closestTri.i2] * result.baryW;
            closestVertexNormal = closestVertexNormal.normalized();
            
            // Compute angle between normals
            double dotProduct = sampleVertexNormal.dot(closestVertexNormal);
            dotProduct = std::max(-1.0, std::min(1.0, dotProduct));  // Clamp to [-1, 1] for numerical stability
            double angleRadians = std::acos(dotProduct);
            double angleDegrees = angleRadians * 180.0 / 3.14159265358979323846;
            
            normalAngles.push_back(angleDegrees);
            
            if (angleDegrees > largeNormalAngleThreshold) {
                largeNormalDevianceCount++;
            }
        }
    }
    
    // Compute statistics
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
    
    return devianceStats;
}

// Bidirectional mesh comparison
BidirectionalDevianceStats compareMeshesBidirectional(const Mesh& meshA, const Mesh& meshB, 
                                                       int numSamplesA, int numSamplesB,
                                                       double maxAngleDegrees,
                                                       bool useAreaWeighting, bool useNormalFiltering,
                                                       unsigned int baseSeed) {
    BidirectionalDevianceStats biStats;
    
    std::cout << "\n=== Bidirectional Mesh Comparison ===" << std::endl;
    std::cout << "This compares in both directions to detect missing/extra geometry" << std::endl;
    
    // Direction 1: A -> B (reference to test)
    std::cout << "\n--- Direction 1: Reference (A) -> Test (B) ---" << std::endl;
    biStats.aToB = compareMeshes(meshA, meshB, numSamplesA, maxAngleDegrees, useAreaWeighting, useNormalFiltering, baseSeed);
    
    // Direction 2: B -> A (test to reference)
    std::cout << "\n--- Direction 2: Test (B) -> Reference (A) ---" << std::endl;
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
    std::cout << "\n=== Deviance Statistics ===" << std::endl;
    std::cout << "Min deviance:     " << stats.minDeviance << std::endl;
    std::cout << "Max deviance:     " << stats.maxDeviance << std::endl;
    std::cout << "Average deviance: " << stats.averageDeviance << std::endl;
    std::cout << "Median deviance:  " << stats.medianDeviance << std::endl;
    std::cout << "RMSD:             " << stats.rmsd << std::endl;
    
    if (showNormalStats && stats.fallbackCount >= 0) {
        std::cout << "\nNormal constraint:" << std::endl;
        std::cout << "  Matched:  " << stats.normalMatchedCount << " samples (" 
                  << (100.0f * stats.normalMatchedCount / stats.totalSamples) << "%)" << std::endl;
        std::cout << "  Fallback: " << stats.fallbackCount << " samples (" 
                  << (100.0f * stats.fallbackCount / stats.totalSamples) << "%)" << std::endl;
    }
    
    std::cout << "\nPrecision analysis:" << std::endl;
    std::cout << "  Large deviations (>0.0001): " << stats.largeDevianceCount << " samples (" 
              << (100.0f * stats.largeDevianceCount / stats.totalSamples) << "%)" << std::endl;
    
    // Display normal variance if available
    if (stats.maxNormalAngleDeg > 0.0) {
        std::cout << "\nVertex Normal Variance (interpolated):" << std::endl;
        std::cout << "  Min angle:     " << stats.minNormalAngleDeg << " degrees" << std::endl;
        std::cout << "  Max angle:     " << stats.maxNormalAngleDeg << " degrees" << std::endl;
        std::cout << "  Average angle: " << stats.averageNormalAngleDeg << " degrees" << std::endl;
        std::cout << "  Median angle:  " << stats.medianNormalAngleDeg << " degrees" << std::endl;
        std::cout << "  Large normal deviations (>15 deg): " << stats.largeNormalDevianceCount << " samples (" 
                  << (100.0f * stats.largeNormalDevianceCount / stats.totalSamples) << "%)" << std::endl;
    }
    
    // Warn about self-comparison issues
    if (stats.averageDeviance > 1e-12 && stats.fallbackCount == 0) {
        std::cout << "\nNote: If comparing identical meshes, non-zero deviance indicates numerical" << std::endl;
        std::cout << "      precision issues in the spatial query. This is expected due to floating-" << std::endl;
        std::cout << "      point arithmetic, especially near edges/vertices where multiple triangles meet." << std::endl;
    }
}

// Print bidirectional comparison statistics
void printBidirectionalStats(const BidirectionalDevianceStats& biStats) {
    std::cout << "\n=== Bidirectional Comparison Results ===" << std::endl;
    std::cout << "\nReference -> Test (A -> B):" << std::endl;
    std::cout << "  Min deviance:     " << biStats.aToB.minDeviance << std::endl;
    std::cout << "  Max deviance:     " << biStats.aToB.maxDeviance << std::endl;
    std::cout << "  Average deviance: " << biStats.aToB.averageDeviance << std::endl;
    std::cout << "  Median deviance:  " << biStats.aToB.medianDeviance << std::endl;
    std::cout << "  RMSD:             " << biStats.aToB.rmsd << std::endl;
    
    std::cout << "\nTest -> Reference (B -> A):" << std::endl;
    std::cout << "  Min deviance:     " << biStats.bToA.minDeviance << std::endl;
    std::cout << "  Max deviance:     " << biStats.bToA.maxDeviance << std::endl;
    std::cout << "  Average deviance: " << biStats.bToA.averageDeviance << std::endl;
    std::cout << "  Median deviance:  " << biStats.bToA.medianDeviance << std::endl;
    std::cout << "  RMSD:             " << biStats.bToA.rmsd << std::endl;
    
    std::cout << "\nOverall (Symmetric Hausdorff-like):" << std::endl;
    std::cout << "  Min deviance:     " << biStats.minDeviance << std::endl;
    std::cout << "  Max deviance:     " << biStats.maxDeviance << std::endl;
    std::cout << "  Average deviance: " << biStats.averageDeviance << std::endl;
    std::cout << "  RMSD:             " << biStats.rmsd << std::endl;
    
    std::cout << "\nAsymmetry Analysis:" << std::endl;
    if (biStats.isAsymmetric) {
        std::cout << "  Status: ASYMMETRIC (ratio: " << biStats.asymmetryRatio << "x)" << std::endl;
        if (biStats.aToB.maxDeviance > biStats.bToA.maxDeviance * 1.5) {
            std::cout << "  -> Reference mesh has points far from test mesh" << std::endl;
            std::cout << "     (Test mesh may have missing geometry/holes)" << std::endl;
        } else if (biStats.bToA.maxDeviance > biStats.aToB.maxDeviance * 1.5) {
            std::cout << "  -> Test mesh has points far from reference mesh" << std::endl;
            std::cout << "     (Test mesh may have extra geometry)" << std::endl;
        }
    } else {
        std::cout << "  Status: Symmetric (ratio: " << biStats.asymmetryRatio << "x)" << std::endl;
        std::cout << "  -> Deviations are similar in both directions" << std::endl;
    }
    
    // Detail on large deviations
    std::cout << "\nLarge deviations (>0.0001) per direction:" << std::endl;
    std::cout << "  A -> B: " << biStats.aToB.largeDevianceCount << " samples ("
              << (100.0 * biStats.aToB.largeDevianceCount / biStats.aToB.totalSamples) << "%)" << std::endl;
    std::cout << "  B -> A: " << biStats.bToA.largeDevianceCount << " samples ("
              << (100.0 * biStats.bToA.largeDevianceCount / biStats.bToA.totalSamples) << "%)" << std::endl;
    
    // Display normal variance if available
    if (biStats.aToB.maxNormalAngleDeg > 0.0 || biStats.bToA.maxNormalAngleDeg > 0.0) {
        std::cout << "\n=== Vertex Normal Variance (Interpolated) ===" << std::endl;
        
        if (biStats.aToB.maxNormalAngleDeg > 0.0) {
            std::cout << "\nReference -> Test (A -> B):" << std::endl;
            std::cout << "  Min angle:     " << biStats.aToB.minNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Max angle:     " << biStats.aToB.maxNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Average angle: " << biStats.aToB.averageNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Median angle:  " << biStats.aToB.medianNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Large deviations (>15 deg): " << biStats.aToB.largeNormalDevianceCount << " samples ("
                      << (100.0 * biStats.aToB.largeNormalDevianceCount / biStats.aToB.totalSamples) << "%)" << std::endl;
        }
        
        if (biStats.bToA.maxNormalAngleDeg > 0.0) {
            std::cout << "\nTest -> Reference (B -> A):" << std::endl;
            std::cout << "  Min angle:     " << biStats.bToA.minNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Max angle:     " << biStats.bToA.maxNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Average angle: " << biStats.bToA.averageNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Median angle:  " << biStats.bToA.medianNormalAngleDeg << " degrees" << std::endl;
            std::cout << "  Large deviations (>15 deg): " << biStats.bToA.largeNormalDevianceCount << " samples ("
                      << (100.0 * biStats.bToA.largeNormalDevianceCount / biStats.bToA.totalSamples) << "%)" << std::endl;
        }
    }
}

} // namespace MeshGeometricDeviation

