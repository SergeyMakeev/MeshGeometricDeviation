#include "MeshGeometricDeviation/MeshComparison.h"
#include <iostream>
#include <fstream>
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
    
    std::cout << "Loaded mesh: " << filename << std::endl;
    std::cout << "  Vertices: " << mesh.verts.size() << std::endl;
    std::cout << "  Triangles: " << mesh.tris.size() << std::endl;
    std::cout << "  UVs: " << mesh.uvs.size() - 1 << std::endl;  // -1 to exclude default UV
    
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
    
    // Check if both meshes have UVs for UV variance computation
    bool hasUVs = (meshA.uvs.size() > 1 && meshB.uvs.size() > 1);
    if (hasUVs) {
        std::cout << "Computing UV variance..." << std::endl;
    }
    
    // Measure distances
    std::cout << "Measuring distances..." << std::endl;
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
        
        // Compute UV variance if both meshes have UVs
        if (hasUVs) {
            const Triangle& sampleTri = meshA.tris[sample.triangleIndex];
            const Triangle& closestTri = meshB.tris[result.triangleIndex];
            
            // Check if both triangles have valid UV indices
            if (sampleTri.t0 > 0 && sampleTri.t1 > 0 && sampleTri.t2 > 0 &&
                closestTri.t0 > 0 && closestTri.t1 > 0 && closestTri.t2 > 0) {
                
                // Get sample point's interpolated UV
                const UV& uv0_sample = meshA.uvs[sampleTri.t0];
                const UV& uv1_sample = meshA.uvs[sampleTri.t1];
                const UV& uv2_sample = meshA.uvs[sampleTri.t2];
                
                double sampleU = uv0_sample.u * sample.baryU + 
                                uv1_sample.u * sample.baryV + 
                                uv2_sample.u * sample.baryW;
                double sampleV = uv0_sample.v * sample.baryU + 
                                uv1_sample.v * sample.baryV + 
                                uv2_sample.v * sample.baryW;
                
                // Get closest point's interpolated UV
                const UV& uv0_closest = meshB.uvs[closestTri.t0];
                const UV& uv1_closest = meshB.uvs[closestTri.t1];
                const UV& uv2_closest = meshB.uvs[closestTri.t2];
                
                double closestU = uv0_closest.u * result.baryU + 
                                 uv1_closest.u * result.baryV + 
                                 uv2_closest.u * result.baryW;
                double closestV = uv0_closest.v * result.baryU + 
                                 uv1_closest.v * result.baryV + 
                                 uv2_closest.v * result.baryW;
                
                // Compute UV distance
                double du = sampleU - closestU;
                double dv = sampleV - closestV;
                double uvDistance = std::sqrt(du * du + dv * dv);
                
                uvDistances.push_back(uvDistance);
                
                if (uvDistance > largeUVThreshold) {
                    largeUVDevianceCount++;
                }
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
    
    // Display UV variance if available
    if (biStats.aToB.maxUVDistance > 0.0 || biStats.bToA.maxUVDistance > 0.0) {
        std::cout << "\n=== UV Coordinate Variance (Interpolated) ===" << std::endl;
        
        if (biStats.aToB.maxUVDistance > 0.0) {
            std::cout << "\nReference -> Test (A -> B):" << std::endl;
            std::cout << "  Min UV distance:     " << biStats.aToB.minUVDistance << std::endl;
            std::cout << "  Max UV distance:     " << biStats.aToB.maxUVDistance << std::endl;
            std::cout << "  Average UV distance: " << biStats.aToB.averageUVDistance << std::endl;
            std::cout << "  Median UV distance:  " << biStats.aToB.medianUVDistance << std::endl;
            std::cout << "  Large deviations (>0.1): " << biStats.aToB.largeUVDevianceCount << " samples ("
                      << (100.0 * biStats.aToB.largeUVDevianceCount / biStats.aToB.totalSamples) << "%)" << std::endl;
        }
        
        if (biStats.bToA.maxUVDistance > 0.0) {
            std::cout << "\nTest -> Reference (B -> A):" << std::endl;
            std::cout << "  Min UV distance:     " << biStats.bToA.minUVDistance << std::endl;
            std::cout << "  Max UV distance:     " << biStats.bToA.maxUVDistance << std::endl;
            std::cout << "  Average UV distance: " << biStats.bToA.averageUVDistance << std::endl;
            std::cout << "  Median UV distance:  " << biStats.bToA.medianUVDistance << std::endl;
            std::cout << "  Large deviations (>0.1): " << biStats.bToA.largeUVDevianceCount << " samples ("
                      << (100.0 * biStats.bToA.largeUVDevianceCount / biStats.bToA.totalSamples) << "%)" << std::endl;
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
                              int numSamplesA, int numSamplesB,
                              double maxAngleDegrees,
                              unsigned int baseSeed) {
    
    std::cout << "\n=== Generating Debug Visualization ===" << std::endl;
    
    // Build spatial database for meshB
    SpatialDb spatialDb(meshB);
    
    // Generate samples on meshA
    std::vector<SurfaceSample> samples = generateAreaWeightedSamples(meshA, numSamplesA, baseSeed);
    
    // Track extreme cases
    struct ExtremeCase {
        SurfaceSample sample;
        SpatialDb::ClosestPointResult closestResult;
        double distance;
        double normalAngle;
        double uvDistance;
    };
    
    ExtremeCase maxDistanceCase;
    ExtremeCase maxNormalAngleCase;
    ExtremeCase maxUVDistanceCase;
    
    // Initialize with invalid values
    maxDistanceCase.distance = 0.0;
    maxDistanceCase.sample.position = Vector3(0, 0, 0);
    maxDistanceCase.closestResult.point = Vector3(0, 0, 0);
    
    maxNormalAngleCase.normalAngle = 0.0;
    maxNormalAngleCase.sample.position = Vector3(0, 0, 0);
    maxNormalAngleCase.closestResult.point = Vector3(0, 0, 0);
    
    maxUVDistanceCase.uvDistance = 0.0;
    maxUVDistanceCase.sample.position = Vector3(0, 0, 0);
    maxUVDistanceCase.closestResult.point = Vector3(0, 0, 0);
    
    bool hasVertexNormals = (!meshA.vertexNormals.empty() && !meshB.vertexNormals.empty());
    bool hasUVs = (meshA.uvs.size() > 1 && meshB.uvs.size() > 1);
    
    std::cout << "Analyzing " << numSamplesA << " samples..." << std::endl;
    std::cout << "Has vertex normals: " << (hasVertexNormals ? "Yes" : "No") << std::endl;
    std::cout << "Has UVs: " << (hasUVs ? "Yes" : "No") << std::endl;
    
    for (const SurfaceSample& sample : samples) {
        SpatialDb::ClosestPointResult result = spatialDb.getClosestPointDetailedWithNormal(
            sample.position, sample.normal, maxAngleDegrees);
        
        double distance = (result.point - sample.position).length();
        
        // Track max distance
        if (distance > maxDistanceCase.distance) {
            maxDistanceCase.sample = sample;
            maxDistanceCase.closestResult = result;
            maxDistanceCase.distance = distance;
        }
        
        // Compute normal angle if available
        if (hasVertexNormals) {
            const Triangle& sampleTri = meshA.tris[sample.triangleIndex];
            Vector3 sampleVertexNormal = 
                meshA.vertexNormals[sampleTri.i0] * sample.baryU +
                meshA.vertexNormals[sampleTri.i1] * sample.baryV +
                meshA.vertexNormals[sampleTri.i2] * sample.baryW;
            sampleVertexNormal = sampleVertexNormal.normalized();
            
            const Triangle& closestTri = meshB.tris[result.triangleIndex];
            Vector3 closestVertexNormal = 
                meshB.vertexNormals[closestTri.i0] * result.baryU +
                meshB.vertexNormals[closestTri.i1] * result.baryV +
                meshB.vertexNormals[closestTri.i2] * result.baryW;
            closestVertexNormal = closestVertexNormal.normalized();
            
            double dotProduct = sampleVertexNormal.dot(closestVertexNormal);
            dotProduct = std::max(-1.0, std::min(1.0, dotProduct));
            double angleRadians = std::acos(dotProduct);
            double angleDegrees = angleRadians * 180.0 / 3.14159265358979323846;
            
            if (angleDegrees > maxNormalAngleCase.normalAngle) {
                maxNormalAngleCase.sample = sample;
                maxNormalAngleCase.closestResult = result;
                maxNormalAngleCase.distance = distance;
                maxNormalAngleCase.normalAngle = angleDegrees;
            }
        }
        
        // Compute UV distance if available
        if (hasUVs) {
            const Triangle& sampleTri = meshA.tris[sample.triangleIndex];
            const Triangle& closestTri = meshB.tris[result.triangleIndex];
            
            if (sampleTri.t0 > 0 && sampleTri.t1 > 0 && sampleTri.t2 > 0 &&
                closestTri.t0 > 0 && closestTri.t1 > 0 && closestTri.t2 > 0) {
                
                const UV& uv0_sample = meshA.uvs[sampleTri.t0];
                const UV& uv1_sample = meshA.uvs[sampleTri.t1];
                const UV& uv2_sample = meshA.uvs[sampleTri.t2];
                
                double sampleU = uv0_sample.u * sample.baryU + 
                                uv1_sample.u * sample.baryV + 
                                uv2_sample.u * sample.baryW;
                double sampleV = uv0_sample.v * sample.baryU + 
                                uv1_sample.v * sample.baryV + 
                                uv2_sample.v * sample.baryW;
                
                const UV& uv0_closest = meshB.uvs[closestTri.t0];
                const UV& uv1_closest = meshB.uvs[closestTri.t1];
                const UV& uv2_closest = meshB.uvs[closestTri.t2];
                
                double closestU = uv0_closest.u * result.baryU + 
                                 uv1_closest.u * result.baryV + 
                                 uv2_closest.u * result.baryW;
                double closestV = uv0_closest.v * result.baryU + 
                                 uv1_closest.v * result.baryV + 
                                 uv2_closest.v * result.baryW;
                
                double du = sampleU - closestU;
                double dv = sampleV - closestV;
                double uvDistance = std::sqrt(du * du + dv * dv);
                
                if (uvDistance > maxUVDistanceCase.uvDistance) {
                    maxUVDistanceCase.sample = sample;
                    maxUVDistanceCase.closestResult = result;
                    maxUVDistanceCase.distance = distance;
                    maxUVDistanceCase.uvDistance = uvDistance;
                }
            }
        }
    }
    
    // Print diagnostic information
    std::cout << "\n=== Diagnostic Information ===" << std::endl;
    
    std::cout << "\nMax Distance Case:" << std::endl;
    std::cout << "  Distance: " << maxDistanceCase.distance << std::endl;
    std::cout << "  Sample position: (" << maxDistanceCase.sample.position.x << ", " 
              << maxDistanceCase.sample.position.y << ", " << maxDistanceCase.sample.position.z << ")" << std::endl;
    std::cout << "  Closest position: (" << maxDistanceCase.closestResult.point.x << ", " 
              << maxDistanceCase.closestResult.point.y << ", " << maxDistanceCase.closestResult.point.z << ")" << std::endl;
    std::cout << "  Sample triangle: " << maxDistanceCase.sample.triangleIndex << std::endl;
    std::cout << "  Closest triangle: " << maxDistanceCase.closestResult.triangleIndex << std::endl;
    
    if (hasVertexNormals) {
        std::cout << "\nMax Normal Angle Case:" << std::endl;
        std::cout << "  Normal angle: " << maxNormalAngleCase.normalAngle << " degrees" << std::endl;
        std::cout << "  Distance: " << maxNormalAngleCase.distance << std::endl;
        std::cout << "  Sample position: (" << maxNormalAngleCase.sample.position.x << ", " 
                  << maxNormalAngleCase.sample.position.y << ", " << maxNormalAngleCase.sample.position.z << ")" << std::endl;
        std::cout << "  Closest position: (" << maxNormalAngleCase.closestResult.point.x << ", " 
                  << maxNormalAngleCase.closestResult.point.y << ", " << maxNormalAngleCase.closestResult.point.z << ")" << std::endl;
        std::cout << "  Sample triangle: " << maxNormalAngleCase.sample.triangleIndex << std::endl;
        std::cout << "  Closest triangle: " << maxNormalAngleCase.closestResult.triangleIndex << std::endl;
        
        // Get normals at sample point
        const Triangle& sampleTri = meshA.tris[maxNormalAngleCase.sample.triangleIndex];
        Vector3 sampleNormal = 
            meshA.vertexNormals[sampleTri.i0] * maxNormalAngleCase.sample.baryU +
            meshA.vertexNormals[sampleTri.i1] * maxNormalAngleCase.sample.baryV +
            meshA.vertexNormals[sampleTri.i2] * maxNormalAngleCase.sample.baryW;
        sampleNormal = sampleNormal.normalized();
        
        const Triangle& closestTri = meshB.tris[maxNormalAngleCase.closestResult.triangleIndex];
        Vector3 closestNormal = 
            meshB.vertexNormals[closestTri.i0] * maxNormalAngleCase.closestResult.baryU +
            meshB.vertexNormals[closestTri.i1] * maxNormalAngleCase.closestResult.baryV +
            meshB.vertexNormals[closestTri.i2] * maxNormalAngleCase.closestResult.baryW;
        closestNormal = closestNormal.normalized();
        
        std::cout << "  Sample normal: (" << sampleNormal.x << ", " << sampleNormal.y << ", " << sampleNormal.z << ")" << std::endl;
        std::cout << "  Closest normal: (" << closestNormal.x << ", " << closestNormal.y << ", " << closestNormal.z << ")" << std::endl;
    }
    
    if (hasUVs && maxUVDistanceCase.uvDistance > 0.0) {
        std::cout << "\nMax UV Distance Case:" << std::endl;
        std::cout << "  UV distance: " << maxUVDistanceCase.uvDistance << std::endl;
        std::cout << "  Position distance: " << maxUVDistanceCase.distance << std::endl;
        std::cout << "  Sample position: (" << maxUVDistanceCase.sample.position.x << ", " 
                  << maxUVDistanceCase.sample.position.y << ", " << maxUVDistanceCase.sample.position.z << ")" << std::endl;
        std::cout << "  Closest position: (" << maxUVDistanceCase.closestResult.point.x << ", " 
                  << maxUVDistanceCase.closestResult.point.y << ", " << maxUVDistanceCase.closestResult.point.z << ")" << std::endl;
        std::cout << "  Sample triangle: " << maxUVDistanceCase.sample.triangleIndex << std::endl;
        std::cout << "  Closest triangle: " << maxUVDistanceCase.closestResult.triangleIndex << std::endl;
        
        // Get UVs
        const Triangle& sampleTri = meshA.tris[maxUVDistanceCase.sample.triangleIndex];
        const UV& uv0_sample = meshA.uvs[sampleTri.t0];
        const UV& uv1_sample = meshA.uvs[sampleTri.t1];
        const UV& uv2_sample = meshA.uvs[sampleTri.t2];
        
        double sampleU = uv0_sample.u * maxUVDistanceCase.sample.baryU + 
                        uv1_sample.u * maxUVDistanceCase.sample.baryV + 
                        uv2_sample.u * maxUVDistanceCase.sample.baryW;
        double sampleV = uv0_sample.v * maxUVDistanceCase.sample.baryU + 
                        uv1_sample.v * maxUVDistanceCase.sample.baryV + 
                        uv2_sample.v * maxUVDistanceCase.sample.baryW;
        
        const Triangle& closestTri = meshB.tris[maxUVDistanceCase.closestResult.triangleIndex];
        const UV& uv0_closest = meshB.uvs[closestTri.t0];
        const UV& uv1_closest = meshB.uvs[closestTri.t1];
        const UV& uv2_closest = meshB.uvs[closestTri.t2];
        
        double closestU = uv0_closest.u * maxUVDistanceCase.closestResult.baryU + 
                         uv1_closest.u * maxUVDistanceCase.closestResult.baryV + 
                         uv2_closest.u * maxUVDistanceCase.closestResult.baryW;
        double closestV = uv0_closest.v * maxUVDistanceCase.closestResult.baryU + 
                         uv1_closest.v * maxUVDistanceCase.closestResult.baryV + 
                         uv2_closest.v * maxUVDistanceCase.closestResult.baryW;
        
        std::cout << "  Sample UV: (" << sampleU << ", " << sampleV << ")" << std::endl;
        std::cout << "  Closest UV: (" << closestU << ", " << closestV << ")" << std::endl;
        std::cout << "  Sample triangle UVs:" << std::endl;
        std::cout << "    v0: (" << uv0_sample.u << ", " << uv0_sample.v << ")" << std::endl;
        std::cout << "    v1: (" << uv1_sample.u << ", " << uv1_sample.v << ")" << std::endl;
        std::cout << "    v2: (" << uv2_sample.u << ", " << uv2_sample.v << ")" << std::endl;
        std::cout << "  Closest triangle UVs:" << std::endl;
        std::cout << "    v0: (" << uv0_closest.u << ", " << uv0_closest.v << ")" << std::endl;
        std::cout << "    v1: (" << uv1_closest.u << ", " << uv1_closest.v << ")" << std::endl;
        std::cout << "    v2: (" << uv2_closest.u << ", " << uv2_closest.v << ")" << std::endl;
        std::cout << "  Barycentric coords (sample): (" << maxUVDistanceCase.sample.baryU << ", " 
                  << maxUVDistanceCase.sample.baryV << ", " << maxUVDistanceCase.sample.baryW << ")" << std::endl;
        std::cout << "  Barycentric coords (closest): (" << maxUVDistanceCase.closestResult.baryU << ", " 
                  << maxUVDistanceCase.closestResult.baryV << ", " << maxUVDistanceCase.closestResult.baryW << ")" << std::endl;
    } else if (hasUVs) {
        std::cout << "\nMax UV Distance Case: NOT FOUND" << std::endl;
        std::cout << "  (No valid UV comparisons - one or both meshes lack UVs on compared triangles)" << std::endl;
    }
    
    // Export to OBJ file
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }
    
    out << "# Debug visualization for MeshGeometricDeviation\n";
    out << "# Reference mesh (A), Test mesh (B), and extreme deviation spheres\n";
    out << "# Load in a 3D viewer to inspect problem areas\n\n";
    
    int vertexOffset = 0;
    
    // Export reference mesh A
    out << "# ===== Reference Mesh A =====\n";
    out << "g reference_mesh\n";
    out << "o mesh_a\n";
    for (const Vertex& v : meshA.verts) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    int meshAVertCount = static_cast<int>(meshA.verts.size());
    for (const Triangle& tri : meshA.tris) {
        out << "f " << (tri.i0 + 1) << " " << (tri.i1 + 1) << " " << (tri.i2 + 1) << "\n";
    }
    vertexOffset = meshAVertCount;
    
    // Export test mesh B
    out << "\n# ===== Test Mesh B =====\n";
    out << "g test_mesh\n";
    out << "o mesh_b\n";
    for (const Vertex& v : meshB.verts) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    int meshBVertCount = static_cast<int>(meshB.verts.size());
    for (const Triangle& tri : meshB.tris) {
        out << "f " << (tri.i0 + 1 + vertexOffset) << " " 
            << (tri.i1 + 1 + vertexOffset) << " " 
            << (tri.i2 + 1 + vertexOffset) << "\n";
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
    double baseSphereRadius = bboxDiagonal * 0.005;  // 0.5% of bounding box diagonal
    if (baseSphereRadius < 0.1) baseSphereRadius = 0.1;
    
    // Export max distance spheres (sample point = larger, surface = smaller, separate groups)
    out << "\n# ===== Max Distance Deviation =====\n";
    out << "# Sample point (larger sphere)\n";
    out << "g max_distance_sample\n";
    generateSphere(out, maxDistanceCase.sample.position, baseSphereRadius * 3.0, vertexOffset, "max_distance_sample");
    
    out << "# Closest surface point (smaller sphere)\n";
    out << "g max_distance_surface\n";
    generateSphere(out, maxDistanceCase.closestResult.point, baseSphereRadius, vertexOffset, "max_distance_surface");
    
    if (hasVertexNormals) {
        out << "\n# ===== Max Normal Angle Deviation =====\n";
        out << "# Sample point (larger sphere)\n";
        out << "g max_normal_sample\n";
        generateSphere(out, maxNormalAngleCase.sample.position, baseSphereRadius * 3.0, vertexOffset, "max_normal_sample");
        
        out << "# Closest surface point (smaller sphere)\n";
        out << "g max_normal_surface\n";
        generateSphere(out, maxNormalAngleCase.closestResult.point, baseSphereRadius, vertexOffset, "max_normal_surface");
    }
    
    if (hasUVs && maxUVDistanceCase.uvDistance > 0.0) {
        out << "\n# ===== Max UV Distance Deviation =====\n";
        out << "# Sample point (larger sphere)\n";
        out << "g max_uv_sample\n";
        generateSphere(out, maxUVDistanceCase.sample.position, baseSphereRadius * 3.0, vertexOffset, "max_uv_sample");
        
        out << "# Closest surface point (smaller sphere)\n";
        out << "g max_uv_surface\n";
        generateSphere(out, maxUVDistanceCase.closestResult.point, baseSphereRadius, vertexOffset, "max_uv_surface");
    } else if (hasUVs) {
        out << "\n# No valid UV comparisons found (meshes may lack UV data on compared triangles)\n";
    }
    
    out.close();
    
    std::cout << "\nDebug visualization exported to: " << filename << std::endl;
    std::cout << "\nObjects in file:" << std::endl;
    std::cout << "  Groups (can be toggled in 3D viewer):" << std::endl;
    std::cout << "    - reference_mesh (mesh_a)" << std::endl;
    std::cout << "    - test_mesh (mesh_b)" << std::endl;
    std::cout << "    - max_distance_sample (LARGE sphere = sample point on A)" << std::endl;
    std::cout << "    - max_distance_surface (small sphere = closest point on B)" << std::endl;
    if (hasVertexNormals) {
        std::cout << "    - max_normal_sample (LARGE sphere = sample point on A)" << std::endl;
        std::cout << "    - max_normal_surface (small sphere = closest point on B)" << std::endl;
    }
    if (hasUVs && maxUVDistanceCase.uvDistance > 0.0) {
        std::cout << "    - max_uv_sample (LARGE sphere = sample point on A)" << std::endl;
        std::cout << "    - max_uv_surface (small sphere = closest point on B)" << std::endl;
    } else if (hasUVs) {
        std::cout << "    - [UV spheres NOT created - no valid UV comparisons found]" << std::endl;
    }
    std::cout << "\n  Sphere sizes:" << std::endl;
    std::cout << "    - Sample sphere (LARGE = 3x base): " << (baseSphereRadius * 3.0) << " units" << std::endl;
    std::cout << "    - Surface sphere (small = 1x base): " << baseSphereRadius << " units" << std::endl;
    std::cout << "    - Base radius (0.5% of bbox diagonal): " << baseSphereRadius << " units" << std::endl;
    std::cout << "\n  IMPORTANT:" << std::endl;
    std::cout << "    - This shows Direction 1 (A->B): samples from '" << meshA.name << "'" << std::endl;
    std::cout << "    - To detect features in mesh B (spikes/protrusions), swap inputs:" << std::endl;
    std::cout << "      mesh_compare <mesh_with_spike> <reference> ... --debug out.obj" << std::endl;
}

} // namespace MeshGeometricDeviation

