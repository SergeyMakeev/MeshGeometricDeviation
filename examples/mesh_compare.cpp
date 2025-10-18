#include "MeshGeometricDeviation/MeshComparison.h"
#include <iostream>

using namespace MeshGeometricDeviation;

int main(int argc, char* argv[]) {
    std::cout << "=== Mesh Deviance Comparison Tool ===" << std::endl;
    
    if (argc < 3) {
        std::cerr << "\nUsage: " << argv[0] << " <reference_mesh.obj> <test_mesh.obj> [sample_density] [max_angle_degrees] [seed]" << std::endl;
        std::cerr << "\nArguments:" << std::endl;
        std::cerr << "  reference_mesh.obj   - The reference mesh (MeshA)" << std::endl;
        std::cerr << "  test_mesh.obj        - The test mesh to compare against (MeshB)" << std::endl;
        std::cerr << "  sample_density       - Samples per square unit of surface area (default: 20.0)" << std::endl;
        std::cerr << "                         Note: Actual sample count is auto-computed based on mesh surface area" << std::endl;
        std::cerr << "                         Each triangle is guaranteed at least one sample point" << std::endl;
        std::cerr << "  max_angle_degrees    - Max angle for normal matching in degrees (default: 45.0)" << std::endl;
        std::cerr << "  seed                 - Random seed for reproducible results (default: 42)" << std::endl;
        std::cerr << "\nNote: Comparison is always bidirectional, sampling both meshes and comparing in both directions." << std::endl;
        std::cerr << "      This detects missing geometry (holes) and extra geometry." << std::endl;
        return 1;
    }
    
    std::string meshAFile = argv[1];
    std::string meshBFile = argv[2];
    double sampleDensity = (argc > 3) ? std::atof(argv[3]) : 20.0;
    double maxAngleDegrees = (argc > 4) ? std::atof(argv[4]) : 45.0;
    unsigned int seed = (argc > 5) ? static_cast<unsigned int>(std::atoi(argv[5])) : 42;
    
    // Validate parameters
    if (sampleDensity <= 0.0) {
        std::cerr << "Error: sample_density must be positive" << std::endl;
        return 1;
    }
    if (maxAngleDegrees <= 0.0 || maxAngleDegrees > 180.0) {
        std::cerr << "Error: max_angle_degrees must be between 0 and 180" << std::endl;
        return 1;
    }
    
    // Load meshes
    std::cout << "\n=== Loading Meshes ===" << std::endl;
    Mesh meshA, meshB;
    
    if (!loadObjFile(meshAFile, meshA)) {
        return 1;
    }
    
    if (!loadObjFile(meshBFile, meshB)) {
        return 1;
    }
    
    if (meshA.tris.empty() || meshB.tris.empty()) {
        std::cerr << "Error: One or both meshes have no triangles" << std::endl;
        return 1;
    }
    
    // Compute vertex normals for normal variance analysis
    std::cout << "\n=== Computing Vertex Normals ===" << std::endl;
    computeVertexNormals(meshA);
    computeVertexNormals(meshB);
    std::cout << "Vertex normals computed for both meshes (for normal variance analysis)" << std::endl;
    
    // Compute number of samples for each mesh based on surface area and density
    std::cout << "\n=== Computing Sample Counts ===" << std::endl;
    std::cout << "Sample density: " << sampleDensity << " samples per square unit" << std::endl;
    
    double areaA = computeMeshSurfaceArea(meshA);
    double areaB = computeMeshSurfaceArea(meshB);
    
    std::cout << "\nReference mesh (A) - " << meshA.name << ":" << std::endl;
    std::cout << "  Surface area: " << areaA << " square units" << std::endl;
    std::cout << "  Triangle count: " << meshA.tris.size() << std::endl;
    
    std::cout << "\nTest mesh (B) - " << meshB.name << ":" << std::endl;
    std::cout << "  Surface area: " << areaB << " square units" << std::endl;
    std::cout << "  Triangle count: " << meshB.tris.size() << std::endl;
    
    int numSamplesA = computeNumSamples(meshA, sampleDensity);
    int numSamplesB = computeNumSamples(meshB, sampleDensity);
    
    std::cout << "\nComputed sample counts:" << std::endl;
    std::cout << "  Reference mesh (A): " << numSamplesA << " samples" << std::endl;
    std::cout << "  Test mesh (B): " << numSamplesB << " samples" << std::endl;
    std::cout << "  Random seed: " << seed << " (for reproducibility)" << std::endl;
    
    // Compare meshes (always bidirectional)
    BidirectionalDevianceStats biStats = compareMeshesBidirectional(meshA, meshB, numSamplesA, numSamplesB, maxAngleDegrees, true, true, seed);
    printBidirectionalStats(biStats);
    
    std::cout << "\n=== Comparison Complete ===" << std::endl;
    
    return 0;
}

