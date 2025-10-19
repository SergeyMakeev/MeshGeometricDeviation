#include "MeshGeometricDeviation/MeshComparison.h"
#include <iostream>

using namespace MeshGeometricDeviation;

int main(int argc, char* argv[]) {
    std::cout << "=== Mesh Deviance Comparison Tool ===" << std::endl;
    
    // Check for --test-export mode
    if (argc == 4 && std::string(argv[1]) == "--test-export") {
        std::cout << "\n=== Test Export Mode ===" << std::endl;
        std::string inputFile = argv[2];
        std::string outputFile = argv[3];
        
        std::cout << "Loading mesh: " << inputFile << std::endl;
        Mesh mesh;
        if (!loadObjFile(inputFile, mesh)) {
            return 1;
        }
        
        // Compute vertex normals if not loaded
        if (mesh.vertexNormals.empty()) {
            std::cout << "Computing vertex normals..." << std::endl;
            computeVertexNormals(mesh);
        }
        
        std::cout << "Exporting mesh: " << outputFile << std::endl;
        if (!exportMeshToObj(outputFile, mesh)) {
            return 1;
        }
        
        std::cout << "\n=== Test Export Complete ===" << std::endl;
        std::cout << "Compare the original and exported files to check for differences." << std::endl;
        return 0;
    }
    
    if (argc < 3) {
        std::cerr << "\nUsage: " << argv[0] << " <reference_mesh.obj> <test_mesh.obj> [sample_density] [max_angle_degrees] [seed] [--debug output.obj]" << std::endl;
        std::cerr << "       " << argv[0] << " --test-export <input.obj> <output.obj>  (test mesh loading/exporting)" << std::endl;
        std::cerr << "\nArguments:" << std::endl;
        std::cerr << "  reference_mesh.obj   - The reference mesh (MeshA)" << std::endl;
        std::cerr << "  test_mesh.obj        - The test mesh to compare against (MeshB)" << std::endl;
        std::cerr << "  sample_density       - Samples per square unit of surface area (default: 20.0)" << std::endl;
        std::cerr << "                         Note: Actual sample count is auto-computed based on mesh surface area" << std::endl;
        std::cerr << "                         Each triangle is guaranteed at least one sample point" << std::endl;
        std::cerr << "  max_angle_degrees    - Max angle for normal matching in degrees (default: 180.0)" << std::endl;
        std::cerr << "                         180.0 effectively disables normal filtering (accepts any orientation)" << std::endl;
        std::cerr << "                         Use smaller values (e.g., 45.0) for stricter normal matching" << std::endl;
        std::cerr << "  seed                 - Random seed for reproducible results (default: 42)" << std::endl;
        std::cerr << "  --debug output.obj   - Export debug visualization with extreme deviation points" << std::endl;
        std::cerr << "\nTest Mode:" << std::endl;
        std::cerr << "  --test-export        - Load a mesh and export it back to test loading/exporting" << std::endl;
        std::cerr << "\nNote: Comparison is always bidirectional, sampling both meshes and comparing in both directions." << std::endl;
        std::cerr << "      This detects missing geometry (holes) and extra geometry." << std::endl;
        return 1;
    }
    
    std::string meshAFile = argv[1];
    std::string meshBFile = argv[2];
    double sampleDensity = (argc > 3) ? std::atof(argv[3]) : 20.0;
    double maxAngleDegrees = (argc > 4) ? std::atof(argv[4]) : 180.0;
    unsigned int seed = (argc > 5) ? static_cast<unsigned int>(std::atoi(argv[5])) : 42;
    
    // Check for debug flag
    std::string debugOutputFile;
    bool exportDebug = false;
    for (int i = 1; i < argc - 1; i++) {
        std::string arg = argv[i];
        if (arg == "--debug" && i + 1 < argc) {
            exportDebug = true;
            debugOutputFile = argv[i + 1];
            break;
        }
    }
    
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
    
    // Compute vertex normals for normal variance analysis (only if not loaded from file)
    std::cout << "\n=== Vertex Normals ===" << std::endl;
    if (meshA.vertexNormals.empty()) {
        std::cout << "Computing vertex normals for reference mesh (not provided in OBJ file)..." << std::endl;
        computeVertexNormals(meshA);
    } else {
        std::cout << "Reference mesh: Using vertex normals from OBJ file" << std::endl;
    }
    
    if (meshB.vertexNormals.empty()) {
        std::cout << "Computing vertex normals for test mesh (not provided in OBJ file)..." << std::endl;
        computeVertexNormals(meshB);
    } else {
        std::cout << "Test mesh: Using vertex normals from OBJ file" << std::endl;
    }
    
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
    
    // Export debug visualization if requested
    if (exportDebug) {
        exportDebugVisualization(debugOutputFile, meshA, meshB, biStats);
    }
    
    std::cout << "\n=== Comparison Complete ===" << std::endl;
    
    return 0;
}

