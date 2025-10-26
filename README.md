# MeshGeometricDeviation

[![CI](https://github.com/SergeyMakeev/MeshGeometricDeviation/actions/workflows/ci.yml/badge.svg)](https://github.com/SergeyMakeev/MeshGeometricDeviation/actions/workflows/ci.yml)

A high-performance C++ library for computing geometric deviations between 3D triangle meshes. Provides bidirectional comparison with comprehensive statistics including min/max deviation, RMSD, and asymmetry analysis.

## Features

### Core Library Features
- **Bidirectional Mesh Comparison** - Detects both missing and extra geometry
- **Outer Shell Sampling** - Optionally samples only the outer visible surface, excluding internal geometry
- **Vertex Normal Variance** - Measures angular differences between interpolated surface normals
- **UV Coordinate Variance** - Measures texture coordinate differences between corresponding points
- **Automatic Sample Count Computation** - Based on surface area and configurable density
- **Guaranteed Coverage** - At least one sample per triangle
- **Reproducible Results** - Fixed random seeds for consistent comparisons
- **Normal-Aware Queries** - Finds closest points on similarly-oriented surfaces
- **KD-Tree Spatial Acceleration** - Efficient spatial queries for large meshes
- **Comprehensive Statistics** - Position, normal, and UV variance with full statistical analysis

### Library Architecture
- **Header-only compatible** - Easy integration
- **Namespace isolation** - `MeshGeometricDeviation::`
- **Configurable logging** - Printf-style with severity levels (Debug/Info/Warning/Error)
- **Modern C++17** - Clean, maintainable code
- **Cross-platform** - Windows, Linux, macOS
- **CMake-based** - Standard build system with proper installation

## Quick Start

### Building the Library

```bash
# Clone the repository
git clone https://github.com/SergeyMakeev/MeshGeometricDeviation.git
cd MeshGeometricDeviation

# Configure and build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release

# Run example
./build/examples/Release/mesh_compare models/cube.obj models/cube_scaled.obj
```

### Using the Library in Your Project

#### CMake Integration

```cmake
find_package(MeshGeometricDeviation REQUIRED)
target_link_libraries(your_target PRIVATE MeshGeometricDeviation)
```

#### Basic Usage Example

```cpp
#include <MeshGeometricDeviation/MeshComparison.h>

using namespace MeshGeometricDeviation;

int main() {
    // Load meshes
    Mesh meshA, meshB;
    loadObjFile("reference.obj", meshA);
    loadObjFile("test.obj", meshB);
    
    // Compute vertex normals for normal variance analysis
    computeVertexNormals(meshA);
    computeVertexNormals(meshB);
    
    // Compute sample counts
    double sampleDensity = 20.0;  // samples per square unit
    int numSamplesA = computeNumSamples(meshA, sampleDensity);
    int numSamplesB = computeNumSamples(meshB, sampleDensity);
    
    // Compare meshes
    auto results = compareMeshesBidirectional(
        meshA, meshB, 
        numSamplesA, numSamplesB,
        45.0,   // max angle for normal matching
        true,   // use area weighting
        true,   // use normal filtering
        42,     // random seed
        false,  // outer shell only (set to true to exclude internal geometry)
        2000    // sphere points for outer shell detection
    );
    
    // Print results
    printBidirectionalStats(results);
    
    // Access statistics
    std::cout << "Max deviation: " << results.maxDeviance << std::endl;
    std::cout << "RMSD: " << results.rmsd << std::endl;
    std::cout << "Asymmetric: " << (results.isAsymmetric ? "Yes" : "No") << std::endl;
    std::cout << "Max normal angle: " << results.aToB.maxNormalAngleDeg << " degrees" << std::endl;
    
    return 0;
}
```

## Command-Line Tool

### Usage

```bash
mesh_compare <reference.obj> <test.obj> [sample_density] [max_angle] [seed] [--outer-shell] [--debug output.obj]
```

**Parameters:**
- `reference.obj` - Reference mesh (MeshA)
- `test.obj` - Test mesh to compare (MeshB)
- `sample_density` - Samples per square unit (default: 20.0)
- `max_angle` - Max angle for normal matching in degrees (default: 180.0)
  - 180.0 effectively disables normal filtering (accepts any orientation)
  - Use smaller values (e.g., 45.0) for stricter normal matching
- `seed` - Random seed for reproducibility (default: 42)

**Optional Flags:**
- `--outer-shell` - Only sample triangles on the outer shell, excluding internal geometry
  - Useful for meshes with internal scaffolding or hidden features
  - Uses sphere projection to efficiently detect visible surfaces
- `--debug output.obj` - Export debug visualization with extreme deviation points

**Note:** The CLI tool defaults to `max_angle=180.0` (no normal filtering) for maximum compatibility. The library API function `compareMeshesBidirectional` defaults to `45.0` for stricter matching. Choose based on your needs.

### Examples

```bash
# Compare with default settings (20 samples/unit^2, 180 deg angle, seed 42)
mesh_compare original.obj modified.obj

# Higher sample density for more accuracy
mesh_compare original.obj modified.obj 100

# Custom angle threshold for stricter normal matching
mesh_compare original.obj modified.obj 20 45.0

# Custom seed for different sampling pattern
mesh_compare original.obj modified.obj 20 180.0 123

# Low density (still guarantees one sample per triangle)
mesh_compare original.obj modified.obj 1.0

# Compare only outer shell (exclude internal geometry)
mesh_compare box_with_internals.obj box_reference.obj 20 180.0 42 --outer-shell

# Outer shell + debug visualization
mesh_compare complex.obj simple.obj 50 45.0 42 --outer-shell --debug debug_output.obj
```

### Output

The tool provides comprehensive statistics:

- **Surface area and computed sample counts** for each mesh
- **Random seed** used for reproducibility
- **Bidirectional comparison results** (A->B and B->A):
  - Min deviance (smallest distance found)
  - Max deviance (largest distance found)
  - Average deviance (mean distance)
  - Percentiles (p10, p20, p30, p40, p50, p60, p70, p80, p90, p95, p99)
  - RMSD (Root Mean Square Deviation - emphasizes larger errors)
- **Vertex Normal Variance** - Angular differences between interpolated vertex normals:
  - Min/max/average angles between normals
  - Percentiles for angles (p10, p20, p30, p40, p50, p60, p70, p80, p90, p95, p99)
  - Count of samples with large normal deviations (>15 deg)
  - Detects surface orientation changes even when positions are close
- **UV Coordinate Variance** - Distance between interpolated UV coordinates:
  - Min/max/average UV distances
  - Percentiles for UV distances (p10, p20, p30, p40, p50, p60, p70, p80, p90, p95, p99)
  - Count of samples with large UV deviations (>0.1)
  - Detects texture mapping changes and seam issues
- **Asymmetry analysis** - Detects if deviations are symmetric or indicate holes/extra geometry
- **Normal constraint statistics** - How many samples matched the orientation constraint
- **Large deviation counts** - Percentage of samples with significant deviations

## Library API

### Mesh Loading

```cpp
bool loadObjFile(const std::string& filename, Mesh& mesh);
```
Loads an OBJ file with automatic polygon triangulation.

### Mesh Analysis

```cpp
double computeMeshSurfaceArea(const Mesh& mesh);
int computeNumSamples(const Mesh& mesh, double samplesPerUnitArea);
void computeVertexNormals(Mesh& mesh);  // Compute area-weighted vertex normals

// Outer shell detection
std::vector<bool> classifyOuterShellTriangles(const Mesh& mesh, int numSpherePoints = 2000);
```

Vertex normals are automatically computed from adjacent face normals using area-weighted averaging. This enables the vertex normal variance metric that measures angular differences between surface orientations.

**Outer Shell Detection:**
The `classifyOuterShellTriangles` function identifies which triangles are part of the mesh's outer visible surface versus internal geometry. It uses an efficient sphere projection technique:
1. Computes a bounding sphere around the mesh
2. Generates uniformly distributed points on the sphere surface
3. Finds the closest triangle to each sphere point
4. Marks those triangles as part of the outer shell

This is useful for excluding internal scaffolding, support structures, or hidden geometry from comparison.

### Logging Control

```cpp
// Log severity levels
enum class LogLevel {
    Debug = 0,    // Detailed diagnostic information (timing, stats)
    Info = 1,     // General informational messages (default)
    Warning = 2,  // Warning messages
    Error = 3,    // Error messages  
    Silent = 4    // No logging
};

// Set minimum log level (filters out messages below this level)
void setLogLevel(LogLevel level);

// Set custom log callback for redirection
typedef void (*LogCallback)(LogLevel level, const char* message);
void setLogCallback(LogCallback callback);
```

**Examples:**
```cpp
// Show only important results (hide timing/debug info)
setLogLevel(LogLevel::Info);  // Default

// Show detailed timing and diagnostic info
setLogLevel(LogLevel::Debug);

// Suppress all library output
setLogLevel(LogLevel::Silent);

// Custom log handler (e.g., write to file)
void myLogger(LogLevel level, const char* message) {
    fprintf(myFile, "[%d] %s", static_cast<int>(level), message);
}
setLogCallback(myLogger);
```

### Mesh Comparison

```cpp
DevianceStats compareMeshes(
    const Mesh& meshA, const Mesh& meshB, 
    int numSamples,
    double maxAngleDegrees = 180.0,
    bool useAreaWeighting = true,
    bool useNormalFiltering = true,
    unsigned int seed = 42,
    bool outerShellOnly = false,
    int spherePoints = 2000
);

BidirectionalDevianceStats compareMeshesBidirectional(
    const Mesh& meshA, const Mesh& meshB,
    int numSamplesA, int numSamplesB,
    double maxAngleDegrees = 45.0,
    bool useAreaWeighting = true,
    bool useNormalFiltering = true,
    unsigned int baseSeed = 42,
    bool outerShellOnly = false,
    int spherePoints = 2000
);
```

**New Parameters:**
- `outerShellOnly` - When true, automatically classifies and samples only outer shell triangles
- `spherePoints` - Number of sphere points used for outer shell detection (more = more accurate, slower)

### Output Functions

```cpp
void printDevianceStats(const DevianceStats& stats, bool showNormalStats = true);
void printBidirectionalStats(const BidirectionalDevianceStats& biStats);
```

## Project Structure

```
MeshGeometricDeviation/
+-- include/
|   +-- MeshGeometricDeviation/
|       +-- MeshComparison.h    # Main comparison API
|       +-- SpatialDb.h          # KD-tree spatial database
+-- src/
|   +-- MeshComparison.cpp       # Comparison implementation
|   +-- SpatialDb.cpp            # Spatial database implementation
+-- examples/
|   +-- CMakeLists.txt
|   +-- mesh_compare.cpp         # Command-line tool
+-- models/                      # Sample OBJ files for testing
+-- .github/
|   +-- workflows/
|       +-- ci.yml               # GitHub Actions CI/CD
+-- cmake/
|   +-- Config.cmake.in          # CMake package config
+-- CMakeLists.txt               # Root build configuration
+-- README.md
```

## Algorithm Overview

The library performs **bidirectional comparison** to detect both missing and extra geometry:

### Direction 1: Reference (A) -> Test (B)
1. **Load Meshes** - Both reference and test meshes loaded from OBJ files with UV coordinates
2. **Compute Vertex Normals** - Per-vertex normals calculated using area-weighted averaging
3. **Compute Sample Counts** - Automatically calculated based on:
   - Surface area of each mesh
   - User-specified sample density (samples per square unit)
   - Minimum guarantee: at least one sample per triangle
4. **Build Spatial Database** - KD-tree constructed for efficient queries
5. **Surface Sampling** - Random points generated with:
   - **First pass**: One sample per triangle (guaranteed coverage)
   - **Second pass**: Additional samples distributed by area
   - Barycentric coordinates for uniform distribution and normal interpolation
   - Fixed random seed for reproducibility
6. **Distance Measurement** - For each sample point:
   - Query closest point on target mesh surface
   - Use normal-aware search (prefers similarly-oriented triangles)
   - Falls back if no normals match
   - Returns barycentric coordinates for normal interpolation
7. **Normal Variance Measurement** - Interpolate vertex normals and compute angular differences
8. **UV Variance Measurement** - Interpolate UV coordinates and compute distance between them
9. **Statistical Analysis** - Compute comprehensive statistics for position, normal, and UV variance

### Direction 2: Test (B) -> Reference (A)
- Process repeated in opposite direction with different seed
- Sample count computed based on mesh B's surface area
- Detects extra geometry in test mesh

### Asymmetry Analysis
- Compares deviations in both directions
- Identifies if deviations are symmetric or indicate missing/extra geometry
- Reports overall statistics

## Dependencies

- **CMake 3.14+** - Build system
- **C++17 compiler** - GCC, Clang, or MSVC
- **fast_obj** - Automatically fetched via CMake FetchContent

## Building

### Requirements
- CMake 3.14 or higher
- C++17 compatible compiler

### Build Options
- `BUILD_EXAMPLES` - Build example applications (default: ON)

### Build Commands

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release

# Install (optional)
cmake --install build --prefix /usr/local
```

## CI/CD

The project includes GitHub Actions workflows that automatically:
- Build on Linux, Windows, and macOS
- Test with multiple compilers (GCC, Clang, MSVC)
- Verify functionality across platforms
- Upload build artifacts

## Performance

- **KD-tree acceleration** - O(log n) spatial queries
- **Area-weighted sampling** - Ensures uniform surface coverage
- **Efficient memory usage** - Optimized data structures
- **Scalable** - Handles meshes with millions of triangles

## Use Cases

- **Quality assurance** - Verify mesh modifications
- **LOD validation** - Check level-of-detail generation
- **Mesh simplification** - Measure decimation error
- **Format conversion** - Validate import/export accuracy
- **Remeshing analysis** - Quantify remeshing deviation
- **Mesh repair** - Assess repair quality
- **Normal map validation** - Verify surface orientation preservation
- **Smoothing analysis** - Measure how smoothing operations affect surface normals
- **UV unwrapping validation** - Check texture coordinate consistency
- **Texture atlas changes** - Detect UV mapping modifications
- **3D printing validation** - Compare models with internal support structures (use outer shell mode)
- **Assembly validation** - Check external appearance while ignoring internal components

## License

This project uses the fast_obj library which is licensed under the MIT License.

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style
- All tests pass
- Documentation is updated
- CI/CD checks pass

## Citation

If you use this library in your research, please cite:

```bibtex
@software{meshgeometricdeviation,
  title = {MeshGeometricDeviation: A C++ Library for Mesh Comparison},
  author = {Your Name},
  year = {2025},
  url = {https://github.com/SergeyMakeev/MeshGeometricDeviation}
}
```

## Acknowledgments

- [fast_obj](https://github.com/thisistherk/fast_obj) - Fast OBJ file parsing
