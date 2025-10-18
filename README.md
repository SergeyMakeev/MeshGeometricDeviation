# MeshGeometricDeviation

[![CI](https://github.com/YOUR_USERNAME/MeshGeometricDeviation/actions/workflows/ci.yml/badge.svg)](https://github.com/YOUR_USERNAME/MeshGeometricDeviation/actions/workflows/ci.yml)

A high-performance C++ library for computing geometric deviations between 3D triangle meshes. Provides bidirectional comparison with comprehensive statistics including min/max deviation, RMSD, and asymmetry analysis.

## Features

### Core Library Features
- **Bidirectional Mesh Comparison** - Detects both missing and extra geometry
- **Automatic Sample Count Computation** - Based on surface area and configurable density
- **Guaranteed Coverage** - At least one sample per triangle
- **Reproducible Results** - Fixed random seeds for consistent comparisons
- **Normal-Aware Queries** - Finds closest points on similarly-oriented surfaces
- **KD-Tree Spatial Acceleration** - Efficient spatial queries for large meshes
- **Comprehensive Statistics** - Min, max, average, median, RMSD, and asymmetry metrics

### Library Architecture
- **Header-only compatible** - Easy integration
- **Namespace isolation** - `MeshGeometricDeviation::`
- **Modern C++17** - Clean, maintainable code
- **Cross-platform** - Windows, Linux, macOS
- **CMake-based** - Standard build system with proper installation

## Quick Start

### Building the Library

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/MeshGeometricDeviation.git
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
    
    // Compute sample counts
    double sampleDensity = 100.0;  // samples per square unit
    int numSamplesA = computeNumSamples(meshA, sampleDensity);
    int numSamplesB = computeNumSamples(meshB, sampleDensity);
    
    // Compare meshes
    auto results = compareMeshesBidirectional(
        meshA, meshB, 
        numSamplesA, numSamplesB,
        45.0,  // max angle for normal matching
        true,  // use area weighting
        true,  // use normal filtering
        42     // random seed
    );
    
    // Print results
    printBidirectionalStats(results);
    
    // Access statistics
    std::cout << "Max deviation: " << results.maxDeviance << std::endl;
    std::cout << "RMSD: " << results.rmsd << std::endl;
    std::cout << "Asymmetric: " << (results.isAsymmetric ? "Yes" : "No") << std::endl;
    
    return 0;
}
```

## Command-Line Tool

### Usage

```bash
mesh_compare <reference.obj> <test.obj> [sample_density] [max_angle] [seed]
```

**Parameters:**
- `reference.obj` - Reference mesh (MeshA)
- `test.obj` - Test mesh to compare (MeshB)
- `sample_density` - Samples per square unit (default: 100.0)
- `max_angle` - Max angle for normal matching in degrees (default: 45.0)
- `seed` - Random seed for reproducibility (default: 42)

### Examples

```bash
# Compare with default settings (100 samples/unit², 45° angle, seed 42)
mesh_compare original.obj modified.obj

# Higher sample density for more accuracy
mesh_compare original.obj modified.obj 200

# Custom angle threshold
mesh_compare original.obj modified.obj 100 30.0

# Custom seed for different sampling pattern
mesh_compare original.obj modified.obj 100 45.0 123

# Low density (still guarantees one sample per triangle)
mesh_compare original.obj modified.obj 1.0
```

### Output

The tool provides comprehensive statistics:

- **Surface area and computed sample counts** for each mesh
- **Random seed** used for reproducibility
- **Bidirectional comparison results** (A→B and B→A):
  - Min deviance (smallest distance found)
  - Max deviance (largest distance found)
  - Average deviance (mean distance)
  - Median deviance (50th percentile distance)
  - RMSD (Root Mean Square Deviation - emphasizes larger errors)
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
```

### Mesh Comparison

```cpp
DevianceStats compareMeshes(
    const Mesh& meshA, const Mesh& meshB, 
    int numSamples,
    double maxAngleDegrees = 45.0,
    bool useAreaWeighting = true,
    bool useNormalFiltering = true,
    unsigned int seed = 42
);

BidirectionalDevianceStats compareMeshesBidirectional(
    const Mesh& meshA, const Mesh& meshB,
    int numSamplesA, int numSamplesB,
    double maxAngleDegrees = 45.0,
    bool useAreaWeighting = true,
    bool useNormalFiltering = true,
    unsigned int baseSeed = 42
);
```

### Output Functions

```cpp
void printDevianceStats(const DevianceStats& stats, bool showNormalStats = true);
void printBidirectionalStats(const BidirectionalDevianceStats& biStats);
```

## Project Structure

```
MeshGeometricDeviation/
├── include/
│   └── MeshGeometricDeviation/
│       ├── MeshComparison.h    # Main comparison API
│       └── SpatialDb.h          # KD-tree spatial database
├── src/
│   ├── MeshComparison.cpp       # Comparison implementation
│   └── SpatialDb.cpp            # Spatial database implementation
├── examples/
│   ├── CMakeLists.txt
│   └── mesh_compare.cpp         # Command-line tool
├── models/                      # Sample OBJ files for testing
├── .github/
│   └── workflows/
│       └── ci.yml               # GitHub Actions CI/CD
├── cmake/
│   └── Config.cmake.in          # CMake package config
├── CMakeLists.txt               # Root build configuration
└── README.md
```

## Algorithm Overview

The library performs **bidirectional comparison** to detect both missing and extra geometry:

### Direction 1: Reference (A) → Test (B)
1. **Load Meshes** - Both reference and test meshes loaded from OBJ files
2. **Compute Sample Counts** - Automatically calculated based on:
   - Surface area of each mesh
   - User-specified sample density (samples per square unit)
   - Minimum guarantee: at least one sample per triangle
3. **Build Spatial Database** - KD-tree constructed for efficient queries
4. **Surface Sampling** - Random points generated with:
   - **First pass**: One sample per triangle (guaranteed coverage)
   - **Second pass**: Additional samples distributed by area
   - Barycentric coordinates for uniform distribution
   - Fixed random seed for reproducibility
5. **Distance Measurement** - For each sample point:
   - Query closest point on target mesh surface
   - Use normal-aware search (prefers similarly-oriented triangles)
   - Falls back if no normals match
6. **Statistical Analysis** - Compute comprehensive statistics

### Direction 2: Test (B) → Reference (A)
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
  url = {https://github.com/YOUR_USERNAME/MeshGeometricDeviation}
}
```

## Acknowledgments

- [fast_obj](https://github.com/thisistherk/fast_obj) - Fast OBJ file parsing
