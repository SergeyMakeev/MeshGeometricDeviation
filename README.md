# MeshDeviance - Mesh Comparison and Analysis Tool

A high-performance C++ tool for loading, analyzing, and comparing 3D meshes in Wavefront .OBJ format.

## Features

### Core Features
- **Fast OBJ file parsing** using [fast_obj](https://github.com/thisistherk/fast_obj)
- **Spatial Database** with KD-tree for efficient spatial queries
- **Mesh Comparison** tool for measuring geometric differences between meshes
- **Normal-aware queries** for surface orientation-sensitive operations
- Automatic dependency management with CMake FetchContent

### Spatial Database (SpatialDb)
- KD-tree based spatial partitioning for fast queries
- Closest point queries on mesh surfaces
- Normal-filtered queries (finds closest points on similarly-oriented triangles)
- Automatic fallback when no triangles match normal constraints
- Efficient distance field computations

### Mesh Comparison Tool
- Compare two meshes with different topology
- **Bidirectional comparison** (samples both meshes in both directions)
- **Automatic sample count computation** based on surface area and configurable density
- **Guaranteed minimum coverage**: at least one sample per triangle
- **Reproducible results** with fixed random seeds
- Detects missing geometry (holes) and extra geometry
- Surface sampling using barycentric coordinates
- Area-weighted sampling for uniform distribution
- Normal-aware distance measurements
- Comprehensive statistical analysis: min, max, average, median, and RMSD
- Reports on normal constraint matching and asymmetry

## Building

### Prerequisites

- CMake 3.14 or higher
- C++17 compatible compiler (GCC, Clang, MSVC)
- Git

### Build Instructions

```bash
# Create build directory
mkdir build
cd build

# Configure CMake
cmake ..

# Build the project
cmake --build .
```

## Usage

### Mesh Loader Demo

```bash
# Run the mesh loader demo with an OBJ file
./mesh_loader path/to/your/model.obj

# Windows
mesh_loader.exe path\to\your\model.obj
```

Example:
```bash
./mesh_loader models/cube.obj
```

### Mesh Comparison Tool

```bash
# Compare two meshes
./mesh_compare <reference_mesh.obj> <test_mesh.obj> [sample_density] [max_angle_degrees] [seed]

# Windows
mesh_compare.exe reference.obj test.obj [sample_density] [max_angle_degrees] [seed]
```

**Parameters:**
- `reference_mesh.obj` - The reference mesh (MeshA) to sample from
- `test_mesh.obj` - The test mesh (MeshB) to compare against
- `sample_density` - Samples per square unit of surface area (default: 100.0)
  - Sample count is automatically computed based on mesh surface area
  - Each triangle is guaranteed at least one sample point
- `max_angle_degrees` - Max angle for normal matching in degrees (default: 45.0)
- `seed` - Random seed for reproducible results (default: 42)

**Examples:**

```bash
# Compare two meshes with default settings (100 samples/unit², 45° angle, seed 42)
./mesh_compare models/cube.obj models/cube_scaled.obj

# Use higher sample density for more accuracy
./mesh_compare models/original.obj models/modified.obj 200

# Stricter normal matching (30° instead of 45°)
./mesh_compare models/original.obj models/modified.obj 100 30.0

# Custom seed for different random sampling pattern
./mesh_compare models/original.obj models/modified.obj 100 45.0 123

# Low density but still guarantees one sample per triangle
./mesh_compare models/original.obj models/modified.obj 1.0
```

**Output includes:**
- **Surface area and computed sample counts** for each mesh
- **Random seed** used for reproducibility
- Bidirectional comparison results (A→B and B→A)
- Min deviance (smallest distance found)
- Max deviance (largest distance found in either direction)
- Average deviance (mean distance)
- Median deviance (50th percentile distance)
- RMSD (Root Mean Square Deviation - gives more weight to large deviations)
- Asymmetry analysis (detects holes vs. extra geometry)
- Normal constraint statistics (how many samples matched the orientation constraint)
- Large deviation counts and percentages

## Project Structure

```
MeshDeviance/
├── CMakeLists.txt          # CMake configuration with FetchContent
├── src/
│   ├── mesh_compare.cpp    # Mesh comparison tool (main application)
│   ├── MeshComparison.h    # Mesh comparison library header
│   ├── MeshComparison.cpp  # Mesh comparison library implementation
│   ├── SpatialDb.h         # Spatial database header
│   └── SpatialDb.cpp       # Spatial database implementation
├── models/
│   ├── cube.obj            # Sample cube model
│   └── cube_scaled.obj     # Scaled cube for testing
└── README.md               # This file
```

## Library Structure

The project is organized as a reusable library (`MeshComparison.h/cpp`) that can be integrated into other projects:

**MeshComparison Library API:**
- `loadObjFile()` - Load OBJ files with automatic triangulation
- `computeMeshSurfaceArea()` - Calculate total mesh surface area
- `computeNumSamples()` - Auto-compute sample count based on density
- `compareMeshes()` - Single-direction mesh comparison
- `compareMeshesBidirectional()` - Bidirectional mesh comparison
- `printDevianceStats()` - Output comparison statistics
- `printBidirectionalStats()` - Output bidirectional statistics

The library uses the `SpatialDb` KD-tree for efficient spatial queries and the `fast_obj` library for OBJ file parsing.

## How It Works

### Mesh Comparison Algorithm

The tool performs **bidirectional comparison** to detect both missing and extra geometry:

**Direction 1: Reference (A) → Test (B)**
1. **Load Meshes**: Both reference (MeshA) and test (MeshB) meshes are loaded
2. **Compute Sample Counts**: Automatically calculate samples based on:
   - Surface area of each mesh (computed from all triangles)
   - User-specified sample density (samples per square unit)
   - Minimum guarantee: at least one sample per triangle
   - Each mesh can have a different sample count based on its area
3. **Build Spatial Database**: A KD-tree is constructed for MeshB for efficient queries
4. **Surface Sampling**: Random points are generated on MeshA's surface using:
   - **First pass**: One sample per triangle (guaranteed coverage)
   - **Second pass**: Additional samples distributed by area (larger triangles get more)
   - Barycentric coordinates for uniform point distribution within triangles
   - Fixed random seed for reproducible results
5. **Distance Measurement**: For each sample point:
   - Query the closest point on MeshB's surface
   - Use normal-aware search (prefers triangles with similar orientation)
   - Falls back to any triangle if no normals match
6. **Statistical Analysis**: Compute comprehensive statistics:
   - Min/max deviance (range of deviations)
   - Average and median deviance (central tendency)
   - RMSD (Root Mean Square Deviation - emphasizes larger errors)

**Direction 2: Test (B) → Reference (A)**
- The process is repeated in the opposite direction with a different seed
- Sample count computed based on mesh B's surface area
- Detects extra geometry in the test mesh

**Asymmetry Analysis**
- Compares deviations in both directions
- Identifies if deviations are symmetric or if one mesh has missing/extra geometry
- Reports overall max deviance and combined statistics

### Spatial Database (KD-Tree)

The `SpatialDb` class provides:
- **Fast spatial queries** using KD-tree partitioning
- **Normal-filtered searches** that prefer triangles facing similar directions
- **Robust fallback** when no triangles match the normal constraint
- **Optimized tree construction** with configurable depth and triangle count limits

## Fast OBJ Library

This project uses the [fast_obj](https://github.com/thisistherk/fast_obj) library, which is automatically downloaded during the CMake configuration phase. The library is:

- Single header implementation
- Fast (5-10x faster than other OBJ loaders)
- C89/C++ compatible
- MIT licensed

## License

This project uses the Fast OBJ library which is licensed under the MIT License.



