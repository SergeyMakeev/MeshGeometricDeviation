# Mesh Geometric Deviation Algorithm

This document explains how the mesh comparison algorithm works in simple, easy-to-understand terms.

## What Does This Tool Do?

This tool compares two 3D meshes (like 3D models) to see how different they are. Think of it like comparing two sculptures to measure how closely they match.

## The Basic Idea

Imagine you have two 3D models:
- **Mesh A** (the reference/original)
- **Mesh B** (the test/modified version)

We want to know: "How different are these two meshes?"

## How It Works (Step by Step)

### Step 1: Random Sampling

Instead of checking every single point (which would be extremely slow), we:
1. Pick random points on the surface of Mesh A
2. Pick random points on the surface of Mesh B

The number of points we pick depends on the mesh size - bigger meshes get more sample points to ensure good coverage.

**Why random?** Random sampling is fast and gives us a good overall picture without having to check millions of points.

### Step 2: Finding the Closest Point

For each random point we picked on Mesh A:
1. We find the closest point on Mesh B
2. We measure the distance between them

This tells us "how far" the meshes are from each other at that spot.

**Example:** If you have a point on your hand in one sculpture, we find where the hand is in the other sculpture and measure how far apart they are.

### Step 3: Bidirectional Comparison

Here's the clever part - we do this **in both directions**:

1. **A → B**: Sample points on Mesh A, find closest points on Mesh B
2. **B → A**: Sample points on Mesh B, find closest points on Mesh A

**Why both directions?**
- **A → B** detects where Mesh B is missing geometry (holes)
- **B → A** detects where Mesh B has extra geometry (bumps/additions)

If we only measured one direction, we'd miss important differences!

### Step 4: Measuring More Than Just Distance

We don't just measure how far apart points are. We also measure:

#### Surface Orientation (Normal Angles)
- Each point on a surface has a direction it's "facing" (called a normal)
- We compare if matching points are facing the same direction
- This catches surfaces that are close but twisted or flipped

**Example:** Two surfaces might be very close together but one is facing up and one is facing down - distance alone wouldn't catch this!

#### Texture Mapping (UV Coordinates)
- If the meshes have texture coordinates (for applying images/textures)
- We check if matching points map to the same spot in the texture
- This catches texture misalignment even when geometry matches

### Step 5: Calculating Statistics

After collecting all the distance measurements, we calculate:

#### Percentiles (Distribution)
We sort all the distances from smallest to largest and find:
- **p10** = 10% of points have distance less than this
- **p20** = 20% of points have distance less than this
- **p50** = 50% of points have distance less than this (the median)
- **p90** = 90% of points have distance less than this
- **p95** = 95% of points have distance less than this
- **p99** = 99% of points have distance less than this

**Why percentiles?** They show the whole picture:
- Low percentiles (p10, p20) = most points match well
- Middle percentiles (p50) = typical difference
- High percentiles (p90, p95, p99) = worst-case differences (outliers)

#### Other Statistics
- **Min/Max**: The smallest and largest distances found
- **Average**: The mean distance across all samples
- **RMSD** (Root Mean Square Deviation): Emphasizes larger errors more than the average

## Key Optimizations

### Spatial Database (BVH Tree)

When we need to find "the closest point on Mesh B" for thousands of sample points, we can't check every triangle every time - that would be way too slow.

Instead, we use a **Bounding Volume Hierarchy (BVH)**:
1. Group triangles into boxes
2. Group those boxes into bigger boxes
3. Create a tree structure

Now finding the closest point is fast:
- Start at the top of the tree
- Ignore entire groups of triangles that are too far away
- Only check triangles that might actually be closest

**Analogy:** Instead of checking every book in a library, you first check which section, then which shelf, then which book.

### Area-Weighted Sampling

Big triangles get more sample points than small triangles. This ensures:
- We don't over-sample tiny details
- We don't under-sample large surfaces
- Coverage is proportional to surface area

### Normal-Aware Queries (Optional)

When finding the closest point, we can optionally prefer points where surfaces are facing similar directions. This helps when:
- Meshes have thin walls or overlapping surfaces
- You want to match "front to front" not "front to back"

## The Output

The tool tells you:

1. **How close are the meshes?**
   - Percentiles show the distribution of differences
   - p50 tells you the typical difference
   - p99 tells you the worst-case outliers

2. **Are differences symmetric?**
   - If A→B and B→A give similar results = meshes are very similar
   - If A→B is much larger = Mesh B has holes/missing parts
   - If B→A is much larger = Mesh B has extra geometry

3. **Surface orientation differences**
   - Even if surfaces are close, are they facing the same way?

4. **Texture mapping differences**
   - Are textures aligned correctly?

## When To Use This Tool

- **Quality control**: Check if a simplified mesh is close enough to the original
- **Format conversion**: Verify a mesh exported to a different format is identical
- **LOD verification**: Check if lower-detail versions maintain shape accuracy
- **Bug detection**: Find where mesh processing introduced errors
- **Version comparison**: See what changed between two versions of a model

## Simple Example

Imagine comparing two 3D models of a coffee cup:

1. We randomly pick 10,000 points on the surface of Cup A
2. For each point, we find the closest point on Cup B and measure the distance
3. We repeat going the other direction (Cup B → Cup A)
4. We calculate statistics:
   - p50 = 0.001 units → typical difference is very small
   - p90 = 0.005 units → 90% of points are within 0.005 units
   - p99 = 0.050 units → 99% of points are within 0.05 units (1% are outliers)
   - Max = 2.000 units → worst case is the handle is 2 units off

From this we learn: The cups are very similar overall, but there's one area (probably the handle) that's quite different.

## Summary

The algorithm is essentially:
1. **Sample** random points on both meshes
2. **Find** closest matches in both directions
3. **Measure** geometric distance, orientation, and texture alignment
4. **Calculate** percentiles and statistics
5. **Report** how similar the meshes are and where they differ

It's fast, accurate, and gives you detailed information about mesh differences without requiring perfect alignment or preprocessing.

