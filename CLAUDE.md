# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ZKTA (Zombie Kill Terrain Analyzer) is a Java 8 terrain analysis system that processes bitmap images to generate navigation meshes for pathfinding in games. The system traces obstacle contours, generates Voronoi diagrams for medial axis computation, and identifies navigable regions and strategic chokepoints.

## Build Commands

This is a Maven project (Java 8). Use these commands:

```bash
# Compile the project
mvn compile

# Run the main application (entry point: ZKTAWindow.java)
mvn exec:java -Dexec.mainClass="ZKTAWindow"

# Package as JAR
mvn package
```

## Core Architecture

The main processing pipeline in **ZKTerrainAnalyzer.java** follows these steps:
1. **Contour Tracing**: Extract obstacle boundaries from bitmap images using Moore neighborhood algorithm
2. **Simplification**: Reduce contour complexity using Douglas-Peucker algorithm (via `simplify` library)
3. **Voronoi Generation**: Create medial axis using `jopenvoronoi` library
4. **Graph Pruning**: Remove nodes too close to obstacles (TOO_CLOSE_TO_OBSTACLE = 5 pixels)
5. **Region Detection**: Identify navigable areas connected by graph edges
6. **Chokepoint Detection**: Find narrow passages (10-50 pixel width) between regions using BWTA2 algorithm

## Key Classes

- **ZKTAWindow.java**: Main entry point with visualization and image I/O
- **ZKTerrainAnalyzer.java**: Core terrain analysis engine and processing pipeline
- **RegionGraph.java**: Graph structure representing navigable regions and connections
- **ChokepointNode.java**: Represents strategic narrow passages between regions
- **Contour.java/LabeledContour.java**: Polygon boundaries for obstacles and holes
- **Node.java**: Basic graph vertex with position and neighbor connections

## Coordinate Systems

The system uses two coordinate systems requiring conversion:
- **Image Coordinates**: Integer pixels (0,0 at top-left) for bitmap processing
- **Voronoi Coordinates**: Normalized floats in unit circle (centered at origin) for jopenvoronoi library

Use coordinate conversion methods in ZKTerrainAnalyzer:
- `coordsToVoronoi(int x, int y)` - Convert image to Voronoi space
- `coordsFromVoronoi(double x, double y)` - Convert back to image pixels

## Dependencies

- **jopenvoronoi**: Voronoi diagram generation (embedded in `org.rogach.jopenvoronoi` package)
- **rtree** (com.github.davidmoten): R-tree spatial indexing for obstacle queries
- **simplify** (com.goebl): Douglas-Peucker line simplification
- **commons-math3**: Mathematical utilities

## Input/Output

- Input images (like `trojan.png`) should be in project root
- Output images (like `result.png`) are generated in project root
- **Output data (regions, chokepoints, edge contacts) is written to `result.json` - not to stdout**
- Console output contains debug/trace information only
- Uses hardcoded file paths - no external configuration required

## Performance Notes

- Uses R-tree for efficient nearest-obstacle distance queries during graph pruning
- System.currentTimeMillis() used for performance timing throughout
- Spatial indexing stores simplified contour line segments for fast distance calculations
- The correct main class to start with is ZKTAWindow
- Run the analyzer with the headless flag to avoid waiting for the window to be closed
- run zktawindow with --headless to not have to wait for user input