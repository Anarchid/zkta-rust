import com.github.davidmoten.rtree.Entry;
import com.github.davidmoten.rtree.RTree;
import com.github.davidmoten.rtree.geometry.Geometries;
import com.github.davidmoten.rtree.geometry.Line;
import com.goebl.simplify.Simplify;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.lang.reflect.Array;
import java.util.*;
import java.util.List;
import java.util.concurrent.atomic.AtomicReferenceArray;

import org.rogach.jopenvoronoi.*;

public class ZKTerrainAnalyzer {
    // given a bitmap (a walkability map in our context) it returns the external contour of obstacles
    BufferedImage bitMap;
    ArrayList<Contour> fullContours;
    ArrayList<Contour> simplifiedContours;
    VoronoiDiagram voronoid;
    Vertex[] errorEdge;
    Vertex[][] borders;
    public ArrayList<LabeledContour> holes;
    int[][] labelMap;
    RTree<Integer,Line> labelTree;
    RegionGraph regions;
    ArrayList<Region> regionList;
    ArrayList<Chokepoint> chokepointList;
    ArrayList<MapEdgeContact> mapEdgeContacts;
    double v_width;
    double v_height;
    double radius;
    double diagonal;
    int width;
    int height;
    static int TOO_CLOSE_TO_OBSTACLE = 15; // Adjusted for proper segment distance calculation
    ArrayList<Edge> pruned;
    RTree<Integer, Line> simplifiedLabelTree;  // R-tree with simplified geometry
    ArrayList<JaggedSegment> jaggedSegments;   // Stores debug info for jagged boundary segments

    ZKTerrainAnalyzer(BufferedImage img){
        width = img.getWidth();
        height = img.getHeight();
        v_width = (double)width;
        v_height = (double)height;
        radius = Math.sqrt(width*width + height*height)/2;

        bitMap = img;
        labelMap = new int[height][width];
        labelTree = RTree.create();
        holes = new ArrayList<>();
        jaggedSegments = new ArrayList<>();
        regionList = new ArrayList<>();
        chokepointList = new ArrayList<>();
        mapEdgeContacts = new ArrayList<>();
        fullContours = traceContours();
        simplifiedContours = simplifyContours();
        voronoid = generateVoronoi((ArrayList<Contour>)simplifiedContours.clone());
        pruned = getPrunedEdges(voronoid);
        labelTree = createRTree();
        regions = pruneGraph(convertGraph(voronoid.get_graph_reference()));
        calculateNodeRadii(regions);
        identifyRegionsAndChokepoints(regions);
        pruneContainedChokepoints(regions);
        cleanupSandwichedRegions(regions);
        skipUnmarkedVertices(regions);
        mergeContiguousNodes(regions);
        simplifiedLabelTree = createSimplifiedRTree();
        // Recalculate radii using the simplified geometry
        calculateNodeRadii(regions);
        // extractRegionPolygons(regions);  // Disabled for now - WIP region detection

        // Instead, split obstacle segments at chokepoint contact points
        splitObstacleSegmentsAtChokepointContacts(regions);

        // Detect map edge contacts for obstacles and chokepoints
        detectMapEdgeContacts();

        // Build region boundaries from chokepoint contacts
        buildRegionBoundariesFromChokepoints(regions);

        // Test: Check if obstacle contours themselves are properly identified as closed
        testObstacleContourClosedness();
    }

    public int[][] getLabelMap(){
        return labelMap;
    }

    public int getHeight(){
        return height;
    }

    public int getWidth(){
        return width;
    }

    public RegionGraph getRegions(){
        return regions;
    }

    public ArrayList<Region> getRegionList(){
        return regionList;
    }

    public ArrayList<JaggedSegment> getJaggedSegments(){
        return jaggedSegments;
    }

    public ArrayList<MapEdgeContact> getMapEdgeContacts(){
        return mapEdgeContacts;
    }

    public Point[] getChokepointSegments(Node chokepoint){
        // Find the Chokepoint object for this node
        for (Chokepoint cp : regions.chokepoints) {
            if (cp.node == chokepoint) {
                return cp.crossingPoints;
            }
        }
        return null;
    }

    private RTree<Integer,Line> createRTree(){
        long timeStarted = System.currentTimeMillis();
        int i = 0;
        for(Contour c:simplifiedContours){
            for (int j = 1;j<c.points.size();j++){
                Point p1 = c.points.get(j-1);
                Point p2 = c.points.get(j);
                labelTree = labelTree.add(i, Geometries.line(p1.x,p1.y,p2.x,p2.y));
            }
        }
        System.out.println("Generated RTree in "+(System.currentTimeMillis() - timeStarted) + "ms");
        return labelTree;
    }

    private RTree<Integer,Line> createSimplifiedRTree(){
        long timeStarted = System.currentTimeMillis();
        RTree<Integer,Line> simplifiedTree = RTree.create();
        int i = 0;

        // Add map boundary segments as obstacles
        // Top edge: (0,0) to (width-1,0)
        simplifiedTree = simplifiedTree.add(i++, Geometries.line(0, 0, width-1, 0));
        // Bottom edge: (0,height-1) to (width-1,height-1)
        simplifiedTree = simplifiedTree.add(i++, Geometries.line(0, height-1, width-1, height-1));
        // Left edge: (0,0) to (0,height-1)
        simplifiedTree = simplifiedTree.add(i++, Geometries.line(0, 0, 0, height-1));
        // Right edge: (width-1,0) to (width-1,height-1)
        simplifiedTree = simplifiedTree.add(i++, Geometries.line(width-1, 0, width-1, height-1));

        // Use the CURRENT simplified contours (after all processing)
        ArrayList<Contour> currentSimplified = getSimplifiedContours();

        // Debug: Print contour count to verify sync with visualization
        System.out.println("DEBUG: Creating R-tree from " + currentSimplified.size() + " simplified contours");

        for(int contourIndex = 0; contourIndex < currentSimplified.size(); contourIndex++){
            Contour c = currentSimplified.get(contourIndex);
            // Use contour index + offset as obstacle ID (offset by 1000 to distinguish from map boundaries)
            int obstacleId = 1000 + contourIndex;
            System.out.println("DEBUG: Contour " + contourIndex + " (obstacle ID " + obstacleId + ") has " + c.points.size() + " points");
            for (int j = 1; j < c.points.size(); j++){
                Point p1 = c.points.get(j-1);
                Point p2 = c.points.get(j);
                simplifiedTree = simplifiedTree.add(obstacleId, Geometries.line(p1.x, p1.y, p2.x, p2.y));
                i++; // Still count segments for debug output
            }
        }
        System.out.println("Generated Simplified RTree with " + i + " line segments (" +
                         "4 map boundaries + " + (i-4) + " obstacle segments) in " +
                         (System.currentTimeMillis() - timeStarted) + "ms");
        return simplifiedTree;
    }

    private double distanceToLine(Point p, Line l){
        // Calculate proper distance from point to line segment
        double x1 = l.x1(), y1 = l.y1();
        double x2 = l.x2(), y2 = l.y2();
        double px = p.x, py = p.y;

        // Vector from line start to end
        double dx = x2 - x1;
        double dy = y2 - y1;

        // If line is actually a point
        if (dx == 0 && dy == 0) {
            return Math.sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
        }

        // Calculate the parameter t for the closest point on the line
        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);

        // Clamp t to [0,1] to stay within the line segment
        t = Math.max(0, Math.min(1, t));

        // Find the closest point on the segment
        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        // Return distance from point to closest point on segment
        return Math.sqrt((px - closestX) * (px - closestX) + (py - closestY) * (py - closestY));
    }

    private double getDistanceToObstacle(Node n){
        // Use the simplified R-tree if available, otherwise fall back to the original
        RTree<Integer,Line> treeToUse = (simplifiedLabelTree != null) ? simplifiedLabelTree : labelTree;
        String treeType = (simplifiedLabelTree != null) ? "simplified" : "original";
        double searchRadius = radius*2;

        rx.Observable<Entry<Integer,Line>> closestObstacle = treeToUse.nearest((Geometries.point(n.position.x,n.position.y)),searchRadius,1);
        Entry<Integer,Line> entry = closestObstacle.toBlocking().first();
        Line l = entry.geometry();
        Integer obstacleId = entry.value();
        double distance = distanceToLine(n.position, l);

        // Log for problematic chokepoint
        if (n.position.x == 105 && n.position.y == 410) {
            String debugTreeType = (treeToUse == simplifiedLabelTree) ? "simplified" : "original";
            System.out.println("DEBUG RADIUS: Node @" + n.position +
                             " found obstacle ID=" + obstacleId +
                             " at distance=" + String.format("%.1f", distance) +
                             " (search radius=" + String.format("%.1f", searchRadius) +
                             ", using " + debugTreeType + " tree)");
            System.out.println("DEBUG RADIUS: Segment from " + l.x1() + "," + l.y1() + " to " + l.x2() + "," + l.y2());
        }

        return distance;
    }

    private void calculateNodeRadii(RegionGraph graph) {
        System.out.println("Calculating radius (distance to obstacle) for all nodes...");
        for (Node n : graph.nodes) {
            n.radius = getDistanceToObstacle(n);
        }
        System.out.println("Completed radius calculation for " + graph.nodes.size() + " nodes");
    }

    private RegionGraph pruneGraph(RegionGraph graph)
    {
        // Get the list of all leafs (nodes with only one element in the adjacent list)
        LinkedList<Node> nodeToPrune = new LinkedList<>();
        for(Node n:graph.nodes){
            if(n.neighbours.size() == 1){
                nodeToPrune.add(n);
            }
        }

        System.out.println("Beginning region discovery with "+nodeToPrune.size()+" leaf nodes");
        int removed = 0;

        // Using leafs as starting points, prune the RegionGraph
        while (!nodeToPrune.isEmpty()) {
            // Pop first element
            Node n0 = nodeToPrune.poll();

            // Keep isolated nodes
            if (n0.neighbours.isEmpty()) continue;

            System.out.println("Processing node @"+n0.position);

            Node n1 = n0.neighbours.iterator().next();

            // Find distance to nearest obstacle
            double dist_0 = getDistanceToObstacle(n0);
            System.out.println("Distance to obstacle: "+dist_0);
            double dist_1 = 0;
            boolean deleted = false;

            if(dist_0 < TOO_CLOSE_TO_OBSTACLE){
                System.out.println("Too close, removing!");
                removed++;
                deleted = true;
                graph.removeNode(n0);
            } else {
                deleted = false; // Don't remove this node
            }

            if(deleted){
                System.out.println("Parent now has degree "+n1.neighbours.size());
                if (n1.neighbours.isEmpty() && dist_1 > TOO_CLOSE_TO_OBSTACLE) {
                    // Isolated node far enough from obstacles
                    System.out.println("Marked parent as isolated region");
                } else if (n1.neighbours.size() == 1) {
                    // Keep pruning if only one child
                    System.out.println("Queued parent for processing");
                    nodeToPrune.add(n1);
                }
            }
        }

        System.out.println("Total nodes removed during discovery "+removed);
        System.out.println("Total nodes retained during discovery: "+graph.nodes.size());

        return graph;
    }

    private void identifyRegionsAndChokepoints(RegionGraph graph) {
        System.out.println("Starting BWTA2 region and chokepoint identification...");

        // Reset all visited flags and parent references
        for (Node n : graph.nodes) {
            n.visited = false;
            n.parent = null;
        }

        // Find all leaf nodes (degree = 1) to start BFS traversal
        Queue<Node> toProcess = new LinkedList<>();
        for (Node n : graph.nodes) {
            if (n.neighbours.size() == 1) {
                toProcess.add(n);
                // Mark leaf nodes as regions (they are local maxima after pruning)
                graph.setNodeAsRegion(n);
                n.visited = true;
                System.out.println("Leaf node @" + n.position + " marked as region (radius=" + n.radius + ")");
            }
        }

        System.out.println("Starting BFS from " + toProcess.size() + " leaf nodes");

        // BFS traversal implementing BWTA2 algorithm
        while (!toProcess.isEmpty()) {
            Node current = toProcess.poll();

            // Add unvisited neighbors to the queue and mark them immediately
            for (Node neighbor : current.neighbours) {
                if (!neighbor.visited) {
                    neighbor.visited = true;
                    neighbor.parent = current;
                    toProcess.add(neighbor);

                    int degree = neighbor.neighbours.size();

                    // Apply BWTA2 rules - each node is processed exactly once
                    if (degree != 2) {
                        // Degree != 2: mark as region (leaf or intersection)
                        graph.setNodeAsRegion(neighbor);
                    } else {
                        // Degree = 2: apply local maxima/minima rules
                        if (neighbor.isLocalMinimum()) {
                            // Local minimum with degree 2
                            if (current.isLocalMinimum()) {
                                // Both neighbor and parent are local minima
                                // Mark the one with smallest radius as chokepoint
                                if (neighbor.radius < current.radius) {
                                    graph.setNodeAsChokepoint(neighbor);
                                }
                                // else: parent has smaller radius, so neighbor remains unmarked
                            } else {
                                // Only neighbor is local minimum
                                graph.setNodeAsChokepoint(neighbor);
                            }
                        } else if (neighbor.isLocalMaximum()) {
                            // Local maximum with degree 2
                            if (current.isLocalMaximum()) {
                                // Both neighbor and parent are local maxima
                                // Mark the one with biggest radius as region
                                if (neighbor.radius > current.radius) {
                                    graph.setNodeAsRegion(neighbor);
                                }
                                // else: parent has bigger radius, so neighbor remains unmarked
                            } else {
                                // Only neighbor is local maximum
                                graph.setNodeAsRegion(neighbor);
                            }
                        }
                        // If neither local min nor max, leave unmarked
                    }
                }
            }
        }

        // Verify no nodes are marked as both regions and chokepoints
        int dualMarked = 0;
        for (Node n : graph.nodes) {
            if (graph.regionNodes.contains(n) && graph.chokepointNodes.contains(n)) {
                System.out.println("ERROR: Node @" + n.position + " is marked as both region and chokepoint!");
                dualMarked++;
            }
        }

        System.out.println("BWTA2 identification complete:");
        System.out.println("- Regions: " + graph.regionNodes.size());
        System.out.println("- Chokepoints: " + graph.chokepointNodes.size());
        System.out.println("- Dual-marked nodes: " + dualMarked);
        System.out.println("- Unmarked nodes: " + (graph.nodes.size() - graph.regionNodes.size() - graph.chokepointNodes.size()));
    }

    private void pruneContainedChokepoints(RegionGraph graph) {
        System.out.println("Pruning chokepoints contained within region nodes...");

        ArrayList<Node> chokepointsToRemove = new ArrayList<>();

        for (Node chokepoint : graph.chokepointNodes) {
            for (Node region : graph.regionNodes) {
                // Calculate distance between chokepoint and region node
                double dx = chokepoint.position.x - region.position.x;
                double dy = chokepoint.position.y - region.position.y;
                double distanceToRegion = Math.sqrt(dx * dx + dy * dy);

                // Check if chokepoint is contained within region's inscribed circle
                if (chokepoint.radius + distanceToRegion < region.radius) {
                    System.out.println("Chokepoint @" + chokepoint.position +
                                     " (radius=" + String.format("%.1f", chokepoint.radius) +
                                     ") contained within region @" + region.position +
                                     " (radius=" + String.format("%.1f", region.radius) +
                                     ", distance=" + String.format("%.1f", distanceToRegion) + ")");
                    chokepointsToRemove.add(chokepoint);
                    break; // No need to check other regions for this chokepoint
                }
            }
        }

        // Remove the contained chokepoints
        for (Node chokepoint : chokepointsToRemove) {
            graph.chokepointNodes.remove(chokepoint);
        }

        System.out.println("Removed " + chokepointsToRemove.size() + " contained chokepoints");
        System.out.println("Final counts after pruning:");
        System.out.println("- Regions: " + graph.regionNodes.size());
        System.out.println("- Chokepoints: " + graph.chokepointNodes.size());
    }

    private void cleanupSandwichedRegions(RegionGraph graph) {
        System.out.println("Cleaning up sandwiched regions between chokepoints...");

        ArrayList<Node> regionsToRemove = new ArrayList<>();
        ArrayList<Node> chokepointsToRemove = new ArrayList<>();
        int mergedCases = 0;

        for (Node region : graph.regionNodes) {
            // Check if region has exactly 2 neighbors, both chokepoints
            if (region.neighbours.size() == 2) {
                Node neighbor1 = null, neighbor2 = null;
                boolean bothChokepoints = true;

                for (Node neighbor : region.neighbours) {
                    if (graph.chokepointNodes.contains(neighbor)) {
                        if (neighbor1 == null) {
                            neighbor1 = neighbor;
                        } else {
                            neighbor2 = neighbor;
                        }
                    } else {
                        bothChokepoints = false;
                        break;
                    }
                }

                if (bothChokepoints && neighbor1 != null && neighbor2 != null) {
                    // Check if region is contained within both chokepoints
                    double dist1 = Math.sqrt(Math.pow(region.position.x - neighbor1.position.x, 2) +
                                           Math.pow(region.position.y - neighbor1.position.y, 2));
                    double dist2 = Math.sqrt(Math.pow(region.position.x - neighbor2.position.x, 2) +
                                           Math.pow(region.position.y - neighbor2.position.y, 2));

                    boolean containedInBoth = (dist1 < neighbor1.radius) &&
                                            (dist2 < neighbor2.radius);

                    if (containedInBoth) {
                        System.out.println("Sandwiched region found: Region @" + region.position +
                                         " (r=" + String.format("%.1f", region.radius) + ")" +
                                         " between chokepoints @" + neighbor1.position +
                                         " (r=" + String.format("%.1f", neighbor1.radius) + ")" +
                                         " and @" + neighbor2.position +
                                         " (r=" + String.format("%.1f", neighbor2.radius) + ")");

                        // Keep the chokepoint with larger radius, remove the other two
                        Node chokepointToKeep, chokepointToRemove;
                        if (neighbor1.radius >= neighbor2.radius) {
                            chokepointToKeep = neighbor1;
                            chokepointToRemove = neighbor2;
                        } else {
                            chokepointToKeep = neighbor2;
                            chokepointToRemove = neighbor1;
                        }

                        System.out.println("Merging into chokepoint @" + chokepointToKeep.position +
                                         " (r=" + String.format("%.1f", chokepointToKeep.radius) + ")");

                        // Transfer connections from removed nodes to kept chokepoint
                        for (Node n : chokepointToRemove.neighbours) {
                            if (n != region) { // Don't connect to the region being removed
                                n.neighbours.remove(chokepointToRemove);
                                n.neighbours.add(chokepointToKeep);
                                chokepointToKeep.neighbours.add(n);
                            }
                        }

                        // Remove connections to the nodes being removed
                        chokepointToKeep.neighbours.remove(region);
                        chokepointToKeep.neighbours.remove(chokepointToRemove);

                        // Mark nodes for removal
                        regionsToRemove.add(region);
                        chokepointsToRemove.add(chokepointToRemove);
                        mergedCases++;
                    }
                }
            }
        }

        // Remove all marked nodes from the graph
        for (Node region : regionsToRemove) {
            graph.nodes.remove(region);
            graph.regionNodes.remove(region);
        }
        for (Node chokepoint : chokepointsToRemove) {
            graph.nodes.remove(chokepoint);
            graph.chokepointNodes.remove(chokepoint);
        }

        System.out.println("Sandwiched region cleanup complete:");
        System.out.println("- Cases merged: " + mergedCases);
        System.out.println("- Regions removed: " + regionsToRemove.size());
        System.out.println("- Chokepoints removed: " + chokepointsToRemove.size());
        System.out.println("- Final regions: " + graph.regionNodes.size());
        System.out.println("- Final chokepoints: " + graph.chokepointNodes.size());
    }

    private void extractRegionPolygons(RegionGraph graph) {
        System.out.println("Starting region polygon extraction...");

        // Step 1: Find chokepoint crossing points and build reverse lookup in contours
        HashMap<Node, Point[]> chokepointCrossings = new HashMap<>();
        graph.chokepoints.clear(); // Clear existing chokepoints

        // First, populate chokepoint contacts in contours
        populateChokepointContacts();

        int chokepointId = 0;
        for (Node chokepointNode : graph.chokepointNodes) {
            CrossingData crossingData = findChokepointCrossingPoints(chokepointNode);

            // Create Chokepoint object
            Chokepoint chokepoint = new Chokepoint(chokepointNode, chokepointId++);

            if (crossingData != null && crossingData.crossingPoints != null) {
                chokepoint.setCrossingData(crossingData.crossingPoints, crossingData.obstacleIds);
                chokepointCrossings.put(chokepointNode, crossingData.crossingPoints);

                System.out.println("Chokepoint @" + chokepointNode.position +
                                 " (r=" + String.format("%.1f", chokepointNode.radius) + ")" +
                                 " crosses obstacles " + crossingData.obstacleIds[0] + " and " + crossingData.obstacleIds[1] +
                                 " at: " + crossingData.crossingPoints[0] + " and " + crossingData.crossingPoints[1]);
            } else {
                chokepointCrossings.put(chokepointNode, null);
                System.out.println("WARNING: No crossing points found for chokepoint @" + chokepointNode.position +
                                 " (r=" + String.format("%.1f", chokepointNode.radius) + ")");
            }

            graph.chokepoints.add(chokepoint);
        }

        System.out.println("Found crossing points for " + chokepointCrossings.size() + " chokepoints");

        // Step 2: Generate walkable area (map bounds minus obstacles)
        System.out.println("Generating walkable area...");
        // For now, we'll use a simple bounding box approach
        // In a full implementation, this would involve polygon operations

        // Step 3: Generate chokepoint segments from crossing points
        System.out.println("Generating chokepoint segments...");
        HashMap<Node, Line> chokepointSegments = new HashMap<>();

        for (Node chokepoint : graph.chokepointNodes) {
            Point[] crossings = chokepointCrossings.get(chokepoint);
            if (crossings != null && crossings.length == 2) {
                // Create line segment between the two crossing points
                Line segment = Geometries.line(crossings[0].x, crossings[0].y, crossings[1].x, crossings[1].y);
                chokepointSegments.put(chokepoint, segment);

                System.out.println("Chokepoint @" + chokepoint.position +
                                 " segment: " + crossings[0] + " -> " + crossings[1]);
            }
        }

        // Step 4: Generate region boundary polygons using counterclockwise tracing
        System.out.println("Tracing region boundary polygons counterclockwise...");
        traceRegionBoundaries(graph, chokepointCrossings);

        // Step 5: Create final data structures
        System.out.println("Creating final region and chokepoint data structures...");
        createFinalDataStructures(graph, chokepointCrossings, chokepointSegments);
    }

    private void createFinalDataStructures(RegionGraph graph,
                                         HashMap<Node, Point[]> chokepointCrossings,
                                         HashMap<Node, Line> chokepointSegments) {

        System.out.println("Final BWTA2 Analysis Results:");
        System.out.println("============================");

        // Print region information
        System.out.println("\nREGIONS (" + graph.regionNodes.size() + "):");
        int regionId = 0;
        for (Node region : graph.regionNodes) {
            System.out.printf("Region R%d: center=@%s, radius=%.1f, neighbors=%d%n",
                            regionId++, region.position, region.radius, region.neighbours.size());

            // List connected chokepoints
            System.out.print("  Connected chokepoints: ");
            for (Node neighbor : region.neighbours) {
                if (graph.chokepointNodes.contains(neighbor)) {
                    System.out.print("C@" + neighbor.position + " ");
                }
            }
            System.out.println();
        }

        // Print chokepoint information
        System.out.println("\nCHOKEPOINTS (" + graph.chokepointNodes.size() + "):");
        int chokepointId = 0;
        for (Node chokepoint : graph.chokepointNodes) {
            Point[] crossings = chokepointCrossings.get(chokepoint);
            Line segment = chokepointSegments.get(chokepoint);

            System.out.printf("Chokepoint C%d: center=@%s, radius=%.1f%n",
                            chokepointId++, chokepoint.position, chokepoint.radius);

            if (crossings != null) {
                System.out.println("  Crossing points: " + crossings[0] + " -> " + crossings[1]);
            }

            // List connected regions
            System.out.print("  Connected regions: ");
            for (Node neighbor : chokepoint.neighbours) {
                if (graph.regionNodes.contains(neighbor)) {
                    System.out.print("R@" + neighbor.position + " ");
                }
            }
            System.out.println();
        }

        System.out.println("\nBWTA2 region extraction completed successfully!");
        System.out.println("Total regions: " + graph.regionNodes.size());
        System.out.println("Total chokepoints: " + graph.chokepointNodes.size());
        System.out.println("Total graph edges: " + countEdges(graph));
    }

    private int countEdges(RegionGraph graph) {
        int edges = 0;
        for (Node node : graph.nodes) {
            edges += node.neighbours.size();
        }
        return edges / 2; // Each edge is counted twice
    }

    private void traceRegionBoundaries(RegionGraph graph, HashMap<Node, Point[]> chokepointCrossings) {
        // Clear existing region list
        regionList.clear();

        int regionId = 0;
        for (Node regionNode : graph.regionNodes) {
            System.out.println("\nTracing boundary for region @" + regionNode.position + ":");

            // Create Region object
            Region region = new Region(regionNode, regionId++);

            // Find all chokepoints connected to this region
            ArrayList<Node> connectedChokepoints = new ArrayList<>();
            for (Node neighbor : regionNode.neighbours) {
                if (graph.chokepointNodes.contains(neighbor)) {
                    connectedChokepoints.add(neighbor);
                }
            }

            if (!connectedChokepoints.isEmpty()) {
                System.out.println("  Region has " + connectedChokepoints.size() + " connected chokepoints");

                // Start counterclockwise tracing from the first chokepoint
                Node startChokepoint = connectedChokepoints.get(0);
                ArrayList<Point> boundaryPoints = traceBoundaryFromChokepoint(regionNode, startChokepoint, chokepointCrossings);

                if (boundaryPoints != null && boundaryPoints.size() >= 3) {
                    region.setPolygon(boundaryPoints);
                    System.out.println("  Traced boundary with " + boundaryPoints.size() + " points");
                } else {
                    System.out.println("  Failed to trace boundary - skipping region");
                    continue;
                }
            }

            regionList.add(region);
        }
    }


    private ArrayList<Point> traceBoundaryFromChokepoint(Node regionNode, Node startChokepoint, HashMap<Node, Point[]> chokepointCrossings) {
        ArrayList<Point> boundaryPoints = new ArrayList<>();

        // Get the chokepoint crossing points
        Point[] crossings = chokepointCrossings.get(startChokepoint);
        if (crossings == null || crossings.length != 2) {
            return null;
        }

        System.out.println("    Starting boundary trace from chokepoint @" + startChokepoint.position);

        // Strategy: Build region boundary by alternating between chokepoint segments and obstacle contour portions
        // 1. Start with the chokepoint segment
        // 2. Follow obstacle contour to the next chokepoint
        // 3. Cross that chokepoint segment
        // 4. Repeat until boundary is closed

        Point currentPoint = crossings[0];
        Point startPoint = currentPoint;
        Node currentChokepoint = startChokepoint;

        int maxIterations = 20; // Safety limit on number of chokepoints to visit
        int iteration = 0;

        while (iteration < maxIterations) {
            // Add the current chokepoint segment
            Point[] currentCrossings = chokepointCrossings.get(currentChokepoint);
            if (currentCrossings == null || currentCrossings.length != 2) {
                System.out.println("    Invalid crossings for chokepoint @" + currentChokepoint.position);
                break;
            }

            // Add both crossing points of this chokepoint
            boundaryPoints.add(currentCrossings[0]);
            boundaryPoints.add(currentCrossings[1]);

            System.out.println("    Added chokepoint segment: " + currentCrossings[0] + " -> " + currentCrossings[1]);

            // Find the next chokepoint by following the contour from the second crossing point
            Node nextChokepoint = findNextChokepointAlongContour(currentCrossings[1], currentChokepoint, regionNode);

            if (nextChokepoint == null) {
                System.out.println("    Could not find next chokepoint, ending trace");
                break;
            }

            if (nextChokepoint == startChokepoint) {
                System.out.println("    Returned to start chokepoint, boundary complete");
                break;
            }

            System.out.println("    Found next chokepoint @" + nextChokepoint.position);

            // Trace along the obstacle contour to the next chokepoint
            ArrayList<Point> contourPath = traceContourBetweenChokepoints(currentCrossings[1], nextChokepoint, regionNode);
            if (contourPath != null && contourPath.size() > 1) {
                // Add contour points (skip first point as it's already added)
                for (int i = 1; i < contourPath.size(); i++) {
                    boundaryPoints.add(contourPath.get(i));
                }
                System.out.println("    Added " + (contourPath.size() - 1) + " contour points");
            }

            currentChokepoint = nextChokepoint;
            iteration++;
        }

        if (iteration >= maxIterations) {
            System.out.println("    Trace exceeded maximum iterations, using fallback");
            return createFallbackTriangle(crossings);
        }

        System.out.println("    Completed boundary trace with " + boundaryPoints.size() + " points");
        return boundaryPoints;
    }

    private ArrayList<Point> createFallbackTriangle(Point[] crossings) {
        ArrayList<Point> points = new ArrayList<>();
        points.add(crossings[0]);
        points.add(crossings[1]);
        points.add(crossings[0]); // Close the triangle
        return points;
    }

    private class ContourSegmentInfo {
        public int contourIndex;
        public int segmentIndex;
        public Contour contour;
        public double length;

        public ContourSegmentInfo(int contourIndex, int segmentIndex, Contour contour) {
            this.contourIndex = contourIndex;
            this.segmentIndex = segmentIndex;
            this.contour = contour;
            this.length = calculateSegmentLength();
        }

        private double calculateSegmentLength() {
            Point[] segmentPoints = contour.getSegmentPoints(segmentIndex);
            if (segmentPoints == null) {
                return 0.0;
            }
            return distance(segmentPoints[0], segmentPoints[1]);
        }

        public double getLength() {
            return length;
        }
    }

    private ContourSegmentInfo findContourSegment(Point crossingPoint) {
        ArrayList<Contour> contours = getSimplifiedContours();

        for (int contourIndex = 0; contourIndex < contours.size(); contourIndex++) {
            Contour contour = contours.get(contourIndex);

            for (int segmentIndex = 0; segmentIndex < contour.length() - 1; segmentIndex++) {
                Point p1 = contour.getPoint(segmentIndex);
                Point p2 = contour.getPoint(segmentIndex + 1);

                // Check if the crossing point lies on this segment
                if (isPointOnSegment(crossingPoint, p1, p2, 2.0)) {
                    return new ContourSegmentInfo(contourIndex, segmentIndex, contour);
                }
            }
        }

        return null;
    }

    private boolean isPointOnSegment(Point point, Point segmentStart, Point segmentEnd, double tolerance) {
        // Calculate distance from point to line segment
        double dx = segmentEnd.x - segmentStart.x;
        double dy = segmentEnd.y - segmentStart.y;

        if (dx == 0 && dy == 0) {
            return distance(point, segmentStart) <= tolerance;
        }

        double t = ((point.x - segmentStart.x) * dx + (point.y - segmentStart.y) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));

        double closestX = segmentStart.x + t * dx;
        double closestY = segmentStart.y + t * dy;

        double dist = Math.sqrt((point.x - closestX) * (point.x - closestX) + (point.y - closestY) * (point.y - closestY));
        return dist <= tolerance;
    }

    private Point getNextContourPoint(ContourSegmentInfo segmentInfo, Node regionNode) {
        // Move to the next point in the contour (counterclockwise relative to region)
        int nextIndex = (segmentInfo.segmentIndex + 1) % segmentInfo.contour.length();
        return segmentInfo.contour.getPoint(nextIndex);
    }

    private boolean isCloseToPoint(Point p1, Point p2, double tolerance) {
        return distance(p1, p2) <= tolerance;
    }

    private boolean isAtChokepointCrossing(Point currentPoint, HashMap<Node, Point[]> chokepointCrossings) {
        for (Point[] crossings : chokepointCrossings.values()) {
            if (crossings != null && crossings.length == 2) {
                if (isCloseToPoint(currentPoint, crossings[0], 3) ||
                    isCloseToPoint(currentPoint, crossings[1], 3)) {
                    return true;
                }
            }
        }
        return false;
    }

    private ContourSegmentInfo advanceToNextSegment(ContourSegmentInfo currentInfo) {
        int nextSegmentIndex = (currentInfo.segmentIndex + 1) % currentInfo.contour.length();
        return new ContourSegmentInfo(currentInfo.contourIndex, nextSegmentIndex, currentInfo.contour);
    }

    private void populateChokepointContacts() {
        System.out.println("Populating chokepoint contacts in contours...");
        ArrayList<Contour> contours = getSimplifiedContours();

        // Clear existing contacts
        for (Contour contour : contours) {
            contour.chokepointContacts.clear();
        }

        int contactsAdded = 0;

        // For each chokepoint, find which contours it touches and add contacts
        for (Node chokepointNode : regions.chokepointNodes) {
            CrossingData crossingData = findChokepointCrossingPoints(chokepointNode);

            if (crossingData != null && crossingData.crossingPoints != null) {
                // Process each crossing point
                for (Point crossingPoint : crossingData.crossingPoints) {
                    // Find which contour and segment contains this crossing point
                    for (int contourIndex = 0; contourIndex < contours.size(); contourIndex++) {
                        Contour contour = contours.get(contourIndex);

                        for (int segmentIndex = 0; segmentIndex < contour.length() - 1; segmentIndex++) {
                            Point p1 = contour.getPoint(segmentIndex);
                            Point p2 = contour.getPoint(segmentIndex + 1);

                            if (isPointOnSegment(crossingPoint, p1, p2, 2.0)) {
                                contour.addChokepointContact(chokepointNode, crossingPoint, segmentIndex);
                                contactsAdded++;
                                System.out.println("  Added contact: Chokepoint @" + chokepointNode.position +
                                                 " touches contour " + contourIndex + " at " + crossingPoint);
                                break; // Found the segment, no need to check others in this contour
                            }
                        }
                    }
                }
            }
        }

        System.out.println("Added " + contactsAdded + " chokepoint contacts to contours");

        // Debug: show contours with multiple chokepoint contacts
        for (int i = 0; i < contours.size(); i++) {
            Contour contour = contours.get(i);
            if (contour.getChokepointContacts().size() > 1) {
                System.out.println("Contour " + i + " has " + contour.getChokepointContacts().size() + " chokepoint contacts");
            }
        }
    }

    private Node findNextChokepointAlongContour(Point startPoint, Node excludeChokepoint, Node regionNode) {
        // Find which contour contains the start point
        ArrayList<Contour> contours = getSimplifiedContours();

        for (Contour contour : contours) {
            // Check if this contour has the start point and multiple chokepoints
            if (contour.getChokepointContacts().size() > 1) {
                boolean hasStartPoint = false;

                // Check if start point is on this contour
                for (int i = 0; i < contour.length() - 1; i++) {
                    Point p1 = contour.getPoint(i);
                    Point p2 = contour.getPoint(i + 1);
                    if (isPointOnSegment(startPoint, p1, p2, 2.0)) {
                        hasStartPoint = true;
                        break;
                    }
                }

                if (hasStartPoint) {
                    // Find the next chokepoint on this contour (excluding the current one)
                    for (Contour.ChokepointContact contact : contour.getOrderedChokepointContacts()) {
                        if (contact.chokepoint != excludeChokepoint &&
                            regionNode.neighbours.contains(contact.chokepoint)) {
                            return contact.chokepoint;
                        }
                    }
                }
            }
        }

        return null;
    }

    private ArrayList<Point> traceContourBetweenChokepoints(Point startPoint, Node targetChokepoint, Node regionNode) {
        // Find the contour containing both the start point and target chokepoint
        ArrayList<Contour> contours = getSimplifiedContours();

        for (Contour contour : contours) {
            // Find start point on contour
            int startIndex = findPointOnContour(contour, startPoint);
            if (startIndex == -1) continue;

            // Find target chokepoint contact
            Point targetContactPoint = null;
            int targetIndex = -1;
            for (Contour.ChokepointContact contact : contour.getChokepointContacts()) {
                if (contact.chokepoint == targetChokepoint) {
                    targetContactPoint = contact.contactPoint;
                    targetIndex = findPointOnContour(contour, targetContactPoint);
                    break;
                }
            }

            if (targetIndex != -1 && targetContactPoint != null) {
                System.out.println("      Tracing contour from index " + startIndex + " to index " + targetIndex);

                // Try both directions and pick the shorter path
                ArrayList<Point> clockwisePath = traceInDirection(contour, startIndex, targetIndex, true);
                ArrayList<Point> counterClockwisePath = traceInDirection(contour, startIndex, targetIndex, false);

                // Return the geometrically shorter path (by distance, not segment count)
                double clockwiseDistance = calculatePathDistance(clockwisePath);
                double counterClockwiseDistance = calculatePathDistance(counterClockwisePath);

                // Debug logging for path selection
                System.out.println("      Clockwise path: " + clockwisePath.size() + " points, distance = " + String.format("%.2f", clockwiseDistance));
                System.out.println("      Counter-clockwise path: " + counterClockwisePath.size() + " points, distance = " + String.format("%.2f", counterClockwiseDistance));

                ArrayList<Point> chosenPath = (clockwiseDistance <= counterClockwiseDistance) ?
                                             clockwisePath : counterClockwisePath;

                String chosenDirection = (clockwiseDistance <= counterClockwiseDistance) ? "clockwise" : "counter-clockwise";
                System.out.println("      Selected " + chosenDirection + " path (" + (chosenPath.size() - 1) + " segments)");

                // Debug: show first few points of each path to verify they're different
                if (clockwisePath.size() >= 2 && counterClockwisePath.size() >= 2) {
                    System.out.println("      Clockwise starts: " + clockwisePath.get(0) + " -> " + clockwisePath.get(1));
                    System.out.println("      Counter-clockwise starts: " + counterClockwisePath.get(0) + " -> " + counterClockwisePath.get(1));
                }

                return chosenPath;
            }
        }

        // Fallback: direct connection (should rarely happen now)
        System.out.println("      Could not find shared contour, using direct connection");
        ArrayList<Point> fallbackPath = new ArrayList<>();
        fallbackPath.add(startPoint);
        return fallbackPath;
    }

    // Calculate the total geometric distance of a path by summing distances between consecutive points
    private double calculatePathDistance(ArrayList<Point> path) {
        if (path.size() < 2) {
            return 0.0;
        }

        double totalDistance = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            totalDistance += distance(path.get(i), path.get(i + 1));
        }
        return totalDistance;
    }

    private int findPointOnContour(Contour contour, Point targetPoint) {
        // First try to find exact point match
        for (int i = 0; i < contour.length(); i++) {
            if (contour.getPoint(i).equals(targetPoint)) {
                return i;
            }
        }

        // If no exact match, find point on segment
        for (int i = 0; i < contour.length() - 1; i++) {
            Point p1 = contour.getPoint(i);
            Point p2 = contour.getPoint(i + 1);
            if (isPointOnSegment(targetPoint, p1, p2, 2.0)) {
                return i; // Return the segment start index
            }
        }

        return -1;
    }

    private ArrayList<Point> traceInDirection(Contour contour, int startIndex, int targetIndex, boolean clockwise) {
        ArrayList<Point> path = new ArrayList<>();
        path.add(contour.getPoint(startIndex));

        int currentIndex = startIndex;
        int contourLength = contour.length();

        // Walk around the contour until we reach the target
        while (true) {
            // Move in the specified direction
            if (clockwise) {
                currentIndex = (currentIndex + 1) % contourLength;
            } else {
                currentIndex = (currentIndex - 1 + contourLength) % contourLength;
            }

            Point currentPoint = contour.getPoint(currentIndex);
            path.add(currentPoint);

            // Stop when we reach the target index
            if (currentIndex == targetIndex) {
                break;
            }

            // Safety check - should never happen with correct logic
            if (path.size() > contourLength) {
                System.out.println("      ERROR: Path exceeded contour length - breaking");
                break;
            }
        }

        return path;
    }

    private void generateRegionPolygons(RegionGraph graph, HashMap<Node, Point[]> chokepointCrossings) {
        // Clear existing region list
        regionList.clear();

        int regionId = 0;
        for (Node regionNode : graph.regionNodes) {
            System.out.println("\nGenerating polygon for region @" + regionNode.position + ":");

            // Create Region object
            Region region = new Region(regionNode, regionId++);

            // Find all chokepoints connected to this region
            ArrayList<Node> connectedChokepoints = new ArrayList<>();
            for (Node neighbor : regionNode.neighbours) {
                if (graph.chokepointNodes.contains(neighbor)) {
                    connectedChokepoints.add(neighbor);
                }
            }

            if (connectedChokepoints.isEmpty()) {
                System.out.println("  Region has no connected chokepoints - isolated region");
                // Still create a circular polygon for isolated regions
                double radius = Math.max(20.0, regionNode.radius * 0.6); // Smaller for isolated regions
                int numPoints = 8;
                ArrayList<Point> boundaryPoints = new ArrayList<>();

                for (int i = 0; i < numPoints; i++) {
                    double angle = (2 * Math.PI * i) / numPoints;
                    int x = (int) (regionNode.position.x + radius * Math.cos(angle));
                    int y = (int) (regionNode.position.y + radius * Math.sin(angle));

                    x = Math.max(0, Math.min(width - 1, x));
                    y = Math.max(0, Math.min(height - 1, y));

                    boundaryPoints.add(new Point(x, y));
                }

                region.setPolygon(boundaryPoints);
                System.out.println("  Generated isolated region polygon with " + boundaryPoints.size() + " vertices (radius=" +
                                 String.format("%.1f", radius) + ")");
                regionList.add(region);
                continue;
            }

            // Create a simple circular boundary around the region center
            // This is a simplified approach - in a full implementation you'd use Voronoi cells
            double radius = Math.max(30.0, regionNode.radius * 0.8); // Use 80% of region radius, minimum 30 pixels
            int numPoints = 12; // Create a 12-sided polygon approximation
            ArrayList<Point> boundaryPoints = new ArrayList<>();

            for (int i = 0; i < numPoints; i++) {
                double angle = (2 * Math.PI * i) / numPoints;
                int x = (int) (regionNode.position.x + radius * Math.cos(angle));
                int y = (int) (regionNode.position.y + radius * Math.sin(angle));

                // Clamp to image boundaries
                x = Math.max(0, Math.min(width - 1, x));
                y = Math.max(0, Math.min(height - 1, y));

                boundaryPoints.add(new Point(x, y));
            }

            region.setPolygon(boundaryPoints);
            System.out.println("  Generated circular polygon with " + boundaryPoints.size() + " vertices (radius=" +
                             String.format("%.1f", radius) + ")");

            regionList.add(region);
        }
    }

    private ArrayList<Point> buildConnectedBoundary(ArrayList<ArrayList<Point>> segments) {
        if (segments.isEmpty()) {
            return null;
        }

        if (segments.size() == 1) {
            // Single segment - return as is
            return segments.get(0);
        }

        ArrayList<Point> connectedBoundary = new ArrayList<>();
        ArrayList<ArrayList<Point>> remainingSegments = new ArrayList<>(segments);

        // Start with the first segment
        ArrayList<Point> currentSegment = remainingSegments.remove(0);
        connectedBoundary.addAll(currentSegment);

        // Try to connect segments
        while (!remainingSegments.isEmpty()) {
            // Get the last point of current boundary
            Point lastPoint = connectedBoundary.get(connectedBoundary.size() - 1);

            // Find a segment that starts with this point
            ArrayList<Point> nextSegment = null;
            int nextIndex = -1;

            for (int i = 0; i < remainingSegments.size(); i++) {
                ArrayList<Point> segment = remainingSegments.get(i);
                if (!segment.isEmpty()) {
                    Point segmentStart = segment.get(0);
                    Point segmentEnd = segment.get(segment.size() - 1);

                    if (segmentStart.equals(lastPoint)) {
                        // This segment connects forward
                        nextSegment = segment;
                        nextIndex = i;
                        break;
                    } else if (segmentEnd.equals(lastPoint)) {
                        // This segment connects in reverse
                        nextSegment = new ArrayList<>(segment);
                        Collections.reverse(nextSegment);
                        nextIndex = i;
                        break;
                    }
                }
            }

            if (nextSegment == null) {
                // Can't connect more segments - check if we closed the loop
                Point firstPoint = connectedBoundary.get(0);
                if (lastPoint.equals(firstPoint) || distance(lastPoint, firstPoint) < 5.0) {
                    // Closed loop achieved
                    System.out.println("  Built closed boundary with " + connectedBoundary.size() + " points");
                    return connectedBoundary;
                }

                // Failed to connect all segments
                System.out.println("  Failed to connect segments - gap between " + lastPoint +
                                 " and remaining segments");
                return null;
            }

            // Add the next segment (skip the first point as it's already in the boundary)
            for (int i = 1; i < nextSegment.size(); i++) {
                connectedBoundary.add(nextSegment.get(i));
            }
            remainingSegments.remove(nextIndex);
        }

        // Check if we have a closed boundary
        Point firstPoint = connectedBoundary.get(0);
        Point lastPoint = connectedBoundary.get(connectedBoundary.size() - 1);
        boolean isClosed = firstPoint.equals(lastPoint) || distance(firstPoint, lastPoint) < 5.0;

        System.out.println("  Built " + (isClosed ? "closed" : "open") +
                         " boundary with " + connectedBoundary.size() + " points");
        return connectedBoundary;
    }

    private CrossingData findChokepointCrossingPoints(Node chokepoint) {
        try {
            // Use R-tree to find the two closest obstacle points to this chokepoint
            // The chokepoint should be positioned at the narrowest point between obstacles

            // Use a generous search radius - at least 100 pixels, or 10x the chokepoint radius, whichever is larger
            double searchRadius = Math.max(100.0, chokepoint.radius * 10);

            rx.Observable<Entry<Integer,Line>> nearestObstacles = simplifiedLabelTree.nearest(
                Geometries.point(chokepoint.position.x, chokepoint.position.y),
                searchRadius,
                20); // Get up to 20 nearest obstacles

            // Find the two closest points on different obstacles
            Point closest1 = null, closest2 = null;
            double minDist1 = Double.MAX_VALUE, minDist2 = Double.MAX_VALUE;
            Line obstacle1 = null, obstacle2 = null;
            Integer obstacleId1 = null, obstacleId2 = null;

            int obstacleCount = 0;
            ArrayList<Integer> foundObstacleIds = new ArrayList<>();
            for (Entry<Integer,Line> entry : nearestObstacles.toBlocking().toIterable()) {
                Line obstacle = entry.geometry();
                Integer obstacleId = entry.value();
                obstacleCount++;
                foundObstacleIds.add(obstacleId);

                // Find closest point on this line segment to the chokepoint
                Point closestPoint = getClosestPointOnLine(chokepoint.position, obstacle);
                double dist = distance(chokepoint.position, closestPoint);

                if (closest1 == null || dist < minDist1) {
                    // Found a closer point - shift previous closest to second closest
                    if (closest1 != null && !obstacle.equals(obstacle1)) {
                        closest2 = closest1;
                        minDist2 = minDist1;
                        obstacle2 = obstacle1;
                        obstacleId2 = obstacleId1;
                    }
                    closest1 = closestPoint;
                    minDist1 = dist;
                    obstacle1 = obstacle;
                    obstacleId1 = obstacleId;
                } else if (!obstacle.equals(obstacle1) && !closestPoint.equals(closest1) && (closest2 == null || dist < minDist2)) {
                    // Different obstacle than the first one, different point than first, and closer than current second closest
                    // Also check that the crossing points are sufficiently far apart (at least one chokepoint radius)
                    double crossingDistance = distance(closest1, closestPoint);
                    if (crossingDistance >= chokepoint.radius) {
                        closest2 = closestPoint;
                        minDist2 = dist;
                        obstacle2 = obstacle;
                        obstacleId2 = obstacleId;
                    }
                }
            }

            // Debug output for problematic cases
            if (closest1 != null && closest2 != null && closest1.equals(closest2)) {
                System.out.println("DEBUG: Chokepoint @" + chokepoint.position +
                                 " found IDENTICAL points from " + obstacleCount + " obstacles!");
                System.out.println("  Point1: " + closest1 + " (dist=" + String.format("%.1f", minDist1) + ")");
                System.out.println("  Point2: " + closest2 + " (dist=" + String.format("%.1f", minDist2) + ")");
                System.out.println("  Obstacles equal? " + (obstacle1 == null || obstacle2 == null ? "null" : obstacle1.equals(obstacle2)));
            }

            if (closest1 != null && closest2 != null) {
                return new CrossingData(
                    new Point[]{closest1, closest2},
                    new Integer[]{obstacleId1, obstacleId2}
                );
            } else if (closest1 != null) {
                System.out.println("DEBUG: Chokepoint @" + chokepoint.position + " (r=" + String.format("%.1f", chokepoint.radius) +
                                 ") found only one obstacle point: " + closest1 + " (dist=" + String.format("%.1f", minDist1) +
                                 ") from " + obstacleCount + " obstacles searched" +
                                 " (other crossing points may have been too close)");
            } else {
                System.out.println("DEBUG: Chokepoint @" + chokepoint.position + " (r=" + String.format("%.1f", chokepoint.radius) +
                                 ") found NO obstacle points from " + obstacleCount + " obstacles searched (search radius=" +
                                 String.format("%.1f", searchRadius) + ")");
            }

            // Log for problematic chokepoint
            if (chokepoint.position.x == 105 && chokepoint.position.y == 410) {
                System.out.println("DEBUG CROSSING: Node @" + chokepoint.position +
                                 " searched " + obstacleCount + " obstacles" +
                                 " (search radius=" + String.format("%.1f", searchRadius) + ")" +
                                 " found IDs: " + foundObstacleIds);
            }

        } catch (Exception e) {
            System.out.println("Error finding crossing points for chokepoint @" + chokepoint.position + ": " + e.getMessage());
        }

        return null;
    }

    private Point getClosestPointOnLine(Point point, Line line) {
        // Get the start and end points of the line segment
        double x1 = line.x1(), y1 = line.y1();
        double x2 = line.x2(), y2 = line.y2();
        double px = point.x, py = point.y;

        // Vector from line start to end
        double dx = x2 - x1;
        double dy = y2 - y1;

        // If line is actually a point
        if (dx == 0 && dy == 0) {
            return new Point((int)x1, (int)y1);
        }

        // Parameter t for the projection
        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);

        // Clamp t to [0, 1] to stay on the line segment
        t = Math.max(0, Math.min(1, t));

        // Calculate the closest point
        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        return new Point((int)Math.round(closestX), (int)Math.round(closestY));
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    private void splitObstacleSegmentsAtChokepointContacts(RegionGraph graph) {
        System.out.println("Splitting obstacle segments at chokepoint contact points...");

        // First, clear and populate the chokepoints list with crossing data
        graph.chokepoints.clear();
        int chokepointId = 0;

        // Create a map to track which segments need splitting and where
        HashMap<Integer, ArrayList<Point>> contourSplitPoints = new HashMap<>();

        // For each chokepoint, find its contact points with obstacles
        for (Node chokepointNode : graph.chokepointNodes) {
            CrossingData crossingData = findChokepointCrossingPoints(chokepointNode);

            // Create Chokepoint object and store it
            Chokepoint chokepoint = new Chokepoint(chokepointNode, chokepointId++);

            if (crossingData != null && crossingData.crossingPoints != null) {
                // Set the crossing data for the chokepoint
                chokepoint.setCrossingData(crossingData.crossingPoints, crossingData.obstacleIds);

                System.out.println("Chokepoint @" + chokepointNode.position +
                                 " (r=" + String.format("%.1f", chokepointNode.radius) + ")" +
                                 " crosses obstacles " + crossingData.obstacleIds[0] + " and " + crossingData.obstacleIds[1] +
                                 " at: " + crossingData.crossingPoints[0] + " and " + crossingData.crossingPoints[1]);

                // Process each crossing point for segment splitting
                for (int i = 0; i < crossingData.crossingPoints.length; i++) {
                    Point crossingPoint = crossingData.crossingPoints[i];
                    Integer obstacleId = crossingData.obstacleIds[i];

                    // Find which contour and segment this crossing point is on
                    for (int contourIndex = 0; contourIndex < simplifiedContours.size(); contourIndex++) {
                        Contour contour = simplifiedContours.get(contourIndex);

                        for (int segmentIndex = 0; segmentIndex < contour.points.size() - 1; segmentIndex++) {
                            Point p1 = contour.points.get(segmentIndex);
                            Point p2 = contour.points.get(segmentIndex + 1);

                            // Check if crossing point is on this segment
                            if (isPointOnSegment(crossingPoint, p1, p2, 2.0)) {
                                // Check if the crossing point is different from both endpoints
                                if (!crossingPoint.equals(p1) && !crossingPoint.equals(p2)) {
                                    // Mark this segment for splitting
                                    if (!contourSplitPoints.containsKey(contourIndex)) {
                                        contourSplitPoints.put(contourIndex, new ArrayList<>());
                                    }

                                    // Add the split point if not already in the list
                                    boolean alreadyAdded = false;
                                    for (Point existingPoint : contourSplitPoints.get(contourIndex)) {
                                        if (existingPoint.equals(crossingPoint)) {
                                            alreadyAdded = true;
                                            break;
                                        }
                                    }

                                    if (!alreadyAdded) {
                                        contourSplitPoints.get(contourIndex).add(crossingPoint);
                                        System.out.println("  Will split contour " + contourIndex +
                                                         " segment [" + segmentIndex + "->" + (segmentIndex+1) + "] " +
                                                         "at point " + crossingPoint +
                                                         " (chokepoint @" + chokepointNode.position + ")");
                                    }
                                }
                                break; // Found the segment, no need to check others in this contour
                            }
                        }
                    }
                }
            } else {
                System.out.println("WARNING: No crossing points found for chokepoint @" + chokepointNode.position +
                                 " (r=" + String.format("%.1f", chokepointNode.radius) + ")");
            }

            // Add the chokepoint to the graph
            graph.chokepoints.add(chokepoint);
        }

        System.out.println("Found crossing points for " + graph.chokepoints.size() + " chokepoints");

        // Now perform the actual splitting
        int totalSplits = 0;
        for (Map.Entry<Integer, ArrayList<Point>> entry : contourSplitPoints.entrySet()) {
            Integer contourIndex = entry.getKey();
            ArrayList<Point> splitPoints = entry.getValue();

            if (splitPoints.size() > 0) {
                Contour originalContour = simplifiedContours.get(contourIndex);
                ArrayList<Point> newPoints = new ArrayList<>();

                // Iterate through original segments
                for (int i = 0; i < originalContour.points.size() - 1; i++) {
                    Point p1 = originalContour.points.get(i);
                    Point p2 = originalContour.points.get(i + 1);

                    // Add the start point
                    newPoints.add(p1);

                    // Check if any split points are on this segment
                    ArrayList<Point> segmentSplits = new ArrayList<>();
                    for (Point splitPoint : splitPoints) {
                        if (isPointOnSegment(splitPoint, p1, p2, 2.0) &&
                            !splitPoint.equals(p1) && !splitPoint.equals(p2)) {
                            segmentSplits.add(splitPoint);
                        }
                    }

                    // Sort split points by distance from p1 if there are multiple on same segment
                    if (segmentSplits.size() > 1) {
                        segmentSplits.sort((a, b) -> {
                            double distA = distance(p1, a);
                            double distB = distance(p1, b);
                            return Double.compare(distA, distB);
                        });
                    }

                    // Add all split points in order
                    for (Point splitPoint : segmentSplits) {
                        newPoints.add(splitPoint);
                        totalSplits++;
                    }
                }

                // Add the last point
                newPoints.add(originalContour.points.get(originalContour.points.size() - 1));

                // Update the contour with new points
                originalContour.points = newPoints;
                System.out.println("  Contour " + contourIndex + " updated: " +
                                 (originalContour.points.size() - splitPoints.size()) +
                                 " points -> " + originalContour.points.size() + " points");
            }
        }

        System.out.println("Split " + totalSplits + " obstacle segments at chokepoint contact points");

        // Rebuild the R-tree with the updated contours
        simplifiedLabelTree = createSimplifiedRTree();
    }

    private void buildRegionBoundariesFromChokepoints(RegionGraph graph) {
        System.out.println("Building region boundaries from chokepoint contacts...");

        // Clear existing region list
        regionList.clear();

        int regionId = 0;

        // Iterate through each region
        for (Node regionNode : graph.regionNodes) {
            System.out.println("\nBuilding boundary for region @" + regionNode.position + ":");

            // Create Region object
            Region region = new Region(regionNode, regionId++);
            ArrayList<Point> boundaryPoints = new ArrayList<>();

            // Find all chokepoints connected to this region
            ArrayList<Node> connectedChokepoints = new ArrayList<>();
            for (Node neighbor : regionNode.neighbours) {
                if (graph.chokepointNodes.contains(neighbor)) {
                    connectedChokepoints.add(neighbor);
                    region.chokepoints.add(neighbor);
                }
            }

            if (connectedChokepoints.isEmpty()) {
                // Isolated region with no chokepoints - skip for now
                System.out.println("  Isolated region - skipping (no boundary generation)");
                continue;
            }

            System.out.println("  Found " + connectedChokepoints.size() + " connected chokepoints");

            // Group chokepoint contacts by obstacle ID
            HashMap<Integer, ArrayList<ChokepointContact>> obstacleContacts = new HashMap<>();

            for (Node chokepointNode : connectedChokepoints) {
                // Find the Chokepoint object for this node
                Chokepoint chokepoint = null;
                for (Chokepoint cp : graph.chokepoints) {
                    if (cp.node == chokepointNode) {
                        chokepoint = cp;
                        break;
                    }
                }

                if (chokepoint != null && chokepoint.crossingPoints != null) {
                    // Add both contact points to the obstacle map
                    for (int i = 0; i < chokepoint.crossingPoints.length; i++) {
                        Integer obstacleId = chokepoint.obstacleIds[i];
                        Point contactPoint = chokepoint.crossingPoints[i];

                        if (!obstacleContacts.containsKey(obstacleId)) {
                            obstacleContacts.put(obstacleId, new ArrayList<>());
                        }

                        obstacleContacts.get(obstacleId).add(
                            new ChokepointContact(chokepointNode, contactPoint, obstacleId)
                        );
                    }
                }
            }

            // Track which segments we've already added to avoid duplicates
            HashSet<String> addedSegments = new HashSet<>();

            // Store path segments separately to preserve connectivity
            ArrayList<ArrayList<Point>> pathSegments = new ArrayList<>();

            // For each obstacle that this region touches
            for (Map.Entry<Integer, ArrayList<ChokepointContact>> entry : obstacleContacts.entrySet()) {
                Integer obstacleId = entry.getKey();
                ArrayList<ChokepointContact> contacts = entry.getValue();

                if (contacts.size() < 2) {
                    System.out.println("  WARNING: Obstacle " + obstacleId + " has only " +
                                     contacts.size() + " contact point(s), skipping");
                    continue;
                }

                System.out.println("  Processing obstacle " + obstacleId + " with " +
                                 contacts.size() + " contact points");

                // Find the contour for this obstacle
                Contour obstacleContour = null;
                int contourIndex = -1;

                // Map obstacle IDs to contour indices
                if (obstacleId >= 1000) {
                    // Simplified contour obstacle IDs start at 1000
                    contourIndex = obstacleId - 1000;
                    if (contourIndex < simplifiedContours.size()) {
                        obstacleContour = simplifiedContours.get(contourIndex);
                    }
                } else if (obstacleId < 4) {
                    // Map boundary IDs (0-3) - these don't have contours
                    System.out.println("    Skipping boundary obstacle " + obstacleId);
                    continue;
                }

                if (obstacleContour == null) {
                    System.out.println("    WARNING: Could not find contour for obstacle " + obstacleId);
                    continue;
                }

                // For each pair of contact points on this obstacle
                for (int i = 0; i < contacts.size(); i++) {
                    for (int j = i + 1; j < contacts.size(); j++) {
                        ChokepointContact contact1 = contacts.get(i);
                        ChokepointContact contact2 = contacts.get(j);

                        // Find the shortest path along the obstacle boundary
                        ArrayList<Point> pathSegment = findShortestPathAlongContour(
                            obstacleContour, contact1.contactPoint, contact2.contactPoint, addedSegments
                        );

                        if (pathSegment != null && !pathSegment.isEmpty()) {
                            pathSegments.add(pathSegment);
                            System.out.println("    Added path from " + contact1.contactPoint +
                                             " to " + contact2.contactPoint +
                                             " (" + pathSegment.size() + " points)");
                        }
                    }
                }
            }

            // Add chokepoint crossing segments to connect obstacle segments
            for (Node chokepointNode : connectedChokepoints) {
                // Find the Chokepoint object for this node
                Chokepoint chokepoint = null;
                for (Chokepoint cp : graph.chokepoints) {
                    if (cp.node == chokepointNode) {
                        chokepoint = cp;
                        break;
                    }
                }

                if (chokepoint != null && chokepoint.crossingPoints != null && chokepoint.crossingPoints.length >= 2) {
                    // Add the chokepoint crossing segment
                    ArrayList<Point> chokepointSegment = new ArrayList<>();
                    chokepointSegment.add(chokepoint.crossingPoints[0]);
                    chokepointSegment.add(chokepoint.crossingPoints[1]);
                    pathSegments.add(chokepointSegment);
                    System.out.println("    Added chokepoint segment from " +
                                     chokepoint.crossingPoints[0] + " to " + chokepoint.crossingPoints[1] + " (2 points)");
                }
            }

            // Build connected boundary from segments
            ArrayList<Point> connectedBoundary = buildConnectedBoundary(pathSegments);
            if (connectedBoundary != null) {
                boundaryPoints = connectedBoundary;
            } else {
                System.out.println("  WARNING: Could not build connected boundary from segments");
                boundaryPoints.clear();
            }


            // Set the boundary polygon for the region
            if (!boundaryPoints.isEmpty()) {
                // Use boundary points directly - they're already in the correct order
                // from tracing along obstacle contours
                region.setPolygon(boundaryPoints);

                // Check if the boundary forms a closed polygon
                // A boundary is considered closed if consecutive points are connected
                // (no large gaps between them)
                boolean isClosed = checkBoundaryConnectivity(boundaryPoints);
                region.isClosed = isClosed;

                System.out.println("  Generated region boundary with " + boundaryPoints.size() +
                                 " points (" + (isClosed ? "CLOSED" : "OPEN") + ")");
            } else {
                // No valid boundary paths found - skip this region
                System.out.println("  No valid boundary paths found - skipping region");
                continue;
            }

            regionList.add(region);
        }

        System.out.println("\nGenerated boundaries for " + regionList.size() + " regions");

        // Run diagnostic on region boundaries to identify jagged segments
        diagnoseBoundarySegments(graph);
    }

    private void testObstacleContourClosedness() {
        System.out.println("\n=== Testing Obstacle Contour Closedness ===");

        ArrayList<Contour> contours = getSimplifiedContours();

        for (int i = 0; i < Math.min(3, contours.size()); i++) {
            Contour contour = contours.get(i);
            System.out.println("Testing contour " + i + " with " + contour.length() + " points");

            // Create a boundary by tracing the entire contour
            ArrayList<Point> contourBoundary = new ArrayList<>();
            for (int j = 0; j < contour.length(); j++) {
                contourBoundary.add(contour.getPoint(j));
            }

            // Test if it's identified as closed using our connectivity check
            boolean isClosed = checkBoundaryConnectivity(contourBoundary);
            System.out.println("  Contour " + i + " closedness check: " + (isClosed ? "CLOSED" : "OPEN"));

            // Also test our bidirectional tracing on this contour
            if (contour.length() > 2) {
                ArrayList<Point> clockwisePath = traceInDirection(contour, 0, contour.length() / 2, true);
                ArrayList<Point> counterClockwisePath = traceInDirection(contour, 0, contour.length() / 2, false);

                System.out.println("  Bidirectional tracing test:");
                System.out.println("    Clockwise path: " + clockwisePath.size() + " points");
                System.out.println("    Counter-clockwise path: " + counterClockwisePath.size() + " points");
                System.out.println("    Expected: " + (contour.length() / 2 + 1) + " points each direction");

                // Check if both paths have reasonable lengths
                int expectedLength = contour.length() / 2 + 1;
                boolean clockwiseOK = Math.abs(clockwisePath.size() - expectedLength) <= 2;
                boolean counterClockwiseOK = Math.abs(counterClockwisePath.size() - expectedLength) <= 2;

                System.out.println("    Bidirectional tracing: " +
                    (clockwiseOK && counterClockwiseOK ? "PASS" : "FAIL"));
            }
        }
        System.out.println("=== End Obstacle Contour Test ===\n");
    }

    private void diagnoseBoundarySegments(RegionGraph graph) {
        System.out.println("\n=== BOUNDARY SEGMENT DIAGNOSTICS ===");

        // Clear previous jagged segments
        jaggedSegments.clear();

        System.out.println("Using strict exact-match validation for segment analysis...");

        int totalSegments = 0;
        int validSegments = 0;
        int jaggedSegments = 0;

        // Check each region's boundary segments
        for (int regionIndex = 0; regionIndex < regionList.size(); regionIndex++) {
            Region region = regionList.get(regionIndex);
            ArrayList<Point> boundary = region.polygon;

            if (boundary == null || boundary.size() < 2) {
                continue;
            }

            System.out.println("\nRegion R" + region.id + " @" + region.seedNode.position + ":");
            System.out.println("  Boundary has " + boundary.size() + " points, " + (boundary.size() - 1) + " segments");

            ArrayList<String> jaggedSegmentsList = new ArrayList<>();
            int regionJaggedCount = 0;

            for (int i = 0; i < boundary.size() - 1; i++) {
                Point p1 = boundary.get(i);
                Point p2 = boundary.get(i + 1);
                String segmentKey = createSegmentKey(p1, p2);

                totalSegments++;

                boolean isChokepointSegment = isValidChokepointSegment(p1, p2, graph);
                boolean isObstacleSegment = isValidObstacleSegment(p1, p2);

                String validationType = "";
                if (isChokepointSegment) {
                    validSegments++;
                    validationType = "chokepoint";
                    System.out.println("      VALID (chokepoint): " + p1 + " -> " + p2 + " (length=" + String.format("%.1f", distance(p1, p2)) + ")");
                } else if (isObstacleSegment) {
                    validSegments++;
                    validationType = "obstacle";
                    System.out.println("      VALID (obstacle): " + p1 + " -> " + p2 + " (length=" + String.format("%.1f", distance(p1, p2)) + ")");
                } else {
                    jaggedSegments++;
                    regionJaggedCount++;
                    double segmentLength = distance(p1, p2);

                    String jaggedId = "R" + region.id + ".J" + regionJaggedCount;
                    jaggedSegmentsList.add(jaggedId + ": " + p1 + " -> " + p2 + " (length=" + String.format("%.1f", segmentLength) + ")");

                    // Store for visualization (using red color)
                    this.jaggedSegments.add(new JaggedSegment(p1, p2, region.id, jaggedId, Color.RED));

                    // Debug logging for jagged segments
                    System.out.println("      JAGGED " + jaggedId + ": " + p1 + " -> " + p2 + " (length=" + String.format("%.1f", segmentLength) + ")");
                }
            }

            if (!jaggedSegmentsList.isEmpty()) {
                System.out.println("  *** JAGGED SEGMENTS DETECTED (" + jaggedSegmentsList.size() + ") ***:");
                for (String jaggedSegment : jaggedSegmentsList) {
                    System.out.println("    " + jaggedSegment);
                }
            } else {
                System.out.println("  All segments are valid (chokepoint or obstacle segments)");
            }
        }

        System.out.println("\n=== DIAGNOSTIC SUMMARY ===");
        System.out.println("Total boundary segments: " + totalSegments);
        System.out.println("Valid segments: " + validSegments + " (" + String.format("%.1f", 100.0 * validSegments / totalSegments) + "%)");
        System.out.println("Jagged segments: " + jaggedSegments + " (" + String.format("%.1f", 100.0 * jaggedSegments / totalSegments) + "%)");
        System.out.println("=== END DIAGNOSTICS ===\n");
    }

    // Enhanced chokepoint segment validation - checks if segment contains chokepoint crossing points
    private boolean isValidChokepointSegment(Point p1, Point p2, RegionGraph graph) {
        // Use strict exact matching for chokepoint segments
        for (Chokepoint chokepoint : graph.chokepoints) {
            if (chokepoint.crossingPoints != null && chokepoint.crossingPoints.length >= 2) {
                Point cp1 = chokepoint.crossingPoints[0];
                Point cp2 = chokepoint.crossingPoints[1];

                // Check if the segment is exactly the chokepoint segment (either direction)
                if ((p1.equals(cp1) && p2.equals(cp2)) || (p1.equals(cp2) && p2.equals(cp1))) {
                    return true;
                }
            }
        }
        return false;
    }

    // Strict obstacle segment validation - only exact matches
    private boolean isValidObstacleSegment(Point p1, Point p2) {
        ArrayList<Contour> contours = getSimplifiedContours();

        // Check if this segment is a direct obstacle segment (exact match only)
        for (Contour contour : contours) {
            for (int i = 0; i < contour.length(); i++) {
                Point cp1 = contour.getPoint(i);
                Point cp2 = contour.getPoint((i + 1) % contour.length());

                if ((p1.equals(cp1) && p2.equals(cp2)) || (p1.equals(cp2) && p2.equals(cp1))) {
                    return true;
                }
            }
        }
        return false;
    }

    // Check if a segment follows a path along a contour (spanning multiple contour segments)
    private boolean segmentFollowsContourPath(Point segStart, Point segEnd, Contour contour) {
        double tolerance = 2.0;

        // Find starting point on contour
        int startIndex = -1;
        for (int i = 0; i < contour.length(); i++) {
            Point cp = contour.getPoint(i);
            if (distance(segStart, cp) <= tolerance) {
                startIndex = i;
                break;
            }
        }

        if (startIndex == -1) return false;

        // Trace along contour to see if we reach the end point
        double maxDistance = distance(segStart, segEnd) * 1.5; // Allow some slack
        double accumulatedDistance = 0;

        // Try both directions
        for (boolean clockwise : new boolean[]{true, false}) {
            accumulatedDistance = 0;
            int current = startIndex;

            for (int steps = 0; steps < contour.length() && accumulatedDistance < maxDistance; steps++) {
                Point currentPoint = contour.getPoint(current);

                // Check if we've reached the end point
                if (distance(segEnd, currentPoint) <= tolerance) {
                    // Verify the accumulated path distance is reasonable
                    double directDistance = distance(segStart, segEnd);
                    if (accumulatedDistance <= directDistance * 2.0) { // Path shouldn't be too winding
                        return true;
                    }
                }

                // Move to next point
                int next = clockwise ? (current + 1) % contour.length()
                                    : (current - 1 + contour.length()) % contour.length();

                if (steps > 0) { // Don't add distance for the first step
                    accumulatedDistance += distance(contour.getPoint(current), contour.getPoint(next));
                }
                current = next;
            }
        }

        return false;
    }

    private ArrayList<Point> findShortestPathAlongContour(
            Contour contour, Point start, Point end, HashSet<String> addedSegments) {

        ArrayList<Point> path = new ArrayList<>();

        // Find indices of start and end points in the contour
        int startIndex = -1;
        int endIndex = -1;

        // Check if points are vertices or need to find nearest segments
        for (int i = 0; i < contour.points.size(); i++) {
            Point p = contour.points.get(i);
            if (p.equals(start)) {
                startIndex = i;
            }
            if (p.equals(end)) {
                endIndex = i;
            }
        }

        if (startIndex == -1 || endIndex == -1) {
            // Points are not vertices, find nearest segments
            System.out.println("      WARNING: Contact points not found as vertices in contour");
            return path;
        }

        // Calculate both possible paths (clockwise and counter-clockwise)
        ArrayList<Point> pathCW = new ArrayList<>();
        ArrayList<Point> pathCCW = new ArrayList<>();

        // Clockwise path
        int current = startIndex;
        while (current != endIndex) {
            pathCW.add(contour.points.get(current));
            current = (current + 1) % contour.points.size();
        }
        pathCW.add(contour.points.get(endIndex));

        // Counter-clockwise path
        current = startIndex;
        while (current != endIndex) {
            pathCCW.add(contour.points.get(current));
            current = (current - 1 + contour.points.size()) % contour.points.size();
        }
        pathCCW.add(contour.points.get(endIndex));

        // Choose the geometrically shorter path (by distance, not segment count)
        double clockwiseDistance = calculatePathDistance(pathCW);
        double counterClockwiseDistance = calculatePathDistance(pathCCW);

        // Debug logging for path selection
        System.out.println("      Clockwise path: " + pathCW.size() + " points, distance = " + String.format("%.2f", clockwiseDistance));
        System.out.println("      Counter-clockwise path: " + pathCCW.size() + " points, distance = " + String.format("%.2f", counterClockwiseDistance));

        ArrayList<Point> shortestPath = (clockwiseDistance <= counterClockwiseDistance) ? pathCW : pathCCW;

        String chosenDirection = (clockwiseDistance <= counterClockwiseDistance) ? "clockwise" : "counter-clockwise";
        System.out.println("      Selected " + chosenDirection + " path (" + (shortestPath.size() - 1) + " segments)");

        // Mark segments as added
        for (int i = 0; i < shortestPath.size() - 1; i++) {
            Point p1 = shortestPath.get(i);
            Point p2 = shortestPath.get(i + 1);
            String segmentKey = createSegmentKey(p1, p2);
            addedSegments.add(segmentKey);
        }

        return shortestPath;
    }

    private String createSegmentKey(Point p1, Point p2) {
        // Create a unique key for a segment (order-independent)
        Point first = (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)) ? p1 : p2;
        Point second = (first == p1) ? p2 : p1;
        return first.x + "," + first.y + "-" + second.x + "," + second.y;
    }

    private boolean checkBoundaryConnectivity(ArrayList<Point> boundary) {
        if (boundary.size() < 3) {
            System.out.println("    DEBUG: Boundary has only " + boundary.size() + " points, need at least 3 for closed polygon");
            return false;  // Need at least 3 points for a polygon
        }

        System.out.println("    DEBUG: Checking connectivity for " + boundary.size() + " point boundary:");
        for (int i = 0; i < boundary.size(); i++) {
            Point current = boundary.get(i);
            Point next = boundary.get((i + 1) % boundary.size());
            System.out.println("      Segment " + i + ": " + current + " -> " + next);
        }

        // Create a proper graph-based connectivity check
        // For a closed boundary, each point should appear exactly twice in the edge graph:
        // once as a start point and once as an end point
        java.util.HashMap<Point, Integer> pointDegree = new java.util.HashMap<>();

        // Count how many times each point appears as part of an edge
        for (int i = 0; i < boundary.size(); i++) {
            Point current = boundary.get(i);
            Point next = boundary.get((i + 1) % boundary.size());

            // Only count if points are not identical (avoid self-loops)
            if (current != next) {
                pointDegree.put(current, pointDegree.getOrDefault(current, 0) + 1);
                pointDegree.put(next, pointDegree.getOrDefault(next, 0) + 1);
            } else {
                System.out.println("    DEBUG: Skipping self-loop at " + current);
            }
        }

        System.out.println("    DEBUG: Point degrees:");
        boolean allCorrect = true;
        for (java.util.Map.Entry<Point, Integer> entry : pointDegree.entrySet()) {
            System.out.println("      " + entry.getKey() + " -> degree " + entry.getValue());
            if (entry.getValue() != 2) {
                System.out.println("        ERROR: Expected degree 2, got " + entry.getValue());
                allCorrect = false;
            }
        }

        if (!allCorrect) {
            return false;
        }

        // Additional check: ensure we can traverse the entire boundary
        boolean traversable = canTraverseBoundary(boundary);
        System.out.println("    DEBUG: Boundary traversable: " + traversable);
        return traversable;
    }

    private boolean canTraverseBoundary(ArrayList<Point> boundary) {
        if (boundary.isEmpty()) {
            System.out.println("    DEBUG TRAVERSE: Empty boundary");
            return false;
        }

        System.out.println("    DEBUG TRAVERSE: Checking traversability for boundary with " + boundary.size() + " points");

        // Build adjacency map using point identity (==)
        java.util.HashMap<Point, java.util.ArrayList<Point>> adjacency = new java.util.HashMap<>();

        for (int i = 0; i < boundary.size(); i++) {
            Point current = boundary.get(i);
            Point next = boundary.get((i + 1) % boundary.size());

            if (current != next) { // Use identity comparison, not equals
                adjacency.computeIfAbsent(current, k -> new java.util.ArrayList<>()).add(next);
                adjacency.computeIfAbsent(next, k -> new java.util.ArrayList<>()).add(current);
            } else {
                System.out.println("    DEBUG TRAVERSE: Skipping self-loop at " + current);
            }
        }

        System.out.println("    DEBUG TRAVERSE: Adjacency map:");
        for (java.util.Map.Entry<Point, java.util.ArrayList<Point>> entry : adjacency.entrySet()) {
            System.out.println("      " + entry.getKey() + " -> " + entry.getValue());
        }

        // For a closed polygon, we need to check if we can traverse the boundary as a cycle
        // Start from first point and try to follow the boundary sequence
        Point start = boundary.get(0);
        boolean canTraverseAsCycle = canTraverseSequentialBoundary(boundary, adjacency);
        System.out.println("    DEBUG TRAVERSE: Can traverse as sequential cycle: " + canTraverseAsCycle);

        return canTraverseAsCycle;
    }

    private boolean canTraverseSequentialBoundary(ArrayList<Point> boundary,
                                                 java.util.HashMap<Point, java.util.ArrayList<Point>> adjacency) {
        if (boundary.size() < 3) {
            System.out.println("    DEBUG TRAVERSE: Boundary too small for polygon");
            return false;
        }

        // Check if we can follow the exact boundary sequence
        for (int i = 0; i < boundary.size(); i++) {
            Point current = boundary.get(i);
            Point next = boundary.get((i + 1) % boundary.size());

            if (current == next) {
                System.out.println("    DEBUG TRAVERSE: Self-loop at " + current + ", skipping");
                continue; // Skip self-loops
            }

            java.util.ArrayList<Point> neighbors = adjacency.get(current);
            if (neighbors == null) {
                System.out.println("    DEBUG TRAVERSE: No neighbors for " + current);
                return false;
            }

            boolean foundNext = false;
            for (Point neighbor : neighbors) {
                if (neighbor == next) { // Use identity comparison
                    foundNext = true;
                    break;
                }
            }

            if (!foundNext) {
                System.out.println("    DEBUG TRAVERSE: Cannot reach " + next + " from " + current);
                System.out.println("    DEBUG TRAVERSE: Available neighbors: " + neighbors);
                return false;
            }
        }

        System.out.println("    DEBUG TRAVERSE: All sequential connections verified");
        return true;
    }

    private boolean traverseFrom(Point current, java.util.HashMap<Point, java.util.ArrayList<Point>> adjacency,
                                java.util.HashSet<Point> visited, int expectedSize) {
        visited.add(current);

        if (visited.size() >= expectedSize) {
            return true; // Successfully visited all unique points
        }

        java.util.ArrayList<Point> neighbors = adjacency.get(current);
        if (neighbors == null) return false;

        for (Point neighbor : neighbors) {
            if (!visited.contains(neighbor)) {
                if (traverseFrom(neighbor, adjacency, visited, expectedSize)) {
                    return true;
                }
            }
        }

        return visited.size() >= expectedSize;
    }

    // Helper class to store jagged segment information for debug visualization
    public static class JaggedSegment {
        public Point start;
        public Point end;
        public int regionId;
        public String identifier;
        public Color regionColor;
        public double length;

        public JaggedSegment(Point start, Point end, int regionId, String identifier, Color regionColor) {
            this.start = start;
            this.end = end;
            this.regionId = regionId;
            this.identifier = identifier;
            this.regionColor = regionColor;
            this.length = Math.sqrt((end.x - start.x) * (end.x - start.x) +
                                   (end.y - start.y) * (end.y - start.y));
        }
    }

    // Helper class to store chokepoint contact information
    private static class ChokepointContact {
        Node chokepointNode;
        Point contactPoint;
        Integer obstacleId;

        ChokepointContact(Node chokepointNode, Point contactPoint, Integer obstacleId) {
            this.chokepointNode = chokepointNode;
            this.contactPoint = contactPoint;
            this.obstacleId = obstacleId;
        }
    }

    // Map edge enumeration
    public enum MapEdge {
        TOP, BOTTOM, LEFT, RIGHT
    }

    // Helper class to store map edge contact information
    public static class MapEdgeContact {
        public Point contactPoint;
        public MapEdge edge;
        public Integer obstacleId;
        public Node chokepointNode;
        public boolean isObstacle;
        public boolean isChokepoint;

        // For obstacle contacts
        public MapEdgeContact(Point contactPoint, MapEdge edge, Integer obstacleId) {
            this.contactPoint = contactPoint;
            this.edge = edge;
            this.obstacleId = obstacleId;
            this.isObstacle = true;
            this.isChokepoint = false;
        }

        // For chokepoint contacts
        public MapEdgeContact(Point contactPoint, MapEdge edge, Node chokepointNode) {
            this.contactPoint = contactPoint;
            this.edge = edge;
            this.chokepointNode = chokepointNode;
            this.isObstacle = false;
            this.isChokepoint = true;
        }
    }

    private void detectMapEdgeContacts() {
        System.out.println("Detecting map edge contacts...");
        mapEdgeContacts.clear();

        // Define map edges as line segments
        Point[] topEdge = {new Point(0, 0), new Point(width-1, 0)};
        Point[] bottomEdge = {new Point(0, height-1), new Point(width-1, height-1)};
        Point[] leftEdge = {new Point(0, 0), new Point(0, height-1)};
        Point[] rightEdge = {new Point(width-1, 0), new Point(width-1, height-1)};

        // Check obstacle segments against map edges
        detectObstacleMapEdgeContacts(topEdge, MapEdge.TOP);
        detectObstacleMapEdgeContacts(bottomEdge, MapEdge.BOTTOM);
        detectObstacleMapEdgeContacts(leftEdge, MapEdge.LEFT);
        detectObstacleMapEdgeContacts(rightEdge, MapEdge.RIGHT);

        // Check chokepoint crossing segments against map edges
        detectChokepointMapEdgeContacts(topEdge, MapEdge.TOP);
        detectChokepointMapEdgeContacts(bottomEdge, MapEdge.BOTTOM);
        detectChokepointMapEdgeContacts(leftEdge, MapEdge.LEFT);
        detectChokepointMapEdgeContacts(rightEdge, MapEdge.RIGHT);

        System.out.println("Found " + mapEdgeContacts.size() + " map edge contacts");
    }

    private void detectObstacleMapEdgeContacts(Point[] mapEdge, MapEdge edge) {
        for (int contourIndex = 0; contourIndex < simplifiedContours.size(); contourIndex++) {
            Contour contour = simplifiedContours.get(contourIndex);
            int obstacleId = 1000 + contourIndex;

            // Check each segment of this obstacle contour
            for (int i = 0; i < contour.length(); i++) {
                Point p1 = contour.getPoint(i);
                Point p2 = contour.getPoint((i + 1) % contour.length());

                // Check if this obstacle segment intersects with the map edge
                Point intersection = getLineIntersection(p1, p2, mapEdge[0], mapEdge[1]);
                if (intersection != null) {
                    // Found an intersection - this is a map edge contact
                    MapEdgeContact contact = new MapEdgeContact(intersection, edge, obstacleId);
                    mapEdgeContacts.add(contact);
                    System.out.println("  Obstacle " + obstacleId + " contacts " + edge + " edge at " + intersection);
                }
            }
        }
    }

    private void detectChokepointMapEdgeContacts(Point[] mapEdge, MapEdge edge) {
        if (regions == null || regions.chokepoints == null) return;

        for (Chokepoint chokepoint : regions.chokepoints) {
            if (chokepoint.crossingPoints != null && chokepoint.crossingPoints.length >= 2) {
                Point p1 = chokepoint.crossingPoints[0];
                Point p2 = chokepoint.crossingPoints[1];

                // Check if this chokepoint crossing segment intersects with the map edge
                Point intersection = getLineIntersection(p1, p2, mapEdge[0], mapEdge[1]);
                if (intersection != null) {
                    // Found an intersection - this is a map edge contact
                    MapEdgeContact contact = new MapEdgeContact(intersection, edge, chokepoint.node);
                    mapEdgeContacts.add(contact);
                    System.out.println("  Chokepoint @" + chokepoint.node.position + " contacts " + edge + " edge at " + intersection);
                }
            }
        }
    }

    // Line intersection algorithm - returns intersection point or null if no intersection
    private Point getLineIntersection(Point p1, Point p2, Point p3, Point p4) {
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;
        double x3 = p3.x, y3 = p3.y;
        double x4 = p4.x, y4 = p4.y;

        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denom) < 1e-10) return null; // Lines are parallel

        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            // Lines intersect within both segments
            double x = x1 + t * (x2 - x1);
            double y = y1 + t * (y2 - y1);
            return new Point((int)Math.round(x), (int)Math.round(y));
        }

        return null; // No intersection within segments
    }

    private void skipUnmarkedVertices(RegionGraph graph) {
        System.out.println("Skipping through unmarked vertices to connect marked nodes directly...");

        // Find all unmarked nodes
        HashSet<Node> unmarkedNodes = new HashSet<>();
        HashSet<Node> markedNodes = new HashSet<>();

        for (Node n : graph.nodes) {
            if (!graph.regionNodes.contains(n) && !graph.chokepointNodes.contains(n)) {
                unmarkedNodes.add(n);
            } else {
                markedNodes.add(n);
            }
        }

        System.out.println("Found " + unmarkedNodes.size() + " unmarked nodes to remove");
        System.out.println("Preserving " + markedNodes.size() + " marked nodes");

        // Build new adjacency relationships
        HashMap<Node, HashSet<Node>> newConnections = new HashMap<>();
        for (Node marked : markedNodes) {
            newConnections.put(marked, new HashSet<>());
        }

        // For each marked node, find all other marked nodes it can reach through unmarked nodes
        for (Node marked : markedNodes) {
            // BFS to find all reachable marked nodes through unmarked paths
            HashSet<Node> visited = new HashSet<>();
            Queue<Node> toVisit = new LinkedList<>();

            visited.add(marked);

            // Start BFS from all neighbors of this marked node
            for (Node neighbor : marked.neighbours) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    if (markedNodes.contains(neighbor)) {
                        // Direct connection to another marked node - preserve it
                        newConnections.get(marked).add(neighbor);
                        newConnections.get(neighbor).add(marked);
                    } else if (unmarkedNodes.contains(neighbor)) {
                        // Start searching through unmarked nodes
                        toVisit.add(neighbor);
                    }
                }
            }

            // Continue BFS through unmarked nodes to find marked nodes
            while (!toVisit.isEmpty()) {
                Node current = toVisit.poll();

                for (Node neighbor : current.neighbours) {
                    if (!visited.contains(neighbor)) {
                        visited.add(neighbor);

                        if (markedNodes.contains(neighbor)) {
                            // Found a marked node through unmarked path - create direct connection
                            newConnections.get(marked).add(neighbor);
                            newConnections.get(neighbor).add(marked);
                            // Don't continue from marked nodes - we handle them separately
                        } else if (unmarkedNodes.contains(neighbor)) {
                            // Continue searching through unmarked nodes
                            toVisit.add(neighbor);
                        }
                    }
                }
            }
        }

        // Apply the new connections
        for (Node marked : markedNodes) {
            marked.neighbours.clear();
            marked.neighbours.addAll(newConnections.get(marked));
        }

        // Remove all unmarked nodes from the graph
        for (Node unmarked : unmarkedNodes) {
            graph.nodes.remove(unmarked);
        }

        System.out.println("Unmarked vertex skipping complete:");
        System.out.println("- Total unmarked nodes removed: " + unmarkedNodes.size());
        System.out.println("- Remaining nodes: " + graph.nodes.size());
        System.out.println("- Remaining regions: " + graph.regionNodes.size());
        System.out.println("- Remaining chokepoints: " + graph.chokepointNodes.size());

        // Verify connectivity
        System.out.println("\nVerifying graph connectivity...");
        HashSet<Node> visited = new HashSet<>();
        Queue<Node> toVisit = new LinkedList<>();

        if (!graph.nodes.isEmpty()) {
            toVisit.add(graph.nodes.get(0));

            while (!toVisit.isEmpty()) {
                Node current = toVisit.poll();
                if (!visited.contains(current)) {
                    visited.add(current);
                    toVisit.addAll(current.neighbours);
                }
            }

            if (visited.size() == graph.nodes.size()) {
                System.out.println("✓ Graph is fully connected (" + visited.size() + " nodes reachable)");
            } else {
                System.out.println("✗ Graph is DISCONNECTED! Only " + visited.size() + " of " +
                                 graph.nodes.size() + " nodes are reachable");
                System.out.println("Disconnected nodes:");
                for (Node n : graph.nodes) {
                    if (!visited.contains(n)) {
                        String type = graph.regionNodes.contains(n) ? "Region" : "Chokepoint";
                        System.out.println("  " + type + " @" + n.position);
                    }
                }
            }
        }
    }

    private void mergeContiguousNodes(RegionGraph graph) {
        System.out.println("Merging contiguous nodes of the same type...");

        // First, list all initial adjacent pairs
        System.out.println("\nInitial adjacent regions before merging:");
        for (Node region : graph.regionNodes) {
            for (Node neighbor : region.neighbours) {
                if (graph.regionNodes.contains(neighbor)) {
                    // Only print each pair once (when first node < second node by position)
                    if (region.position.x < neighbor.position.x ||
                        (region.position.x == neighbor.position.x && region.position.y < neighbor.position.y)) {
                        System.out.println("  Adjacent regions: @" + region.position +
                                         " (r=" + String.format("%.1f", region.radius) + ") <-> @" +
                                         neighbor.position + " (r=" + String.format("%.1f", neighbor.radius) + ")");
                    }
                }
            }
        }

        int regionsMerged = 0;
        int chokepointsMerged = 0;
        HashSet<Node> nodesToRemove = new HashSet<>();

        // Keep merging until no more merges are possible
        boolean changed = true;
        int iterations = 0;
        while (changed) {
            changed = false;
            iterations++;
            System.out.println("Merge iteration " + iterations + "...");

            // Check each region for contiguous regions to merge
            for (Node region : graph.regionNodes) {
                if (nodesToRemove.contains(region)) continue;

                for (Node neighbor : new HashSet<>(region.neighbours)) {
                    if (graph.regionNodes.contains(neighbor) && !nodesToRemove.contains(neighbor)) {
                        // Found two adjacent regions - merge them
                        Node nodeToKeep, nodeToRemove;

                        if (region.radius >= neighbor.radius) {
                            nodeToKeep = region;
                            nodeToRemove = neighbor;
                        } else {
                            nodeToKeep = neighbor;
                            nodeToRemove = region;
                        }

                        System.out.println("Merging regions: keeping @" + nodeToKeep.position +
                                         " (radius=" + String.format("%.1f", nodeToKeep.radius) +
                                         "), removing @" + nodeToRemove.position +
                                         " (radius=" + String.format("%.1f", nodeToRemove.radius) + ")");

                        // Transfer all connections from nodeToRemove to nodeToKeep
                        for (Node n : nodeToRemove.neighbours) {
                            if (n != nodeToKeep) {
                                n.neighbours.remove(nodeToRemove);
                                n.neighbours.add(nodeToKeep);
                                nodeToKeep.neighbours.add(n);
                            }
                        }

                        // Remove the connection between the two nodes being merged
                        nodeToKeep.neighbours.remove(nodeToRemove);

                        // Mark for removal
                        nodesToRemove.add(nodeToRemove);
                        regionsMerged++;
                        changed = true;
                        break; // Start over since we modified the graph
                    }
                }
                if (changed) break;
            }

            // Check each chokepoint for contiguous chokepoints to merge
            if (!changed) {
                for (Node chokepoint : graph.chokepointNodes) {
                    if (nodesToRemove.contains(chokepoint)) continue;

                    for (Node neighbor : new HashSet<>(chokepoint.neighbours)) {
                        if (graph.chokepointNodes.contains(neighbor) && !nodesToRemove.contains(neighbor)) {
                            // Found two adjacent chokepoints - merge them
                            Node nodeToKeep, nodeToRemove;

                            if (chokepoint.radius >= neighbor.radius) {
                                nodeToKeep = chokepoint;
                                nodeToRemove = neighbor;
                            } else {
                                nodeToKeep = neighbor;
                                nodeToRemove = chokepoint;
                            }

                            System.out.println("Merging chokepoints: keeping @" + nodeToKeep.position +
                                             " (radius=" + String.format("%.1f", nodeToKeep.radius) +
                                             "), removing @" + nodeToRemove.position +
                                             " (radius=" + String.format("%.1f", nodeToRemove.radius) + ")");

                            // Transfer all connections from nodeToRemove to nodeToKeep
                            for (Node n : nodeToRemove.neighbours) {
                                if (n != nodeToKeep) {
                                    n.neighbours.remove(nodeToRemove);
                                    n.neighbours.add(nodeToKeep);
                                    nodeToKeep.neighbours.add(n);
                                }
                            }

                            // Remove the connection between the two nodes being merged
                            nodeToKeep.neighbours.remove(nodeToRemove);

                            // Mark for removal
                            nodesToRemove.add(nodeToRemove);
                            chokepointsMerged++;
                            changed = true;
                            break; // Start over since we modified the graph
                        }
                    }
                    if (changed) break;
                }
            }
        }

        // Remove all merged nodes from the graph
        for (Node node : nodesToRemove) {
            graph.nodes.remove(node);
            graph.regionNodes.remove(node);
            graph.chokepointNodes.remove(node);
        }

        System.out.println("Contiguous node merging complete:");
        System.out.println("- Regions merged: " + regionsMerged);
        System.out.println("- Chokepoints merged: " + chokepointsMerged);
        System.out.println("- Final graph size: " + graph.nodes.size() + " nodes");
        System.out.println("- Final regions: " + graph.regionNodes.size());
        System.out.println("- Final chokepoints: " + graph.chokepointNodes.size());

        // Debug: Check if any regions are still adjacent
        System.out.println("\nChecking for remaining adjacent nodes of same type:");
        int adjacentRegions = 0;
        int adjacentChokepoints = 0;

        for (Node node : graph.regionNodes) {
            for (Node neighbor : node.neighbours) {
                if (graph.regionNodes.contains(neighbor)) {
                    System.out.println("WARNING: Adjacent regions still exist: @" + node.position +
                                     " (r=" + String.format("%.1f", node.radius) + ") <-> @" +
                                     neighbor.position + " (r=" + String.format("%.1f", neighbor.radius) + ")");
                    adjacentRegions++;
                }
            }
        }

        for (Node node : graph.chokepointNodes) {
            for (Node neighbor : node.neighbours) {
                if (graph.chokepointNodes.contains(neighbor)) {
                    System.out.println("WARNING: Adjacent chokepoints still exist: @" + node.position +
                                     " (r=" + String.format("%.1f", node.radius) + ") <-> @" +
                                     neighbor.position + " (r=" + String.format("%.1f", neighbor.radius) + ")");
                    adjacentChokepoints++;
                }
            }
        }

        if (adjacentRegions > 0) {
            System.out.println("Found " + (adjacentRegions/2) + " pairs of adjacent regions that weren't merged");
        }
        if (adjacentChokepoints > 0) {
            System.out.println("Found " + (adjacentChokepoints/2) + " pairs of adjacent chokepoints that weren't merged");
        }

        // List all regions with their connections for debugging
        System.out.println("\nFinal region connections:");
        int rIdx = 0;
        for (Node region : graph.regionNodes) {
            System.out.print("R" + rIdx + " @" + region.position + " connected to: ");
            for (Node neighbor : region.neighbours) {
                String type = graph.regionNodes.contains(neighbor) ? "R" : "C";
                System.out.print(type + "@" + neighbor.position + " ");
            }
            System.out.println();
            rIdx++;
        }
    }

    private RegionGraph convertGraph(HalfEdgeDiagram g){
        HashMap<Vertex,Node> nodes = new HashMap<>();
        for (Edge e : g.edges) {
            if (e.valid) {
                if(!pruned.contains(e)){ // replace with inline check
                    Node n1 = vertexToNode(nodes, e.source);
                    Node n2 = vertexToNode(nodes, e.target);
                    n1.addNeighbour(n2);
                    n2.addNeighbour(n1);
                }
            }
        }
        System.out.println("Converted voronoid to regiongraph with "+nodes.size()+" nodes");
        return new RegionGraph(nodes.values());
   }

    private Node vertexToNode(HashMap<Vertex,Node>nodes, Vertex v){
        if(nodes.containsKey(v)){
            return nodes.get(v);
        }
        int[] v1 = this.coordsFromVoronoi(v);
        Node n = new Node(v1[0],v1[1]);
        nodes.put(v,n);
        return n;
    }

    private ArrayList<Edge> getPrunedEdges(VoronoiDiagram voronoid) {
        HalfEdgeDiagram hed = voronoid.get_graph_reference();
        ArrayList<Vertex> vertices = new ArrayList<>();
        for(Vertex v:hed.vertices){
            vertices.add(v);
        }
        ArrayList<Edge> pruned = new ArrayList<>();

        for(Vertex v: vertices){
            int[] c = coordsFromVoronoi(v);
            // remove vertex if inside obstacle or on the border
            c[0] = snap(snap(c[0],0,5),width,5);
            c[1] = snap(snap(c[1],0,5),width,5);

            if(c[0] < 0 || c[0] >= width || c[1] < 1 || c[1] >= height) {
                pruned.addAll(v.out_edges);
                pruned.addAll(v.in_edges);
            }
            else if(labelMap[c[0]][c[1]] != 0){
                pruned.addAll(v.out_edges);
                pruned.addAll(v.in_edges);
            }
        }
        System.out.println(pruned.size()+" voronoi edges pruned");
        return pruned;
    }

    public VoronoiDiagram getVoronoid(){
        return voronoid;
    }

    public ArrayList<Contour> getContours(){
        return fullContours;
    }

    public ArrayList<Contour> getSimplifiedContours(){
        return simplifiedContours;
    }

    int clamp(int value, int min, int max) {
        return value;
        //return Math.min(Math.max(value, min), max);
    }

    int snap(int value, int attractor, int tolerance){
        return Math.abs(value-attractor)<tolerance?attractor:value;
    }

    double[] coordsToVoronoi(int x, int y){
        double vx = ((double)x - v_width/2)/radius;
        double vy = ((double)y - v_height/2)/radius;
        return new double[]{vx, vy};
    }

    int[] coordsFromVoronoi(Vertex v){
        return coordsFromVoronoi(v.position);
    }

    int[] coordsFromVoronoi(org.rogach.jopenvoronoi.Point p){
        return coordsFromVoronoi(p.x,p.y);
    }

    int[] coordsFromVoronoi(double x, double y){
        int ix = (int)Math.round(x*radius + v_width/2);
        int iy = (int)Math.round(y*radius + v_height/2);
        return new int[]{ix,iy};
    }

    private VoronoiDiagram generateVoronoi(ArrayList<Contour>obstacles){
        long timeStarted = System.currentTimeMillis();
        VoronoiDiagram vd = new VoronoiDiagram();

        Point[] boundingBox = new Point[4];
        boundingBox[0] = new Point(1,1);
        boundingBox[1] = new Point(width-1,1);
        boundingBox[2] = new Point(width-1,height-1);
        boundingBox[3] = new Point(1,height-1);

        Vertex[] cornerVertices = new Vertex[4];

        ArrayList<Vertex> allVertices = new ArrayList<>();
        Vertex[][] contourVertices = new Vertex[obstacles.size()][];
        Iterator<Contour> i = obstacles.iterator();

        int v = 0;
        // add vertices for detected contours
        while (i.hasNext()) {
            Point[] pts = i.next().getPointsArray();
            Vertex[] vertices = new Vertex[pts.length-1];

            // jopenvoronoi requires all points to be within a unit circle centered at zero
            // cast and add all vertices; skip last vertex of each contour
            for(int j=0;j < pts.length-1;j++){
                // snap to border because of voodoo:
                // first, bwta2 does this for some reason
                // second, skipping this causes NPE in voronoi
                int px = snap(snap(pts[j].x, width,5),1,5);
                int py = snap(snap(pts[j].y, height,5),1,5);
                double[] vc = coordsToVoronoi(px,py);

                vertices[j] = vd.insert_point_site(new org.rogach.jopenvoronoi.Point(vc[0],vc[1]));
                allVertices.add(vertices[j]);
            }
            contourVertices[v] = vertices;
            v++;
        }

        // add vertices for corners
        for(int z = 0; z < boundingBox.length;z++){
            Point p = boundingBox[z];
            double[] vc = coordsToVoronoi(p.x,p.y);
            Vertex cv = vd.insert_point_site(new org.rogach.jopenvoronoi.Point(vc[0],vc[1]));
            cornerVertices[z] = cv;
        }

        // define borders
        borders = new Vertex[4][];
        double tolerance = 0.005;
        borders[0] = getHorizontalBorder(allVertices,cornerVertices[0],cornerVertices[1],tolerance); // north
        borders[1] = getHorizontalBorder(allVertices,cornerVertices[2],cornerVertices[3],tolerance); // south
        borders[2] = getVerticalBorder(allVertices,cornerVertices[1],cornerVertices[2],tolerance); // east
        borders[3] = getVerticalBorder(allVertices,cornerVertices[0],cornerVertices[3],tolerance); // west

        // assign segments
        for(int vi=0;vi < contourVertices.length;vi++) {
            for (int j = 0; j < contourVertices[vi].length; j++) {
                int k = j + 1 < contourVertices[vi].length ? j + 1 : 0;
                // check if these two vertices were on a border, and thus already linked
                boolean novel = true;
                isNovel:
                for(int l=0;l<borders.length;l++) {
                    for (int n = 1; n < borders[l].length; n++) {
                        if(borders[l][n-1] == contourVertices[vi][j] && borders[l][n] == contourVertices[vi][k]){
                            novel = false;
                            break isNovel;
                        }
                        if(borders[l][n] == contourVertices[vi][j] && borders[l][n-1] == contourVertices[vi][k]){
                            novel = false;
                            break isNovel;
                        }
                    }
                }

                if(novel) {
                    Vertex v1 = contourVertices[vi][j];
                    Vertex v2 = contourVertices[vi][k];
                    try {
                        vd.insert_line_site(v1, v2);
                    } catch (Exception e) {
                        this.errorEdge = new Vertex[2];
                        errorEdge[0] = contourVertices[vi][j];
                        errorEdge[1] = contourVertices[vi][k];
                        return vd;
                    }
                }
            }
        }

        // assign borders
        for(int l=0;l<borders.length;l++) {
            for (int n = 1; n < borders[l].length; n++) {
                Vertex v1 = borders[l][n - 1];
                Vertex v2 = borders[l][n];
                try {
                    vd.insert_line_site(v1, v2);
                } catch (Exception e) {
                    e.printStackTrace();
                    this.errorEdge = new Vertex[2];
                    errorEdge[0] = borders[l][n - 1];
                    errorEdge[1] = borders[l][n];
                    return vd;
                }
                // -1 will meant to us that the obstacle is edge of map
                labelTree = labelTree.add(-1, Geometries.line(v1.position.x,v1.position.y,v2.position.x,v2.position.y));
            }
        }

        System.out.println("Generated voronoi in "+(System.currentTimeMillis() - timeStarted) + "ms");

        return vd;
    }

    class SortByX implements Comparator<Vertex>
    {
        public int compare(Vertex a, Vertex b)
        {
            return (int)Math.signum(a.position.x - b.position.x);
        }
    }

    class SortByY implements Comparator<Vertex>
    {
        public int compare(Vertex a, Vertex b)
        {
            return (int)Math.signum(a.position.y - b.position.y);
        }
    }

    // link vertices within a horizontal line with edges
    private Vertex[] getHorizontalBorder(ArrayList<Vertex> vertices,Vertex left, Vertex right, double tolerance){
        ArrayList<Vertex> inTheWay = new ArrayList<>();

        double height = (left.position.y+right.position.y)/2;

        inTheWay.add(left);
        inTheWay.add(right);

        for(Vertex v:vertices){
            if(Math.abs(v.position.y-height) < tolerance){
                inTheWay.add(v);
            }
        }
        inTheWay.sort(new SortByX());
        return inTheWay.toArray(new Vertex[inTheWay.size()]);
    }

    // link vertices within a vertical line with edges
    private Vertex[] getVerticalBorder(ArrayList<Vertex> vertices, Vertex top, Vertex bottom, double tolerance){
        ArrayList<Vertex> inTheWay = new ArrayList<>();
        double width = (top.position.x+bottom.position.x)/2;
        inTheWay.add(top);
        inTheWay.add(bottom);
        for(Vertex v:vertices){
            if(Math.abs(v.position.x-width) < tolerance){
                inTheWay.add(v);
            }
        }
        inTheWay.sort(new SortByY());
        return inTheWay.toArray(new Vertex[inTheWay.size()]);
    }

    private ArrayList<Contour> simplifyContours(){
        long timeStarted = System.currentTimeMillis();
        ArrayList<Contour> simple = new ArrayList<Contour>();
        Iterator<Contour> i = fullContours.iterator();
        int filteredByPoints = 0, filteredByArea = 0;
        while (i.hasNext()) {
            Contour simplifiedContour = simplifyContour(i.next());
            if(simplifiedContour.length() > 3){
                double area = simplifiedContour.getArea();
                if(area > 100.0) { // Filter out contours smaller than 100 square pixels
                    simple.add(simplifiedContour);
                } else {
                    filteredByArea++;
                    System.out.println("DEBUG: Filtered tiny contour with area " + String.format("%.1f", area) + " and " + simplifiedContour.length() + " points");
                }
            } else {
                filteredByPoints++;
            }
        }
        System.out.println("DEBUG: Filtered " + filteredByPoints + " contours by point count, " + filteredByArea + " by area");
        System.out.println("Total time spent simplifying: "+(System.currentTimeMillis() - timeStarted) + "ms");
        return simple;
    }

    private Contour simplifyContour(Contour c) {
        Simplify<Point> simplify = new Simplify<Point>(new Point[0]);
        Point[] allPoints = c.getPointsArray();
        Point[] lessPoints = simplify.simplify(allPoints, 8, true);

        return new Contour(lessPoints);

    }

    // fill a non-simplified hole polygon with appropriate obstacle label
    private void fillHole(Contour hole, int labelID){
        int[] bounds = hole.getBounds();

        ArrayList<Point> points = hole.getPoints();

        for(int y = bounds[2];y<bounds[3];y++) {
            ArrayList<Integer> scanLine = new ArrayList<>();
            for(Point p: points){
                if(p.y == y){
                    scanLine.add(p.x);
                }
            }
            Integer[] intersects = new Integer[scanLine.size()];
            scanLine.toArray(intersects);
            Arrays.sort(intersects);

            if(intersects.length > 1) {
                boolean inside = false;
                for (int n = 0; n < intersects.length-1; n += 1) {
                    inside = !inside;

                    int min = intersects[n];
                    int max = intersects[n+1];

                    if (max <= min+1){
                        inside = false;
                    }

                    if(inside){
                        for(int x = min;x<max;x++){
                            labelMap[x][y] = labelID;
                        }
                    }
                }
            }
        }
    }

    private ArrayList<Contour> traceContours()
    {
        long timeStarted = System.currentTimeMillis();
        int cy, cx, tracingDirection, connectedComponentsCount = 0, labelId = 0;
        int width = bitMap.getWidth();
        int height = bitMap.getHeight();

        ArrayList<Contour> contours = new ArrayList<Contour>();

        for (cx = 0; cx < width; ++cx) {
            for (cy = 0, labelId = 0; cy < height; ++cy) {
                Color c = new Color(bitMap.getRGB(cx, cy));

                if (c.getRed() > 0) { // is obstacle
                    if (labelId != 0) { // use pre-pixel label
                        labelMap[cx][cy] = labelId;
                    } else {
                        labelId = labelMap[cx][cy];

                        if (labelId == 0) {
                            labelId = ++connectedComponentsCount;
                            tracingDirection = 0;
                            // external contour
                            Contour obstacle  = traceContour(cx, cy, labelId, tracingDirection);
                            contours.add(obstacle);
                            labelMap[cx][cy] = labelId;
                        }
                    }
                } else if (labelId != 0) { // walkable & pre-pixel has been labeled
                    if (labelMap[cx][cy] == 0) {
                        tracingDirection = 1;
                        // TODO: retain holes if they are interesting (large) enough
                        LabeledContour hole = new LabeledContour(traceContour(cx, cy-1, labelId, tracingDirection),labelId);
                        fillHole(hole, labelId);
                        holes.add(hole);

                        /*
                        BoostPolygon polygon;
                        boost::geometry::assign_points(polygon, hole);
                        // if polygon isn't too small, add it to the result
                        if (boost::geometry::area(polygon) > MIN_ARE_POLYGON) {
                            LOG(" - [WARNING] Found big walkable HOLE");
                        } else {
                            // "remove" the hole filling it with the polygon label
                            holesToLabel.emplace_back(hole, labelId);
                        }*/

                    }
                    labelId = 0;
                }
            }
        }

        System.out.println("Total time spent tracing: "+(System.currentTimeMillis() - timeStarted) + "ms");
        return contours;
    }

    Contour traceContour(int cx, int cy, int labelId, int tracingDirection)
    {
        boolean tracingStopFlag = false, keepSearching = true;
        int fx, fy, sx = cx, sy = cy;
        Contour contourPoints = new Contour();
        TracePoint p = tracer(cx, cy, tracingDirection);

        contourPoints.addPoint(p);


        if (p.x != sx || p.y != sy) {
            fx = p.x; // final/initial points
            fy = p.y;

            while (keepSearching) {
                labelMap[p.x][p.y] = labelId;
                p = tracer(p.x, p.y, (p.traceDirection + 6) % 8);

                contourPoints.addPoint(p);

                if (p.x == sx && p.y == sy) {
                    tracingStopFlag = true;
                } else if (tracingStopFlag) {
                    if (p.x == fx && p.y == fy) {
                        keepSearching = false;
                    } else {
                        tracingStopFlag = false;
                    }
                }
            }
        }

        // Explicitly add the final closing segment if the contour has points
        // The tracing algorithm implicitly closes but doesn't store the final segment
        if (contourPoints.length() > 0) {
            Point firstPoint = contourPoints.getPoint(0);
            Point lastPoint = contourPoints.getPoint(contourPoints.length() - 1);

            // Only add if the contour isn't already explicitly closed
            if (!firstPoint.equals(lastPoint)) {
                contourPoints.addPoint(firstPoint);
            }
        }

        return contourPoints;
    }

    static int[][] searchDirection = { { 1, 0 }, { 1, 1 }, { 0 , 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 } };

    private TracePoint tracer(TracePoint tp){
        return tracer(tp.x,tp.y,(tp.traceDirection + 6) % 8);
    }

    private TracePoint tracer(int cx, int cy, int tracingdirection)
    {
        int i, y, x;
        int width = bitMap.getWidth();
        int height = bitMap.getHeight();

        for (i = 0; i < 7; ++i) {

            y = cy + searchDirection[tracingdirection][0];
            x = cx + searchDirection[tracingdirection][1];

            // filter invalid position (out of range)
            if (x < 0 || x >= width || y < 0 || y >= height) {
                tracingdirection = (tracingdirection + 1) % 8;
                continue;
            }
            Color c = new Color(bitMap.getRGB(x,y));

            if (c.getRed() > 0) {
                return new TracePoint(x,y,tracingdirection);
            } else {
                // not an obstacle, label as such
                labelMap[x][y] = -1;
                tracingdirection = (tracingdirection + 1) % 8;
            }
        }
        return new TracePoint(cx,cy,tracingdirection);
    }
}
