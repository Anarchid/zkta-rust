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
    double v_width;
    double v_height;
    double radius;
    double diagonal;
    int width;
    int height;
    static int TOO_CLOSE_TO_OBSTACLE = 5;
    ArrayList<Edge> pruned;

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
        regionList = new ArrayList<>();
        chokepointList = new ArrayList<>();
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
        extractRegionPolygons(regions);
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

    public Point[] getChokepointSegments(Node chokepoint){
        return findChokepointCrossingPoints(chokepoint);
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

    private double distanceToLine(Point p, Line l){
        double A = p.x - l.x1(); // position of point rel one end of line
        double B = p.y - l.y1();
        double C = l.x2() - l.x1(); // vector along line
        double D = l.y2() - l.y1();
        double E = -D; // orthogonal vector
        double F = C;

        double dot = A * E + B * F;
        double len_sq = E * E + F * F;

        return (float) Math.abs(dot) / Math.sqrt(len_sq);
    }

    private double getDistanceToObstacle(Node n){
        rx.Observable<Entry<Integer,Line>> closestObstacle = labelTree.nearest((Geometries.point(n.position.x,n.position.y)),radius*2,1);
        Line l = closestObstacle.toBlocking().first().geometry();
        return distanceToLine(n.position, l);
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
                dist_1 = getDistanceToObstacle(n1);
                System.out.println("Parent node distance to obstacle: "+dist_1);
                // Only remove if significantly closer to obstacle than parent AND parent is not too close
                if(dist_0 - 0.9 < dist_1 && dist_1 > TOO_CLOSE_TO_OBSTACLE){
                    System.out.println("Parent is farther from obstacle - removing!");
                    deleted = true;
                    removed++;
                    graph.removeNode(n0);
                } else {
                    deleted = false; // Don't remove this node
                }
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

        // Step 1: Find chokepoint crossing points using R-tree
        HashMap<Node, Point[]> chokepointCrossings = new HashMap<>();

        for (Node chokepoint : graph.chokepointNodes) {
            Point[] crossingPoints = findChokepointCrossingPoints(chokepoint);
            chokepointCrossings.put(chokepoint, crossingPoints);

            if (crossingPoints != null) {
                System.out.println("Chokepoint @" + chokepoint.position +
                                 " (r=" + String.format("%.1f", chokepoint.radius) + ")" +
                                 " crosses obstacles at: " +
                                 crossingPoints[0] + " and " + crossingPoints[1]);
            } else {
                System.out.println("WARNING: No crossing points found for chokepoint @" + chokepoint.position +
                                 " (r=" + String.format("%.1f", chokepoint.radius) + ")");
            }
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

        // Step 4: Generate region boundary polygons
        System.out.println("Generating region boundary polygons...");
        generateRegionPolygons(graph, chokepointCrossings);

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

    private ArrayList<Point> sortBoundaryPoints(Point center, ArrayList<Point> points) {
        // Sort points by angle around the region center to create a proper polygon
        ArrayList<Point> sorted = new ArrayList<>(points);

        sorted.sort((p1, p2) -> {
            double angle1 = Math.atan2(p1.y - center.y, p1.x - center.x);
            double angle2 = Math.atan2(p2.y - center.y, p2.x - center.x);
            return Double.compare(angle1, angle2);
        });

        return sorted;
    }

    private Point[] findChokepointCrossingPoints(Node chokepoint) {
        try {
            // Use R-tree to find the two closest obstacle points to this chokepoint
            // The chokepoint should be positioned at the narrowest point between obstacles

            // Use a generous search radius - at least 50 pixels, or 5x the chokepoint radius, whichever is larger
            double searchRadius = Math.max(50.0, chokepoint.radius * 5);

            rx.Observable<Entry<Integer,Line>> nearestObstacles = labelTree.nearest(
                Geometries.point(chokepoint.position.x, chokepoint.position.y),
                searchRadius,
                20); // Get up to 20 nearest obstacles

            // Find the two closest points on different obstacles
            Point closest1 = null, closest2 = null;
            double minDist1 = Double.MAX_VALUE, minDist2 = Double.MAX_VALUE;
            Line obstacle1 = null, obstacle2 = null;

            int obstacleCount = 0;
            for (Entry<Integer,Line> entry : nearestObstacles.toBlocking().toIterable()) {
                Line obstacle = entry.geometry();
                obstacleCount++;

                // Find closest point on this line segment to the chokepoint
                Point closestPoint = getClosestPointOnLine(chokepoint.position, obstacle);
                double dist = distance(chokepoint.position, closestPoint);

                if (closest1 == null || dist < minDist1) {
                    // Found a closer point - shift previous closest to second closest
                    if (closest1 != null && !obstacle.equals(obstacle1)) {
                        closest2 = closest1;
                        minDist2 = minDist1;
                        obstacle2 = obstacle1;
                    }
                    closest1 = closestPoint;
                    minDist1 = dist;
                    obstacle1 = obstacle;
                } else if (!obstacle.equals(obstacle1) && !closestPoint.equals(closest1) && (closest2 == null || dist < minDist2)) {
                    // Different obstacle than the first one, different point than first, and closer than current second closest
                    closest2 = closestPoint;
                    minDist2 = dist;
                    obstacle2 = obstacle;
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
                return new Point[]{closest1, closest2};
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
        while (i.hasNext()) {
            Contour simplifiedContour = simplifyContour(i.next());
            if(simplifiedContour.length() > 3){
                simple.add(simplifiedContour);
            }
        }
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
