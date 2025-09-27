import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import javax.imageio.ImageIO;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.JsonNode;

public class RegionGraphPlotter {

    static class Point {
        int x, y;
        Point(int x, int y) { this.x = x; this.y = y; }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            Point point = (Point) obj;
            return x == point.x && y == point.y;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y);
        }

        public String toString() { return "(" + x + "," + y + ")"; }
    }

    static class Segment {
        Point start, end;
        String source;
        Color color;

        Segment(Point start, Point end, String source) {
            this.start = start;
            this.end = end;
            this.source = source;
            this.color = getSourceColor(source);
        }

        static Color getSourceColor(String source) {
            if (source.startsWith("obstacle_1000")) return Color.RED;
            if (source.startsWith("obstacle_1004")) return Color.GREEN;
            if (source.startsWith("obstacle_1007")) return Color.BLUE;
            if (source.startsWith("obstacle_1009")) return Color.ORANGE;
            if (source.startsWith("chokepoint")) return Color.MAGENTA;
            if (source.startsWith("edge_contact")) return Color.CYAN;
            return Color.GRAY;
        }
    }

    static class PlotGraph {
        Set<Point> nodes = new HashSet<>();
        ArrayList<Segment> segments = new ArrayList<>();
        Map<Point, Integer> nodeDegrees = new HashMap<>();
        Map<Point, ArrayList<Point>> adjacency = new HashMap<>();

        void addSegment(Segment segment) {
            segments.add(segment);
            nodes.add(segment.start);
            nodes.add(segment.end);

            nodeDegrees.put(segment.start, nodeDegrees.getOrDefault(segment.start, 0) + 1);
            nodeDegrees.put(segment.end, nodeDegrees.getOrDefault(segment.end, 0) + 1);

            // Build adjacency list
            adjacency.computeIfAbsent(segment.start, k -> new ArrayList<>()).add(segment.end);
            adjacency.computeIfAbsent(segment.end, k -> new ArrayList<>()).add(segment.start);
        }

        Segment findSegment(Point start, Point end) {
            for (Segment seg : segments) {
                if ((seg.start.equals(start) && seg.end.equals(end)) ||
                    (seg.start.equals(end) && seg.end.equals(start))) {
                    return seg;
                }
            }
            return null;
        }

        double getSegmentLength(Point start, Point end) {
            return Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2));
        }
    }

    // Dijkstra result for pathfinding
    static class DijkstraResult {
        ArrayList<Point> path = new ArrayList<>();
        double pathLength = 0.0;
        boolean pathFound = false;
        Point startPoint, endPoint;
        Segment prohibitedSegment;

        DijkstraResult(Point start, Point end, Segment prohibited) {
            this.startPoint = start;
            this.endPoint = end;
            this.prohibitedSegment = prohibited;
        }
    }

    static DijkstraResult runDijkstra(PlotGraph graph, Point start, Point end, Segment prohibitedSegment) {
        DijkstraResult result = new DijkstraResult(start, end, prohibitedSegment);

        Map<Point, Double> distances = new HashMap<>();
        Map<Point, Point> previous = new HashMap<>();
        Set<Point> visited = new HashSet<>();

        // Initialize distances
        for (Point node : graph.nodes) {
            distances.put(node, Double.POSITIVE_INFINITY);
        }
        distances.put(start, 0.0);

        while (true) {
            // Find unvisited node with minimum distance
            Point current = null;
            double minDistance = Double.POSITIVE_INFINITY;
            for (Point node : graph.nodes) {
                if (!visited.contains(node) && distances.get(node) < minDistance) {
                    minDistance = distances.get(node);
                    current = node;
                }
            }

            if (current == null || minDistance == Double.POSITIVE_INFINITY) {
                break; // No more reachable nodes
            }

            visited.add(current);

            if (current.equals(end)) {
                // Reconstruct path
                ArrayList<Point> path = new ArrayList<>();
                Point step = end;
                while (step != null) {
                    path.add(0, step);
                    step = previous.get(step);
                }
                result.path = path;
                result.pathLength = distances.get(end);
                result.pathFound = true;
                break;
            }

            // Check neighbors
            ArrayList<Point> neighbors = graph.adjacency.get(current);
            if (neighbors != null) {
                for (Point neighbor : neighbors) {
                    if (visited.contains(neighbor)) continue;

                    // Check if this edge is prohibited
                    Segment edge = graph.findSegment(current, neighbor);
                    if (prohibitedSegment != null && edge != null &&
                        ((edge.start.equals(prohibitedSegment.start) && edge.end.equals(prohibitedSegment.end)) ||
                         (edge.start.equals(prohibitedSegment.end) && edge.end.equals(prohibitedSegment.start)))) {
                        continue; // Skip prohibited segment
                    }

                    double edgeLength = graph.getSegmentLength(current, neighbor);
                    double newDistance = distances.get(current) + edgeLength;

                    if (newDistance < distances.get(neighbor)) {
                        distances.put(neighbor, newDistance);
                        previous.put(neighbor, current);
                    }
                }
            }
        }

        return result;
    }

    static DijkstraResult performPathfindingDemo(PlotGraph graph, int regionId, String dataFile) {
        try {
            // Get chokepoint IDs for this region
            ArrayList<Integer> regionChokepointIds = extractRegionChokepointIds(dataFile, regionId);

            if (regionChokepointIds.isEmpty()) {
                System.out.println("Region " + regionId + " has no chokepoints, skipping pathfinding demo");
                return null;
            }

            // Extract crossing points from the FIRST chokepoint only
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(new File(dataFile));

            Point start = null, end = null;
            int testChokepointId = regionChokepointIds.get(0);
            Segment prohibitedSegment = null;

            JsonNode chokepoints = root.get("chokepoints");
            if (chokepoints != null) {
                for (JsonNode cp : chokepoints) {
                    int chokepointId = cp.get("id").asInt();
                    if (chokepointId == testChokepointId) {
                        JsonNode crossingPoints = cp.get("crossingPoints");
                        if (crossingPoints != null && crossingPoints.size() == 2) {
                            start = new Point(crossingPoints.get(0).get("x").asInt(),
                                            crossingPoints.get(0).get("y").asInt());
                            end = new Point(crossingPoints.get(1).get("x").asInt(),
                                          crossingPoints.get(1).get("y").asInt());

                            // The prohibited segment is the chokepoint segment connecting these two points
                            prohibitedSegment = new Segment(start, end, "chokepoint_" + testChokepointId);
                        }
                        break;
                    }
                }
            }

            if (start == null || end == null) {
                System.out.println("Could not extract crossing points from chokepoint " + testChokepointId);
                return null;
            }

            System.out.println("Running Dijkstra pathfinding from " + start + " to " + end);
            System.out.println("Testing chokepoint " + testChokepointId + " - can we route around it via obstacles?");
            System.out.println("Prohibiting segment: " + prohibitedSegment.start + " -> " + prohibitedSegment.end + " (" + prohibitedSegment.source + ")");

            DijkstraResult result = runDijkstra(graph, start, end, prohibitedSegment);

            if (result.pathFound) {
                System.out.println("Alternative path found! Length: " + String.format("%.2f", result.pathLength) + ", nodes: " + result.path.size());
                System.out.println("This indicates obstacle contours provide connectivity around blocked chokepoint");
            } else {
                System.out.println("No alternative path found - confirms incomplete boundary connectivity in region " + regionId);
                System.out.println("Obstacle contours cannot provide alternative routing around chokepoint " + testChokepointId);
            }

            return result;

        } catch (Exception e) {
            System.err.println("Error in pathfinding demo: " + e.getMessage());
            return null;
        }
    }

    public static void main(String[] args) {
        try {
            // Parse command line arguments
            if (args.length < 1) {
                System.out.println("Usage: RegionGraphPlotter <regionId> [dataFile]");
                System.out.println("  regionId: The region ID to analyze (e.g., 15)");
                System.out.println("  dataFile: JSON data file (default: result.json)");
                System.out.println("\nExample: RegionGraphPlotter 15");
                System.out.println("         RegionGraphPlotter 2 my_data.json");
                return;
            }

            int regionId = Integer.parseInt(args[0]);
            String dataFile = args.length > 1 ? args[1] : "result.json";

            System.out.println("=== Region " + regionId + " Graph Analysis ===");

            // Extract region's obstacle IDs from JSON
            Set<Integer> regionObstacleIds = extractRegionObstacleIds(dataFile, regionId);
            if (regionObstacleIds.isEmpty()) {
                System.err.println("Error: Region " + regionId + " not found or has no connected obstacles");
                return;
            }

            System.out.println("Region " + regionId + " obstacle IDs: " + regionObstacleIds);

            // Extract the graph from JSON
            PlotGraph graph = buildRegionGraph(dataFile, regionId, regionObstacleIds);
            System.out.println("Loaded graph with " + graph.nodes.size() + " nodes and " + graph.segments.size() + " segments");

            // Run pathfinding demonstration
            DijkstraResult pathResult = performPathfindingDemo(graph, regionId, dataFile);

            // Create the plot
            BufferedImage plot = createGraphPlot(graph, regionId, pathResult);

            // Save the plot
            String outputFile = "region_" + regionId + "_graph_plot.png";
            ImageIO.write(plot, "PNG", new File(outputFile));
            System.out.println("Graph plot saved to " + outputFile);

            // Print analysis
            printRegionGraphAnalysis(graph, regionId);

        } catch (NumberFormatException e) {
            System.err.println("Error: Region ID must be a number");
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }

    static Set<Integer> extractRegionObstacleIds(String filename, int regionId) throws IOException {
        Set<Integer> obstacleIds = new HashSet<>();
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(filename));

        // Find the specified region
        JsonNode regions = root.get("regions");
        if (regions != null) {
            for (JsonNode region : regions) {
                if (region.get("id").asInt() == regionId) {
                    JsonNode chokepointNodes = region.get("chokepointNodes");

                    // Get obstacle IDs from the connected chokepoints
                    JsonNode chokepoints = root.get("chokepoints");
                    if (chokepoints != null && chokepointNodes != null) {
                        for (JsonNode chokepointId : chokepointNodes) {
                            int id = chokepointId.asInt();
                            for (JsonNode cp : chokepoints) {
                                if (cp.get("id").asInt() == id) {
                                    JsonNode obstacleArray = cp.get("obstacleIds");
                                    if (obstacleArray != null) {
                                        for (JsonNode obstacleId : obstacleArray) {
                                            obstacleIds.add(obstacleId.asInt());
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                }
            }
        }

        return obstacleIds;
    }

    static ArrayList<Integer> extractRegionChokepointIds(String filename, int regionId) throws IOException {
        ArrayList<Integer> chokepointIds = new ArrayList<>();
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(filename));

        // Find the specified region
        JsonNode regions = root.get("regions");
        if (regions != null) {
            for (JsonNode region : regions) {
                if (region.get("id").asInt() == regionId) {
                    JsonNode chokepointNodes = region.get("chokepointNodes");
                    if (chokepointNodes != null) {
                        for (JsonNode chokepointId : chokepointNodes) {
                            chokepointIds.add(chokepointId.asInt());
                        }
                    }
                    break;
                }
            }
        }

        return chokepointIds;
    }

    static PlotGraph buildRegionGraph(String filename, int regionId, Set<Integer> regionObstacleIds) throws IOException {
        PlotGraph graph = new PlotGraph();
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(filename));

        // Get region's chokepoint IDs
        ArrayList<Integer> regionChokepointIds = extractRegionChokepointIds(filename, regionId);

        // 1. Extract contour segments from region's obstacles
        JsonNode obstacleContours = root.get("obstacleContours");
        if (obstacleContours != null) {
            for (JsonNode contour : obstacleContours) {
                int obstacleId = contour.get("obstacleId").asInt();
                if (regionObstacleIds.contains(obstacleId)) {
                    JsonNode points = contour.get("points");
                    if (points != null && points.size() > 1) {
                        for (int i = 0; i < points.size() - 1; i++) {
                            Point p1 = new Point(points.get(i).get("x").asInt(), points.get(i).get("y").asInt());
                            Point p2 = new Point(points.get(i + 1).get("x").asInt(), points.get(i + 1).get("y").asInt());
                            graph.addSegment(new Segment(p1, p2, "obstacle_" + obstacleId));
                        }
                    }
                }
            }
        }

        // 2. Extract chokepoint crossing segments for this region
        JsonNode chokepoints = root.get("chokepoints");
        if (chokepoints != null) {
            for (JsonNode cp : chokepoints) {
                int chokepointId = cp.get("id").asInt();
                if (regionChokepointIds.contains(chokepointId)) {
                    JsonNode crossingPoints = cp.get("crossingPoints");
                    if (crossingPoints != null && crossingPoints.size() == 2) {
                        Point p1 = new Point(crossingPoints.get(0).get("x").asInt(),
                                           crossingPoints.get(0).get("y").asInt());
                        Point p2 = new Point(crossingPoints.get(1).get("x").asInt(),
                                           crossingPoints.get(1).get("y").asInt());
                        graph.addSegment(new Segment(p1, p2, "chokepoint_" + chokepointId));
                    }
                }
            }
        }

        // 3. Extract edge contact segments for this region's obstacles
        JsonNode contourEdgeContacts = root.get("contourEdgeContacts");
        if (contourEdgeContacts != null) {
            for (JsonNode contact : contourEdgeContacts) {
                int obstacleId = contact.get("obstacleId").asInt();
                if (regionObstacleIds.contains(obstacleId)) {
                    JsonNode segStart = contact.get("edgeSegmentStart");
                    JsonNode segEnd = contact.get("edgeSegmentEnd");
                    if (segStart != null && segEnd != null) {
                        Point start = new Point(segStart.get("x").asInt(), segStart.get("y").asInt());
                        Point end = new Point(segEnd.get("x").asInt(), segEnd.get("y").asInt());
                        graph.addSegment(new Segment(start, end, "edge_contact_" + obstacleId));
                    }
                }
            }
        }

        return graph;
    }

    static BufferedImage createGraphPlot(PlotGraph graph, int regionId, DijkstraResult pathResult) {
        // Find bounds
        int minX = Integer.MAX_VALUE, maxX = Integer.MIN_VALUE;
        int minY = Integer.MAX_VALUE, maxY = Integer.MIN_VALUE;

        for (Point node : graph.nodes) {
            minX = Math.min(minX, node.x);
            maxX = Math.max(maxX, node.x);
            minY = Math.min(minY, node.y);
            maxY = Math.max(maxY, node.y);
        }

        // Add padding
        int padding = 50;
        int width = (maxX - minX) + 2 * padding;
        int height = (maxY - minY) + 2 * padding;

        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = image.createGraphics();

        // Enable antialiasing
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // White background
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, width, height);

        // Draw segments
        for (Segment segment : graph.segments) {
            g.setColor(segment.color);
            g.setStroke(new BasicStroke(2.0f));

            int x1 = segment.start.x - minX + padding;
            int y1 = segment.start.y - minY + padding;
            int x2 = segment.end.x - minX + padding;
            int y2 = segment.end.y - minY + padding;

            g.drawLine(x1, y1, x2, y2);
        }

        // Draw Dijkstra path if found
        if (pathResult != null && pathResult.pathFound && pathResult.path.size() > 1) {
            g.setColor(Color.YELLOW);
            g.setStroke(new BasicStroke(4.0f)); // Thick yellow line for path

            for (int i = 0; i < pathResult.path.size() - 1; i++) {
                Point p1 = pathResult.path.get(i);
                Point p2 = pathResult.path.get(i + 1);

                int x1 = p1.x - minX + padding;
                int y1 = p1.y - minY + padding;
                int x2 = p2.x - minX + padding;
                int y2 = p2.y - minY + padding;

                g.drawLine(x1, y1, x2, y2);
            }
        }

        // Draw nodes with different sizes based on degree
        for (Point node : graph.nodes) {
            int degree = graph.nodeDegrees.get(node);
            int nodeSize = 3 + degree; // Base size 3, +1 per degree

            int x = node.x - minX + padding;
            int y = node.y - minY + padding;

            // Special highlighting for path start/end points
            if (pathResult != null && pathResult.pathFound) {
                if (node.equals(pathResult.startPoint)) {
                    g.setColor(Color.GREEN);
                    nodeSize += 3; // Larger for start point
                } else if (node.equals(pathResult.endPoint)) {
                    g.setColor(Color.ORANGE);
                    nodeSize += 3; // Larger for end point
                } else {
                    // Node color based on degree
                    if (degree == 1) g.setColor(Color.RED);        // Endpoints
                    else if (degree == 2) g.setColor(Color.BLACK);  // Chain nodes
                    else g.setColor(Color.BLUE);                    // Junction nodes
                }
            } else {
                // Node color based on degree
                if (degree == 1) g.setColor(Color.RED);        // Endpoints
                else if (degree == 2) g.setColor(Color.BLACK);  // Chain nodes
                else g.setColor(Color.BLUE);                    // Junction nodes
            }

            g.fillOval(x - nodeSize/2, y - nodeSize/2, nodeSize, nodeSize);
        }

        // Add legend
        drawLegend(g, width, height);

        // Add title
        g.setColor(Color.BLACK);
        g.setFont(new Font("Arial", Font.BOLD, 16));
        g.drawString("Region " + regionId + " Component Connectivity Graph", 10, 25);

        g.dispose();
        return image;
    }

    static void drawLegend(Graphics2D g, int width, int height) {
        int legendX = width - 200;
        int legendY = 50;
        int lineHeight = 20;

        g.setFont(new Font("Arial", Font.PLAIN, 12));

        // Segment colors
        g.setColor(Color.BLACK);
        g.drawString("Segments:", legendX, legendY);

        Color[] colors = {Color.RED, Color.GREEN, Color.BLUE, Color.ORANGE, Color.MAGENTA, Color.CYAN};
        String[] labels = {"Obstacle 1000", "Obstacle 1004", "Obstacle 1007", "Obstacle 1009", "Chokepoints", "Edge Contacts"};

        for (int i = 0; i < colors.length; i++) {
            g.setColor(colors[i]);
            g.setStroke(new BasicStroke(3.0f));
            g.drawLine(legendX, legendY + (i + 1) * lineHeight, legendX + 20, legendY + (i + 1) * lineHeight);

            g.setColor(Color.BLACK);
            g.drawString(labels[i], legendX + 25, legendY + (i + 1) * lineHeight + 4);
        }

        // Path legend (if applicable)
        legendY += (colors.length + 2) * lineHeight;
        g.setColor(Color.BLACK);
        g.drawString("Pathfinding:", legendX, legendY);

        g.setColor(Color.YELLOW);
        g.setStroke(new BasicStroke(4.0f));
        g.drawLine(legendX, legendY + lineHeight, legendX + 20, legendY + lineHeight);
        g.setColor(Color.BLACK);
        g.drawString("Dijkstra path", legendX + 25, legendY + lineHeight + 4);

        // Node legend
        legendY += 2 * lineHeight;
        g.setColor(Color.BLACK);
        g.drawString("Nodes:", legendX, legendY);

        Color[] nodeColors = {Color.RED, Color.BLACK, Color.BLUE, Color.GREEN, Color.ORANGE};
        String[] nodeLabels = {"Degree 1 (endpoints)", "Degree 2 (chain)", "Degree 3+ (junctions)", "Path start", "Path end"};

        for (int i = 0; i < nodeColors.length; i++) {
            g.setColor(nodeColors[i]);
            g.fillOval(legendX + 5, legendY + (i + 1) * lineHeight - 5, 10, 10);

            g.setColor(Color.BLACK);
            g.drawString(nodeLabels[i], legendX + 25, legendY + (i + 1) * lineHeight + 4);
        }
    }

    static void printRegionGraphAnalysis(PlotGraph graph, int regionId) {
        System.out.println("\n=== Region " + regionId + " Graph Analysis ===");

        // Count segments by type
        Map<String, Integer> segmentCounts = new HashMap<>();
        for (Segment segment : graph.segments) {
            String type = segment.source.split("_")[0];
            segmentCounts.put(type, segmentCounts.getOrDefault(type, 0) + 1);
        }

        System.out.println("Segment counts by type:");
        for (Map.Entry<String, Integer> entry : segmentCounts.entrySet()) {
            System.out.println("  " + entry.getKey() + ": " + entry.getValue());
        }

        // Count nodes by degree
        Map<Integer, Integer> degreeCounts = new HashMap<>();
        for (Point node : graph.nodes) {
            int degree = graph.nodeDegrees.get(node);
            degreeCounts.put(degree, degreeCounts.getOrDefault(degree, 0) + 1);
        }

        System.out.println("\nNode counts by degree:");
        for (Map.Entry<Integer, Integer> entry : degreeCounts.entrySet()) {
            System.out.println("  Degree " + entry.getKey() + ": " + entry.getValue() + " nodes");
        }

        // Connectivity analysis
        System.out.println("\nConnectivity analysis:");
        if (degreeCounts.getOrDefault(1, 0) > 0) {
            System.out.println("  " + degreeCounts.get(1) + " endpoint nodes detected (degree 1)");
        }
        if (degreeCounts.getOrDefault(3, 0) > 0) {
            System.out.println("  " + degreeCounts.get(3) + " junction nodes detected (degree 3+)");
        }
        System.out.println("  Graph may have isolated components");
        System.out.println("  This could explain incomplete boundary extraction for region " + regionId);
    }
}