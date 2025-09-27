import java.io.*;
import java.util.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.*;

public class R15GraphAnalysis {

    // Pure geometric point
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
        public String getId() { return x + "_" + y; }
    }

    // Pure geometric segment
    static class Segment {
        Point start, end;
        String source; // For debugging: obstacle ID, chokepoint ID, edge contact, etc.

        Segment(Point start, Point end, String source) {
            this.start = start;
            this.end = end;
            this.source = source;
        }

        public double getLength() {
            return Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2));
        }

        public String toString() {
            return start + " -> " + end + " (" + source + ")";
        }
    }

    // Complete connectivity graph
    static class CompleteGraph {
        Set<Point> nodes = new HashSet<>();
        List<Segment> segments = new ArrayList<>();
        Map<Point, List<Point>> adjacency = new HashMap<>();

        void addSegment(Segment segment) {
            segments.add(segment);
            nodes.add(segment.start);
            nodes.add(segment.end);

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

        // Get node degree (number of connections)
        int getNodeDegree(Point node) {
            List<Point> neighbors = adjacency.get(node);
            return neighbors != null ? neighbors.size() : 0;
        }
    }

    // Dijkstra result
    static class DijkstraResult {
        Map<Point, Double> distances = new HashMap<>();
        Map<Point, Point> previous = new HashMap<>();
        List<Point> path = new ArrayList<>();
        double pathLength = 0.0;
    }

    public static void main(String[] args) {
        try {
            System.out.println("=== R15 Complete Component Analysis ===");

            // Step 1: Extract R15's related component IDs
            System.out.println("\nStep 1: Extracting R15 Component Relationships");
            Set<Integer> r15ObstacleIds = extractR15ObstacleIds("result.json");
            System.out.println("R15 obstacle IDs: " + r15ObstacleIds);

            // Step 2: Build complete connectivity graph from R15 components
            System.out.println("\nStep 2: Building Complete R15 Component Graph");
            CompleteGraph graph = buildCompleteR15Graph("result.json", r15ObstacleIds);

            System.out.println("Complete graph extracted:");
            System.out.println("  Total nodes: " + graph.nodes.size());
            System.out.println("  Total segments: " + graph.segments.size());

            // Step 3: Analyze node degrees
            System.out.println("\nStep 3: Node Degree Analysis");
            analyzeNodeDegrees(graph);

            // Step 4: Select prohibited segment and run Dijkstra
            System.out.println("\nStep 4: Running Dijkstra with Prohibited Segment");
            Segment prohibitedSegment = selectR15ChokepointSegment(graph);
            if (prohibitedSegment != null) {
                System.out.println("Prohibited segment: " + prohibitedSegment);
                DijkstraResult result = runConstrainedDijkstra(graph, prohibitedSegment);

                // Step 5: Generate complete analysis
                System.out.println("\nStep 5: Generating Complete Analysis");
                generateCompleteAnalysis(graph, r15ObstacleIds, prohibitedSegment, result);
            } else {
                System.out.println("No R15 chokepoint segment found for prohibition");
            }

            System.out.println("\nComplete R15 analysis done! Check r15_complete_analysis.json");

        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }

    static Set<Integer> extractR15ObstacleIds(String filename) throws IOException {
        Set<Integer> obstacleIds = new HashSet<>();
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(filename));

        // Find region 15
        JsonNode regions = root.get("regions");
        if (regions != null) {
            for (JsonNode region : regions) {
                if (region.get("id").asInt() == 15) {
                    JsonNode chokepointNodes = region.get("chokepointNodes");
                    System.out.println("R15 chokepoint nodes: " + chokepointNodes);

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

    static CompleteGraph buildCompleteR15Graph(String filename, Set<Integer> r15ObstacleIds) throws IOException {
        CompleteGraph graph = new CompleteGraph();
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(filename));

        // 1. Extract ALL contour segments from R15's obstacles
        JsonNode obstacleContours = root.get("obstacleContours");
        if (obstacleContours != null) {
            for (JsonNode contour : obstacleContours) {
                int obstacleId = contour.get("obstacleId").asInt();
                if (r15ObstacleIds.contains(obstacleId)) {
                    JsonNode points = contour.get("points");
                    if (points != null && points.size() > 1) {
                        System.out.println("Processing obstacle " + obstacleId + " with " + points.size() + " points");

                        // Create segments between consecutive contour points
                        for (int i = 0; i < points.size() - 1; i++) {
                            Point p1 = new Point(points.get(i).get("x").asInt(), points.get(i).get("y").asInt());
                            Point p2 = new Point(points.get(i + 1).get("x").asInt(), points.get(i + 1).get("y").asInt());
                            graph.addSegment(new Segment(p1, p2, "obstacle_" + obstacleId + "_seg_" + i));
                        }
                    }
                }
            }
        }

        // 2. Extract chokepoint crossing segments for R15 (chokepoints 9, 11, 29)
        JsonNode chokepoints = root.get("chokepoints");
        if (chokepoints != null) {
            for (JsonNode cp : chokepoints) {
                int chokepointId = cp.get("id").asInt();
                if (chokepointId == 9 || chokepointId == 11 || chokepointId == 29) {
                    JsonNode crossingPoints = cp.get("crossingPoints");
                    if (crossingPoints != null && crossingPoints.size() == 2) {
                        Point p1 = new Point(crossingPoints.get(0).get("x").asInt(),
                                           crossingPoints.get(0).get("y").asInt());
                        Point p2 = new Point(crossingPoints.get(1).get("x").asInt(),
                                           crossingPoints.get(1).get("y").asInt());
                        graph.addSegment(new Segment(p1, p2, "chokepoint_" + chokepointId));
                        System.out.println("Added chokepoint " + chokepointId + " segment: " + p1 + " -> " + p2);
                    }
                }
            }
        }

        // 3. Extract edge contact segments for R15's obstacles
        JsonNode contourEdgeContacts = root.get("contourEdgeContacts");
        if (contourEdgeContacts != null) {
            for (JsonNode contact : contourEdgeContacts) {
                int obstacleId = contact.get("obstacleId").asInt();
                if (r15ObstacleIds.contains(obstacleId)) {
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

    static void analyzeNodeDegrees(CompleteGraph graph) {
        Map<Integer, Integer> degreeDistribution = new HashMap<>();
        int maxDegree = 0;
        Point maxDegreeNode = null;

        for (Point node : graph.nodes) {
            int degree = graph.getNodeDegree(node);
            degreeDistribution.put(degree, degreeDistribution.getOrDefault(degree, 0) + 1);

            if (degree > maxDegree) {
                maxDegree = degree;
                maxDegreeNode = node;
            }
        }

        System.out.println("Node degree distribution:");
        for (Map.Entry<Integer, Integer> entry : degreeDistribution.entrySet()) {
            System.out.println("  Degree " + entry.getKey() + ": " + entry.getValue() + " nodes");
        }

        if (maxDegreeNode != null) {
            System.out.println("Highest degree node: " + maxDegreeNode + " (degree " + maxDegree + ")");
        }
    }

    static Segment selectR15ChokepointSegment(CompleteGraph graph) {
        // Select chokepoint 11 segment as prohibited (R15 -> R2 connection)
        Point c11p1 = new Point(447, 25);
        Point c11p2 = new Point(417, 92);

        return graph.findSegment(c11p1, c11p2);
    }

    static DijkstraResult runConstrainedDijkstra(CompleteGraph graph, Segment prohibitedSegment) {
        DijkstraResult result = new DijkstraResult();

        if (prohibitedSegment == null) {
            System.out.println("  No prohibited segment found");
            return result;
        }

        Point startPoint = prohibitedSegment.start;
        Point targetPoint = prohibitedSegment.end;

        System.out.println("  Finding alternative path from " + startPoint + " to " + targetPoint);

        // Initialize distances
        for (Point node : graph.nodes) {
            result.distances.put(node, Double.POSITIVE_INFINITY);
        }
        result.distances.put(startPoint, 0.0);

        Set<Point> unvisited = new HashSet<>(graph.nodes);

        while (!unvisited.isEmpty()) {
            // Find unvisited node with minimum distance
            Point current = null;
            double minDist = Double.POSITIVE_INFINITY;
            for (Point node : unvisited) {
                if (result.distances.get(node) < minDist) {
                    minDist = result.distances.get(node);
                    current = node;
                }
            }

            if (current == null || minDist == Double.POSITIVE_INFINITY) break;
            unvisited.remove(current);

            // Check neighbors
            List<Point> neighbors = graph.adjacency.get(current);
            if (neighbors != null) {
                for (Point neighbor : neighbors) {
                    if (!unvisited.contains(neighbor)) continue;

                    // Skip prohibited segment
                    Segment segment = graph.findSegment(current, neighbor);
                    if (segment == prohibitedSegment) {
                        continue;
                    }

                    double newDist = result.distances.get(current) + segment.getLength();
                    if (newDist < result.distances.get(neighbor)) {
                        result.distances.put(neighbor, newDist);
                        result.previous.put(neighbor, current);
                    }
                }
            }
        }

        // Reconstruct path
        if (result.distances.get(targetPoint) != Double.POSITIVE_INFINITY) {
            List<Point> path = new ArrayList<>();
            Point current = targetPoint;
            while (current != null) {
                path.add(0, current);
                current = result.previous.get(current);
            }
            result.path = path;
            result.pathLength = result.distances.get(targetPoint);

            System.out.println("  Found alternative path with " + path.size() + " nodes");
            System.out.println("  Path length: " + result.pathLength);
        } else {
            System.out.println("  No alternative path found");
        }

        return result;
    }

    static void generateCompleteAnalysis(CompleteGraph graph, Set<Integer> r15ObstacleIds,
                                       Segment prohibitedSegment, DijkstraResult dijkstraResult) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode output = mapper.createObjectNode();

        // R15 component information
        ObjectNode r15Info = mapper.createObjectNode();
        ArrayNode obstacleArray = mapper.createArrayNode();
        for (Integer id : r15ObstacleIds) {
            obstacleArray.add(id);
        }
        r15Info.set("relatedObstacleIds", obstacleArray);
        r15Info.put("chokepointIds", "[9, 11, 29]");
        output.set("r15ComponentInfo", r15Info);

        // Complete graph statistics
        ObjectNode graphStats = mapper.createObjectNode();
        graphStats.put("totalNodes", graph.nodes.size());
        graphStats.put("totalSegments", graph.segments.size());

        // Node degree analysis
        Map<Integer, Integer> degreeDistribution = new HashMap<>();
        for (Point node : graph.nodes) {
            int degree = graph.getNodeDegree(node);
            degreeDistribution.put(degree, degreeDistribution.getOrDefault(degree, 0) + 1);
        }

        ObjectNode degreeDist = mapper.createObjectNode();
        for (Map.Entry<Integer, Integer> entry : degreeDistribution.entrySet()) {
            degreeDist.put("degree_" + entry.getKey(), entry.getValue());
        }
        graphStats.set("nodeDegreeDistribution", degreeDist);
        output.set("graphStatistics", graphStats);

        // Dijkstra result
        ObjectNode dijkstraObj = mapper.createObjectNode();
        dijkstraObj.put("pathFound", !dijkstraResult.path.isEmpty());
        dijkstraObj.put("pathLength", dijkstraResult.pathLength);
        dijkstraObj.put("pathNodeCount", dijkstraResult.path.size());

        if (prohibitedSegment != null) {
            ObjectNode prohibitedObj = mapper.createObjectNode();
            prohibitedObj.set("start", pointToJson(mapper, prohibitedSegment.start));
            prohibitedObj.set("end", pointToJson(mapper, prohibitedSegment.end));
            prohibitedObj.put("source", prohibitedSegment.source);
            dijkstraObj.set("prohibitedSegment", prohibitedObj);
        }

        output.set("dijkstraResult", dijkstraObj);

        // Analysis conclusion
        ObjectNode analysis = mapper.createObjectNode();
        analysis.put("approach", "Complete extraction of ALL vertices and segments from R15's related obstacles and chokepoints");
        analysis.put("componentsAnalyzed", "Obstacles " + r15ObstacleIds + " + chokepoints 9,11,29 + edge contacts");
        analysis.put("boundaryCompletionAnalysis", dijkstraResult.path.isEmpty() ?
            "No alternative path found - indicates incomplete geometric connectivity for R15 boundary" :
            "Alternative path found - R15 boundary can be completed through existing connectivity");
        output.set("analysis", analysis);

        // Write output
        mapper.writerWithDefaultPrettyPrinter().writeValue(new File("r15_complete_analysis.json"), output);
    }

    static ObjectNode pointToJson(ObjectMapper mapper, Point p) {
        ObjectNode obj = mapper.createObjectNode();
        obj.put("x", p.x);
        obj.put("y", p.y);
        return obj;
    }
}