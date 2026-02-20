import com.github.davidmoten.rtree.Entry;
import com.github.davidmoten.rtree.RTree;
import com.github.davidmoten.rtree.geometry.Geometries;
import com.github.davidmoten.rtree.geometry.Line;
import com.goebl.simplify.Simplify;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.*;
import java.util.List;

import org.rogach.jopenvoronoi.*;

public class ZKTerrainAnalyzer {
    BufferedImage bitMap;
    ArrayList<Contour> fullContours;
    ArrayList<Contour> simplifiedContours;
    VoronoiDiagram voronoid;
    Vertex[] errorEdge;
    Vertex[][] borders;
    public ArrayList<LabeledContour> holes;
    int[][] labelMap;
    RTree<Integer, Line> rtree;
    RegionGraph regions;
    ArrayList<Region> regionList;
    ArrayList<Chokepoint> chokepointList;
    double v_width;
    double v_height;
    double radius;
    int width;
    int height;
    static int TOO_CLOSE_TO_OBSTACLE = 15;
    ArrayList<Edge> pruned;

    ZKTerrainAnalyzer(BufferedImage img) {
        width = img.getWidth();
        height = img.getHeight();
        v_width = (double) width;
        v_height = (double) height;
        radius = Math.sqrt(width * width + height * height) / 2;

        bitMap = img;
        labelMap = new int[height][width];
        holes = new ArrayList<>();
        regionList = new ArrayList<>();
        chokepointList = new ArrayList<>();

        // Stage 1: Trace contours
        fullContours = traceContours();
        // Stage 2: Simplify contours
        simplifiedContours = simplifyContours();
        // Stage 3: Build R-tree
        rtree = buildRTree();
        // Stage 4: Generate Voronoi
        voronoid = generateVoronoi((ArrayList<Contour>) simplifiedContours.clone());
        // Stage 5: Build node graph
        pruned = getPrunedEdges(voronoid);
        regions = convertGraph(voronoid.get_graph_reference());
        // Stage 6: Prune leaf nodes
        pruneLeafNodes(regions);
        // Stage 7: Calculate radii
        calculateRadii(regions);
        // Stage 8: Classify nodes (BWTA2)
        classifyNodes(regions);
        // Stage 9: Contract graph
        contractGraph(regions);
        // Stage 10: Compute chokepoint gates
        computeChokepointGates(regions);
        // Stage 11: Split contours at gate points
        splitContoursAtGatePoints();
        // Stage 12: Construct region boundaries
        constructRegionBoundaries();

        System.out.println("Analysis complete: " + regionList.size() + " regions, " + chokepointList.size() + " chokepoints");
    }

    // ---- Public accessors ----

    public int[][] getLabelMap() { return labelMap; }
    public int getHeight() { return height; }
    public int getWidth() { return width; }
    public RegionGraph getRegions() { return regions; }
    public ArrayList<Region> getRegionList() { return regionList; }
    public ArrayList<Chokepoint> getChokepointList() { return chokepointList; }
    public VoronoiDiagram getVoronoid() { return voronoid; }
    public ArrayList<Contour> getContours() { return fullContours; }
    public ArrayList<Contour> getSimplifiedContours() { return simplifiedContours; }

    public Point[] getChokepointSegments(Node chokepointNode) {
        for (Chokepoint cp : chokepointList) {
            if (cp.node == chokepointNode) {
                return cp.crossingPoints;
            }
        }
        return null;
    }

    // ---- Coordinate conversion ----

    double[] coordsToVoronoi(int x, int y) {
        double vx = ((double) x - v_width / 2) / radius;
        double vy = ((double) y - v_height / 2) / radius;
        return new double[]{vx, vy};
    }

    int[] coordsFromVoronoi(Vertex v) {
        return coordsFromVoronoi(v.position);
    }

    int[] coordsFromVoronoi(org.rogach.jopenvoronoi.Point p) {
        return coordsFromVoronoi(p.x, p.y);
    }

    int[] coordsFromVoronoi(double x, double y) {
        int ix = (int) Math.round(x * radius + v_width / 2);
        int iy = (int) Math.round(y * radius + v_height / 2);
        return new int[]{ix, iy};
    }

    private int snap(int value, int attractor, int tolerance) {
        return Math.abs(value - attractor) < tolerance ? attractor : value;
    }

    private int clamp(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }

    // ---- Stage 1: Trace contours ----

    private ArrayList<Contour> traceContours() {
        long timeStarted = System.currentTimeMillis();
        int cy, cx, tracingDirection, connectedComponentsCount = 0, labelId = 0;

        ArrayList<Contour> contours = new ArrayList<>();

        for (cx = 0; cx < width; ++cx) {
            for (cy = 0, labelId = 0; cy < height; ++cy) {
                Color c = new Color(bitMap.getRGB(cx, cy));

                if (c.getRed() > 0) { // is obstacle
                    if (labelId != 0) {
                        labelMap[cy][cx] = labelId;
                    } else {
                        labelId = labelMap[cy][cx];
                        if (labelId == 0) {
                            labelId = ++connectedComponentsCount;
                            tracingDirection = 0;
                            Contour obstacle = traceContour(cx, cy, labelId, tracingDirection);
                            contours.add(obstacle);
                            labelMap[cy][cx] = labelId;
                        }
                    }
                } else if (labelId != 0) {
                    if (labelMap[cy][cx] == 0) {
                        tracingDirection = 1;
                        LabeledContour hole = new LabeledContour(traceContour(cx, cy - 1, labelId, tracingDirection), labelId);
                        fillHole(hole, labelId);
                        holes.add(hole);
                    }
                    labelId = 0;
                }
            }
        }

        System.out.println("Traced " + contours.size() + " contours in " + (System.currentTimeMillis() - timeStarted) + "ms");
        return contours;
    }

    private Contour traceContour(int cx, int cy, int labelId, int tracingDirection) {
        boolean tracingStopFlag = false, keepSearching = true;
        int fx, fy, sx = cx, sy = cy;
        Contour contourPoints = new Contour();
        TracePoint p = tracer(cx, cy, tracingDirection);
        contourPoints.addPoint(p);

        if (p.x != sx || p.y != sy) {
            fx = p.x;
            fy = p.y;
            while (keepSearching) {
                labelMap[p.y][p.x] = labelId;
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

    static int[][] searchDirection = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

    private TracePoint tracer(int cx, int cy, int tracingdirection) {
        for (int i = 0; i < 7; ++i) {
            int y = cy + searchDirection[tracingdirection][0];
            int x = cx + searchDirection[tracingdirection][1];

            if (x < 0 || x >= width || y < 0 || y >= height) {
                tracingdirection = (tracingdirection + 1) % 8;
                continue;
            }
            Color c = new Color(bitMap.getRGB(x, y));
            if (c.getRed() > 0) {
                return new TracePoint(x, y, tracingdirection);
            } else {
                labelMap[y][x] = -1;
                tracingdirection = (tracingdirection + 1) % 8;
            }
        }
        return new TracePoint(cx, cy, tracingdirection);
    }

    private void fillHole(Contour hole, int labelID) {
        int[] bounds = hole.getBounds();
        ArrayList<Point> points = hole.getPoints();

        for (int y = bounds[2]; y < bounds[3]; y++) {
            ArrayList<Integer> scanLine = new ArrayList<>();
            for (Point p : points) {
                if (p.y == y) {
                    scanLine.add(p.x);
                }
            }
            Integer[] intersects = new Integer[scanLine.size()];
            scanLine.toArray(intersects);
            Arrays.sort(intersects);

            if (intersects.length > 1) {
                boolean inside = false;
                for (int n = 0; n < intersects.length - 1; n += 1) {
                    inside = !inside;
                    int min = intersects[n];
                    int max = intersects[n + 1];
                    if (max <= min + 1) {
                        inside = false;
                    }
                    if (inside) {
                        for (int x = min; x < max; x++) {
                            labelMap[y][x] = labelID;
                        }
                    }
                }
            }
        }
    }

    // ---- Stage 2: Simplify contours ----

    private ArrayList<Contour> simplifyContours() {
        long timeStarted = System.currentTimeMillis();
        ArrayList<Contour> simple = new ArrayList<>();
        for (Contour c : fullContours) {
            Contour simplified = simplifyContour(c);
            if (simplified.length() > 3 && simplified.getArea() > 100.0) {
                simple.add(simplified);
            }
        }
        System.out.println("Simplified to " + simple.size() + " contours in " + (System.currentTimeMillis() - timeStarted) + "ms");
        return simple;
    }

    private Contour simplifyContour(Contour c) {
        Simplify<Point> simplify = new Simplify<>(new Point[0]);
        Point[] allPoints = c.getPointsArray();
        Point[] lessPoints = simplify.simplify(allPoints, 8, true);
        return new Contour(lessPoints);
    }

    // ---- Stage 3: Build R-tree ----

    private RTree<Integer, Line> buildRTree() {
        long timeStarted = System.currentTimeMillis();
        RTree<Integer, Line> tree = RTree.create();
        for (int i = 0; i < simplifiedContours.size(); i++) {
            Contour c = simplifiedContours.get(i);
            int obstacleId = i + 1;
            for (int j = 1; j < c.points.size(); j++) {
                Point p1 = c.points.get(j - 1);
                Point p2 = c.points.get(j);
                tree = tree.add(obstacleId, Geometries.line(p1.x, p1.y, p2.x, p2.y));
            }
            // closing edge: last point to first point
            if (c.points.size() > 2) {
                Point last = c.points.get(c.points.size() - 1);
                Point first = c.points.get(0);
                if (!last.equals(first)) {
                    tree = tree.add(obstacleId, Geometries.line(last.x, last.y, first.x, first.y));
                }
            }
        }
        System.out.println("Built R-tree in " + (System.currentTimeMillis() - timeStarted) + "ms");
        return tree;
    }

    // ---- Stage 4: Generate Voronoi ----

    private VoronoiDiagram generateVoronoi(ArrayList<Contour> obstacles) {
        long timeStarted = System.currentTimeMillis();
        VoronoiDiagram vd = new VoronoiDiagram();

        Point[] boundingBox = {
            new Point(1, 1), new Point(width - 1, 1),
            new Point(width - 1, height - 1), new Point(1, height - 1)
        };

        Vertex[] cornerVertices = new Vertex[4];
        ArrayList<Vertex> allVertices = new ArrayList<>();
        Vertex[][] contourVertices = new Vertex[obstacles.size()][];

        // Add contour vertices
        for (int vi = 0; vi < obstacles.size(); vi++) {
            Point[] pts = obstacles.get(vi).getPointsArray();
            Vertex[] vertices = new Vertex[pts.length - 1];
            for (int j = 0; j < pts.length - 1; j++) {
                int px = snap(snap(pts[j].x, width, 5), 1, 5);
                int py = snap(snap(pts[j].y, height, 5), 1, 5);
                double[] vc = coordsToVoronoi(px, py);
                vertices[j] = vd.insert_point_site(new org.rogach.jopenvoronoi.Point(vc[0], vc[1]));
                allVertices.add(vertices[j]);
            }
            contourVertices[vi] = vertices;
        }

        // Add corner vertices
        for (int z = 0; z < boundingBox.length; z++) {
            Point p = boundingBox[z];
            double[] vc = coordsToVoronoi(p.x, p.y);
            cornerVertices[z] = vd.insert_point_site(new org.rogach.jopenvoronoi.Point(vc[0], vc[1]));
        }

        // Build borders
        borders = new Vertex[4][];
        double tolerance = 0.005;
        borders[0] = getHorizontalBorder(allVertices, cornerVertices[0], cornerVertices[1], tolerance); // north
        borders[1] = getHorizontalBorder(allVertices, cornerVertices[2], cornerVertices[3], tolerance); // south
        borders[2] = getVerticalBorder(allVertices, cornerVertices[1], cornerVertices[2], tolerance);   // east
        borders[3] = getVerticalBorder(allVertices, cornerVertices[0], cornerVertices[3], tolerance);   // west

        // Insert contour line sites
        for (int vi = 0; vi < contourVertices.length; vi++) {
            for (int j = 0; j < contourVertices[vi].length; j++) {
                int k = j + 1 < contourVertices[vi].length ? j + 1 : 0;
                boolean novel = true;
                isNovel:
                for (int l = 0; l < borders.length; l++) {
                    for (int n = 1; n < borders[l].length; n++) {
                        if (borders[l][n - 1] == contourVertices[vi][j] && borders[l][n] == contourVertices[vi][k]) {
                            novel = false;
                            break isNovel;
                        }
                        if (borders[l][n] == contourVertices[vi][j] && borders[l][n - 1] == contourVertices[vi][k]) {
                            novel = false;
                            break isNovel;
                        }
                    }
                }
                if (novel) {
                    try {
                        vd.insert_line_site(contourVertices[vi][j], contourVertices[vi][k]);
                    } catch (Exception e) {
                        this.errorEdge = new Vertex[]{contourVertices[vi][j], contourVertices[vi][k]};
                        return vd;
                    }
                }
            }
        }

        // Insert border line sites and add to R-tree as obstacle 0 (map boundary)
        for (int l = 0; l < borders.length; l++) {
            for (int n = 1; n < borders[l].length; n++) {
                try {
                    vd.insert_line_site(borders[l][n - 1], borders[l][n]);
                } catch (Exception e) {
                    e.printStackTrace();
                    this.errorEdge = new Vertex[]{borders[l][n - 1], borders[l][n]};
                    return vd;
                }
                int[] c1 = coordsFromVoronoi(borders[l][n - 1]);
                int[] c2 = coordsFromVoronoi(borders[l][n]);
                rtree = rtree.add(0, Geometries.line(c1[0], c1[1], c2[0], c2[1]));
            }
        }

        System.out.println("Generated Voronoi in " + (System.currentTimeMillis() - timeStarted) + "ms");
        return vd;
    }

    private Vertex[] getHorizontalBorder(ArrayList<Vertex> vertices, Vertex left, Vertex right, double tolerance) {
        ArrayList<Vertex> inTheWay = new ArrayList<>();
        double h = (left.position.y + right.position.y) / 2;
        inTheWay.add(left);
        inTheWay.add(right);
        for (Vertex v : vertices) {
            if (Math.abs(v.position.y - h) < tolerance) {
                inTheWay.add(v);
            }
        }
        inTheWay.sort((a, b) -> (int) Math.signum(a.position.x - b.position.x));
        return inTheWay.toArray(new Vertex[0]);
    }

    private Vertex[] getVerticalBorder(ArrayList<Vertex> vertices, Vertex top, Vertex bottom, double tolerance) {
        ArrayList<Vertex> inTheWay = new ArrayList<>();
        double w = (top.position.x + bottom.position.x) / 2;
        inTheWay.add(top);
        inTheWay.add(bottom);
        for (Vertex v : vertices) {
            if (Math.abs(v.position.x - w) < tolerance) {
                inTheWay.add(v);
            }
        }
        inTheWay.sort((a, b) -> (int) Math.signum(a.position.y - b.position.y));
        return inTheWay.toArray(new Vertex[0]);
    }

    // ---- Stage 5: Build node graph ----

    private ArrayList<Edge> getPrunedEdges(VoronoiDiagram voronoid) {
        HalfEdgeDiagram hed = voronoid.get_graph_reference();
        ArrayList<Edge> pruned = new ArrayList<>();

        for (Vertex v : hed.vertices) {
            int[] c = coordsFromVoronoi(v);
            c[0] = snap(snap(c[0], 0, 5), width, 5);
            c[1] = snap(snap(c[1], 0, 5), height, 5);

            if (c[0] < 0 || c[0] >= width || c[1] < 0 || c[1] >= height) {
                pruned.addAll(v.out_edges);
                pruned.addAll(v.in_edges);
            } else if (labelMap[c[1]][c[0]] > 0) {
                pruned.addAll(v.out_edges);
                pruned.addAll(v.in_edges);
            }
        }
        System.out.println(pruned.size() + " Voronoi edges pruned");
        return pruned;
    }

    private RegionGraph convertGraph(HalfEdgeDiagram g) {
        HashMap<Vertex, Node> nodes = new HashMap<>();
        for (Edge e : g.edges) {
            if (e.valid && !pruned.contains(e)) {
                Node n1 = vertexToNode(nodes, e.source);
                Node n2 = vertexToNode(nodes, e.target);
                n1.addNeighbour(n2);
                n2.addNeighbour(n1);
            }
        }
        System.out.println("Converted Voronoi to graph with " + nodes.size() + " nodes");
        return new RegionGraph(nodes.values());
    }

    private Node vertexToNode(HashMap<Vertex, Node> nodes, Vertex v) {
        if (nodes.containsKey(v)) {
            return nodes.get(v);
        }
        int[] coords = this.coordsFromVoronoi(v);
        Node n = new Node(coords[0], coords[1]);
        nodes.put(v, n);
        return n;
    }

    // ---- Stage 6: Prune leaf nodes ----

    private void pruneLeafNodes(RegionGraph graph) {
        LinkedList<Node> toPrune = new LinkedList<>();
        for (Node n : graph.nodes) {
            if (n.neighbours.size() == 1) {
                toPrune.add(n);
            }
        }

        int removed = 0;
        while (!toPrune.isEmpty()) {
            Node n0 = toPrune.poll();
            if (n0.neighbours.isEmpty()) continue;

            Node n1 = n0.neighbours.iterator().next();
            double dist = getDistanceToObstacle(n0);

            if (dist < TOO_CLOSE_TO_OBSTACLE) {
                removed++;
                graph.removeNode(n0);
                if (n1.neighbours.size() == 1) {
                    toPrune.add(n1);
                }
            }
        }
        System.out.println("Pruned " + removed + " leaf nodes, " + graph.nodes.size() + " remaining");
    }

    // ---- Stage 7: Calculate radii ----

    private void calculateRadii(RegionGraph graph) {
        for (Node n : graph.nodes) {
            n.radius = getDistanceToObstacle(n);
        }
    }

    private double getDistanceToObstacle(Node n) {
        double searchRadius = radius * 2;
        rx.Observable<Entry<Integer, Line>> closest = rtree.nearest(
            Geometries.point(n.position.x, n.position.y), searchRadius, 1);
        Entry<Integer, Line> entry = closest.toBlocking().first();
        return distanceToLine(n.position, entry.geometry());
    }

    private double distanceToLine(Point p, Line l) {
        double x1 = l.x1(), y1 = l.y1();
        double x2 = l.x2(), y2 = l.y2();
        double px = p.x, py = p.y;

        double dx = x2 - x1;
        double dy = y2 - y1;

        if (dx == 0 && dy == 0) {
            return Math.sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
        }

        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));

        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        return Math.sqrt((px - closestX) * (px - closestX) + (py - closestY) * (py - closestY));
    }

    // ---- Stage 8: Classify nodes (BWTA2) ----

    private void classifyNodes(RegionGraph graph) {
        for (Node n : graph.nodes) {
            n.visited = false;
            n.parent = null;
        }

        Queue<Node> toProcess = new LinkedList<>();
        for (Node n : graph.nodes) {
            if (n.neighbours.size() == 1) {
                toProcess.add(n);
                graph.setNodeAsRegion(n);
                n.visited = true;
            }
        }

        while (!toProcess.isEmpty()) {
            Node current = toProcess.poll();
            for (Node neighbor : current.neighbours) {
                if (!neighbor.visited) {
                    neighbor.visited = true;
                    neighbor.parent = current;
                    toProcess.add(neighbor);

                    int degree = neighbor.neighbours.size();

                    if (degree != 2) {
                        graph.setNodeAsRegion(neighbor);
                    } else {
                        if (neighbor.isLocalMinimum()) {
                            if (current.isLocalMinimum()) {
                                if (neighbor.radius < current.radius) {
                                    graph.setNodeAsChokepoint(neighbor);
                                }
                            } else {
                                graph.setNodeAsChokepoint(neighbor);
                            }
                        } else if (neighbor.isLocalMaximum()) {
                            if (current.isLocalMaximum()) {
                                if (neighbor.radius > current.radius) {
                                    graph.setNodeAsRegion(neighbor);
                                }
                            } else {
                                graph.setNodeAsRegion(neighbor);
                            }
                        }
                    }
                }
            }
        }

        System.out.println("Classified: " + graph.regionNodes.size() + " regions, " +
            graph.chokepointNodes.size() + " chokepoints, " +
            (graph.nodes.size() - graph.regionNodes.size() - graph.chokepointNodes.size()) + " unmarked");
    }

    // ---- Stage 9: Contract graph ----

    private void contractGraph(RegionGraph graph) {
        // Skip unmarked vertices: connect marked nodes through chains of unmarked nodes
        HashSet<Node> marked = new HashSet<>();
        marked.addAll(graph.regionNodes);
        marked.addAll(graph.chokepointNodes);

        HashMap<Node, HashSet<Node>> newNeighbors = new HashMap<>();
        for (Node m : marked) {
            newNeighbors.put(m, new HashSet<>());
        }

        for (Node m : marked) {
            // BFS through unmarked nodes to find connected marked nodes
            Queue<Node> queue = new LinkedList<>();
            HashSet<Node> visited = new HashSet<>();
            visited.add(m);

            for (Node n : m.neighbours) {
                if (marked.contains(n)) {
                    // Direct marked-to-marked connection
                    newNeighbors.get(m).add(n);
                } else if (!visited.contains(n)) {
                    queue.add(n);
                    visited.add(n);
                }
            }

            while (!queue.isEmpty()) {
                Node current = queue.poll();
                for (Node n : current.neighbours) {
                    if (visited.contains(n)) continue;
                    visited.add(n);
                    if (marked.contains(n)) {
                        newNeighbors.get(m).add(n);
                    } else {
                        queue.add(n);
                    }
                }
            }
        }

        // Apply new adjacency
        for (Node m : marked) {
            m.neighbours.clear();
            m.neighbours.addAll(newNeighbors.get(m));
        }

        // Remove all unmarked nodes
        graph.nodes.retainAll(marked);

        // Merge contiguous same-type nodes
        mergeContiguous(graph);

        System.out.println("After contraction: " + graph.regionNodes.size() + " regions, " +
            graph.chokepointNodes.size() + " chokepoints, " + graph.nodes.size() + " total nodes");
    }

    private void mergeContiguous(RegionGraph graph) {
        boolean merged = true;
        while (merged) {
            merged = false;

            // Merge adjacent regions: keep larger radius
            for (Node r : new ArrayList<>(graph.regionNodes)) {
                if (!graph.regionNodes.contains(r)) continue;
                for (Node n : new ArrayList<>(r.neighbours)) {
                    if (graph.regionNodes.contains(n) && n != r) {
                        Node keep = r.radius >= n.radius ? r : n;
                        Node remove = keep == r ? n : r;
                        // Transfer neighbors
                        for (Node nn : remove.neighbours) {
                            if (nn != keep) {
                                nn.neighbours.remove(remove);
                                nn.neighbours.add(keep);
                                keep.neighbours.add(nn);
                            }
                        }
                        keep.neighbours.remove(remove);
                        graph.regionNodes.remove(remove);
                        graph.nodes.remove(remove);
                        merged = true;
                        break;
                    }
                }
                if (merged) break;
            }
            if (merged) continue;

            // Merge adjacent chokepoints: keep smaller radius (narrowest)
            for (Node c : new ArrayList<>(graph.chokepointNodes)) {
                if (!graph.chokepointNodes.contains(c)) continue;
                for (Node n : new ArrayList<>(c.neighbours)) {
                    if (graph.chokepointNodes.contains(n) && n != c) {
                        Node keep = c.radius <= n.radius ? c : n;
                        Node remove = keep == c ? n : c;
                        for (Node nn : remove.neighbours) {
                            if (nn != keep) {
                                nn.neighbours.remove(remove);
                                nn.neighbours.add(keep);
                                keep.neighbours.add(nn);
                            }
                        }
                        keep.neighbours.remove(remove);
                        graph.chokepointNodes.remove(remove);
                        graph.nodes.remove(remove);
                        merged = true;
                        break;
                    }
                }
                if (merged) break;
            }
        }
    }

    // ---- Stage 10: Compute chokepoint gates ----

    private void computeChokepointGates(RegionGraph graph) {
        int chokepointId = 0;
        for (Node cpNode : graph.chokepointNodes) {
            Chokepoint cp = new Chokepoint(cpNode, chokepointId++);

            CrossingData crossingData = findCrossingPoints(cpNode);
            if (crossingData != null) {
                cp.setCrossingData(crossingData.crossingPoints, crossingData.obstacleIds);
            }

            // Link chokepoint to adjacent regions
            for (Node neighbor : cpNode.neighbours) {
                if (graph.regionNodes.contains(neighbor)) {
                    // Find or create Region for this node
                    Region region = findOrCreateRegion(neighbor, graph);
                    cp.addRegion(region);
                    region.addChokepoint(cpNode);
                }
            }

            chokepointList.add(cp);
            graph.chokepoints.add(cp);
        }

        // Ensure all region nodes have a Region object
        for (Node rNode : graph.regionNodes) {
            findOrCreateRegion(rNode, graph);
        }

        System.out.println("Computed " + chokepointList.size() + " chokepoint gates");
    }

    private Region findOrCreateRegion(Node regionNode, RegionGraph graph) {
        for (Region r : regionList) {
            if (r.seedNode == regionNode) return r;
        }
        Region r = new Region(regionNode, regionList.size());
        regionList.add(r);
        return r;
    }

    private CrossingData findCrossingPoints(Node cpNode) {
        // Find the two closest obstacle segments from different obstacles
        double searchRadius = radius * 2;
        rx.Observable<Entry<Integer, Line>> nearest = rtree.nearest(
            Geometries.point(cpNode.position.x, cpNode.position.y), searchRadius, 10);

        List<Entry<Integer, Line>> entries = nearest.toList().toBlocking().single();

        if (entries.size() < 2) return null;

        // Find two entries from different obstacles
        Entry<Integer, Line> first = entries.get(0);
        Entry<Integer, Line> second = null;

        for (int i = 1; i < entries.size(); i++) {
            if (!entries.get(i).value().equals(first.value())) {
                second = entries.get(i);
                break;
            }
        }

        if (second == null) return null;

        Point cp1 = closestPointOnLine(cpNode.position, first.geometry());
        Point cp2 = closestPointOnLine(cpNode.position, second.geometry());

        return new CrossingData(
            new Point[]{cp1, cp2},
            new Integer[]{first.value(), second.value()}
        );
    }

    private Point closestPointOnLine(Point p, Line l) {
        double x1 = l.x1(), y1 = l.y1();
        double x2 = l.x2(), y2 = l.y2();
        double px = p.x, py = p.y;

        double dx = x2 - x1;
        double dy = y2 - y1;

        if (dx == 0 && dy == 0) {
            return new Point((int) x1, (int) y1);
        }

        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));

        return new Point((int) Math.round(x1 + t * dx), (int) Math.round(y1 + t * dy));
    }

    // ---- Stage 11: Split contours at gate points ----

    private void splitContoursAtGatePoints() {
        // For each chokepoint, insert its crossing points into the corresponding contours
        for (Chokepoint cp : chokepointList) {
            if (cp.crossingPoints == null || cp.obstacleIds == null) continue;

            for (int i = 0; i < cp.crossingPoints.length; i++) {
                Point crossingPoint = cp.crossingPoints[i];
                int obstacleId = cp.obstacleIds[i];

                if (obstacleId <= 0 || obstacleId > simplifiedContours.size()) continue;
                Contour contour = simplifiedContours.get(obstacleId - 1);

                // Find the segment this point is on and insert it
                insertPointIntoContour(contour, crossingPoint);
            }
        }
    }

    private void insertPointIntoContour(Contour contour, Point point) {
        // Check if point already exists in contour
        for (Point p : contour.points) {
            if (p.equals(point)) return;
        }

        // Find the nearest segment and insert the point
        double minDist = Double.MAX_VALUE;
        int bestIdx = -1;

        for (int i = 0; i < contour.points.size(); i++) {
            Point a = contour.points.get(i);
            Point b = contour.points.get((i + 1) % contour.points.size());
            double dist = distanceToSegment(point, a, b);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = i;
            }
        }

        if (bestIdx >= 0 && minDist < 5.0) {
            contour.points.add(bestIdx + 1, point);
        }
    }

    private double distanceToSegment(Point p, Point a, Point b) {
        double dx = b.x - a.x, dy = b.y - a.y;
        if (dx == 0 && dy == 0) {
            return Math.sqrt((p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y));
        }
        double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));
        double cx = a.x + t * dx, cy = a.y + t * dy;
        return Math.sqrt((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy));
    }

    // ---- Stage 12: Construct region boundaries ----

    private void constructRegionBoundaries() {
        for (Region region : regionList) {
            ArrayList<Point> boundary = buildRegionBoundary(region);
            if (boundary != null && boundary.size() >= 3) {
                region.setPolygon(boundary);
                region.isClosed = true;
            }
        }
        int closedCount = 0;
        for (Region r : regionList) {
            if (r.isClosed) closedCount++;
        }
        System.out.println("Built boundaries: " + closedCount + "/" + regionList.size() + " closed");
    }

    private ArrayList<Point> buildRegionBoundary(Region region) {
        // Collect gate endpoints for this region's chokepoints
        ArrayList<Point[]> gates = new ArrayList<>();
        ArrayList<Integer[]> gateObstacleIds = new ArrayList<>();

        for (Node cpNode : region.chokepoints) {
            for (Chokepoint cp : chokepointList) {
                if (cp.node == cpNode && cp.crossingPoints != null && cp.crossingPoints.length == 2
                        && cp.obstacleIds != null && cp.obstacleIds.length == 2) {
                    gates.add(cp.crossingPoints);
                    gateObstacleIds.add(cp.obstacleIds);
                    break;
                }
            }
        }

        if (gates.isEmpty()) {
            return buildSingleContourBoundary(region);
        }

        // All gated regions (single or multi) use the Dijkstra approach
        return buildGatedBoundary(region, gates, gateObstacleIds);
    }

    private ArrayList<Point> buildSingleContourBoundary(Region region) {
        // Find nearest obstacle contour
        double minDist = Double.MAX_VALUE;
        Contour nearest = null;

        for (Contour c : simplifiedContours) {
            for (Point p : c.points) {
                double d = Math.sqrt(Math.pow(p.x - region.getCenter().x, 2) +
                    Math.pow(p.y - region.getCenter().y, 2));
                if (d < minDist) {
                    minDist = d;
                    nearest = c;
                }
            }
        }

        if (nearest == null) return null;

        // Classify each contour vertex as on/off the map edge
        int n = nearest.points.size();
        boolean[] onEdge = new boolean[n];
        int edgeCount = 0;
        for (int i = 0; i < n; i++) {
            onEdge[i] = isOnMapEdge(nearest.points.get(i));
            if (onEdge[i]) edgeCount++;
        }

        if (edgeCount == 0 || edgeCount == n) {
            // Entirely on or off the edge - just return contour
            return new ArrayList<>(nearest.points);
        }

        // Find transition points: where the contour exits the map edge (last on-edge
        // vertex before an off-edge vertex) and where it enters (first on-edge vertex
        // after an off-edge vertex). These bound the interior arc.
        int exitIdx = -1, enterIdx = -1;
        for (int i = 0; i < n; i++) {
            int next = (i + 1) % n;
            if (onEdge[i] && !onEdge[next]) {
                exitIdx = i;  // last on-edge before going interior
            }
            if (!onEdge[i] && onEdge[next]) {
                enterIdx = next;  // first on-edge after interior
            }
        }

        if (exitIdx < 0 || enterIdx < 0) {
            return new ArrayList<>(nearest.points);
        }

        // The interior arc goes from exitIdx to enterIdx (through the off-edge vertices).
        // Include the transition vertices themselves (they're on the map edge and serve
        // as the connection points for the map-edge closure).
        ArrayList<Point> obstacleArc = new ArrayList<>();
        for (int i = exitIdx; ; i = (i + 1) % n) {
            obstacleArc.add(nearest.points.get(i));
            if (i == enterIdx) break;
        }

        // Now close the boundary with map-edge segments from arc end back to arc start
        Point arcEnd = obstacleArc.get(obstacleArc.size() - 1);
        Point arcStart = obstacleArc.get(0);

        ArrayList<Point> mapPath = getMapEdgePath(arcEnd, arcStart);
        if (mapPath != null && mapPath.size() > 1) {
            // Skip first point (it's arcEnd, already in boundary)
            for (int i = 1; i < mapPath.size(); i++) {
                obstacleArc.add(mapPath.get(i));
            }
        }

        return obstacleArc;
    }

    private ArrayList<Point> getMapEdgePath(Point from, Point to) {
        // Build a small graph of just map-edge segments and find the path
        HashMap<Point, HashSet<Point>> mapGraph = new HashMap<>();

        Point[] corners = {
            new Point(1, 1), new Point(width - 2, 1),
            new Point(width - 2, height - 2), new Point(1, height - 2)
        };

        // Add the 4 map edges as connected segments
        ArrayList<Point> allEdgePoints = new ArrayList<>();
        allEdgePoints.add(from);
        allEdgePoints.add(to);
        for (Point c : corners) {
            allEdgePoints.add(c);
        }

        connectEdgePoints(mapGraph, allEdgePoints, corners, 0); // top
        connectEdgePoints(mapGraph, allEdgePoints, corners, 1); // right
        connectEdgePoints(mapGraph, allEdgePoints, corners, 2); // bottom
        connectEdgePoints(mapGraph, allEdgePoints, corners, 3); // left

        return dijkstra(mapGraph, from, to, null);
    }

    private ArrayList<Point> buildGatedBoundary(Region region, ArrayList<Point[]> gates,
                                                 ArrayList<Integer[]> gateObstacleIds) {
        // Collect relevant obstacle contour IDs (skip map boundary ID 0)
        HashSet<Integer> relevantObstacles = new HashSet<>();
        for (Integer[] ids : gateObstacleIds) {
            for (Integer id : ids) {
                if (id > 0 && id <= simplifiedContours.size()) {
                    relevantObstacles.add(id);
                }
            }
        }

        // Build adjacency graph: contour edges + gate edges + map boundary edges
        HashMap<Point, HashSet<Point>> adjGraph = new HashMap<>();

        // 1. Add obstacle contour edges
        for (int obstacleId : relevantObstacles) {
            Contour c = simplifiedContours.get(obstacleId - 1);
            for (int i = 0; i < c.points.size(); i++) {
                Point p1 = c.points.get(i);
                Point p2 = c.points.get((i + 1) % c.points.size());
                addBidirectionalEdge(adjGraph, p1, p2);
            }
        }

        // 2. Add gate edges (so gate endpoints are vertices in the graph)
        for (Point[] gate : gates) {
            addBidirectionalEdge(adjGraph, gate[0], gate[1]);
        }

        // 3. Always add map boundary edges: even when no gate directly touches
        // obstacle 0, the boundary may need to bridge between different obstacle
        // contours along the map edge.
        addMapBoundaryEdges(adjGraph);

        // Dijkstra: find shortest path from gate[0] endpoint A to endpoint B,
        // going the long way around (prohibiting the direct gate edge)
        Point start = gates.get(0)[0];
        Point goal = gates.get(0)[1];

        ArrayList<Point> path = dijkstra(adjGraph, start, goal, gates.get(0));

        if (path != null && path.size() >= 3) {
            return path;
        }

        return null;
    }

    private void addBidirectionalEdge(HashMap<Point, HashSet<Point>> graph, Point a, Point b) {
        graph.computeIfAbsent(a, k -> new HashSet<>()).add(b);
        graph.computeIfAbsent(b, k -> new HashSet<>()).add(a);
    }

    private void addMapBoundaryEdges(HashMap<Point, HashSet<Point>> adjGraph) {
        // Collect ALL points already in the graph that lie on a map edge.
        // This includes gate endpoints on the map edge and contour vertices on the map edge.
        ArrayList<Point> edgePoints = new ArrayList<>();
        for (Point p : adjGraph.keySet()) {
            if (isOnMapEdge(p)) {
                edgePoints.add(p);
            }
        }

        Point[] corners = {
            new Point(1, 1), new Point(width - 2, 1),
            new Point(width - 2, height - 2), new Point(1, height - 2)
        };

        // Connect points along each of the 4 map edges
        connectEdgePoints(adjGraph, edgePoints, corners, 0); // top
        connectEdgePoints(adjGraph, edgePoints, corners, 1); // right
        connectEdgePoints(adjGraph, edgePoints, corners, 2); // bottom
        connectEdgePoints(adjGraph, edgePoints, corners, 3); // left
    }

    private boolean isOnMapEdge(Point p) {
        return p.x <= 1 || p.x >= width - 2 || p.y <= 1 || p.y >= height - 2;
    }

    private void connectEdgePoints(HashMap<Point, HashSet<Point>> adjGraph,
                                    ArrayList<Point> edgePoints, Point[] corners, int edgeIndex) {
        ArrayList<Point> onThisEdge = new ArrayList<>();

        for (Point p : edgePoints) {
            boolean onEdge = false;
            switch (edgeIndex) {
                case 0: onEdge = p.y <= 1; break;
                case 1: onEdge = p.x >= width - 2; break;
                case 2: onEdge = p.y >= height - 2; break;
                case 3: onEdge = p.x <= 1; break;
            }
            if (onEdge) onThisEdge.add(p);
        }

        if (onThisEdge.isEmpty()) return;

        // Add the two corners for this edge
        switch (edgeIndex) {
            case 0: // top: corners[0] and corners[1]
                onThisEdge.add(corners[0]);
                onThisEdge.add(corners[1]);
                onThisEdge.sort(Comparator.comparingInt(p -> p.x));
                break;
            case 1: // right: corners[1] and corners[2]
                onThisEdge.add(corners[1]);
                onThisEdge.add(corners[2]);
                onThisEdge.sort(Comparator.comparingInt(p -> p.y));
                break;
            case 2: // bottom: corners[2] and corners[3]
                onThisEdge.add(corners[2]);
                onThisEdge.add(corners[3]);
                onThisEdge.sort(Comparator.comparingInt(p -> p.x));
                break;
            case 3: // left: corners[3] and corners[0]
                onThisEdge.add(corners[3]);
                onThisEdge.add(corners[0]);
                onThisEdge.sort(Comparator.comparingInt(p -> p.y));
                break;
        }

        // Connect consecutive points along this edge
        for (int i = 0; i < onThisEdge.size() - 1; i++) {
            addBidirectionalEdge(adjGraph, onThisEdge.get(i), onThisEdge.get(i + 1));
        }
    }

    private ArrayList<Point> dijkstra(HashMap<Point, HashSet<Point>> graph, Point start, Point goal,
                                       Point[] prohibitedGate) {
        HashMap<Point, Double> dist = new HashMap<>();
        HashMap<Point, Point> prev = new HashMap<>();
        PriorityQueue<Point> queue = new PriorityQueue<>(Comparator.comparingDouble(p -> dist.getOrDefault(p, Double.MAX_VALUE)));

        dist.put(start, 0.0);
        queue.add(start);

        while (!queue.isEmpty()) {
            Point current = queue.poll();

            if (current.equals(goal)) {
                // Reconstruct path
                ArrayList<Point> path = new ArrayList<>();
                Point p = goal;
                while (p != null) {
                    path.add(0, p);
                    p = prev.get(p);
                }
                return path;
            }

            double currentDist = dist.getOrDefault(current, Double.MAX_VALUE);
            HashSet<Point> neighbors = graph.get(current);
            if (neighbors == null) continue;

            for (Point neighbor : neighbors) {
                // Prohibit direct gate edge
                if (prohibitedGate != null) {
                    if ((current.equals(prohibitedGate[0]) && neighbor.equals(prohibitedGate[1])) ||
                        (current.equals(prohibitedGate[1]) && neighbor.equals(prohibitedGate[0]))) {
                        continue;
                    }
                }

                double edgeDist = Math.sqrt(Math.pow(current.x - neighbor.x, 2) +
                    Math.pow(current.y - neighbor.y, 2));
                double newDist = currentDist + edgeDist;

                if (newDist < dist.getOrDefault(neighbor, Double.MAX_VALUE)) {
                    dist.put(neighbor, newDist);
                    prev.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        return null; // No path found
    }

}
