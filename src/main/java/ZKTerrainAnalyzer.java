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
        fullContours = traceContours();
        simplifiedContours = simplifyContours();
        voronoid = generateVoronoi((ArrayList<Contour>)simplifiedContours.clone());
        pruned = getPrunedEdges(voronoid);
        labelTree = createRTree();
        regions = pruneGraph(convertGraph(voronoid.get_graph_reference()));
        calculateNodeRadii(regions);
        identifyRegionsAndChokepoints(regions);
        pruneContainedChokepoints(regions);
        skipUnmarkedVertices(regions);
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
        // First: Remove nodes that are too close to obstacles
        System.out.println("Pruning nodes too close to obstacles...");
        int removed = 0;
        ArrayList<Node> nodesToRemove = new ArrayList<>();

        for (Node n : graph.nodes) {
            double dist = getDistanceToObstacle(n);
            if (dist < TOO_CLOSE_TO_OBSTACLE) {
                nodesToRemove.add(n);
            }
        }

        for (Node n : nodesToRemove) {
            graph.removeNode(n);
            removed++;
        }

        System.out.println("Removed " + removed + " nodes too close to obstacles");
        System.out.println("Remaining nodes: " + graph.nodes.size());

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

    private void skipUnmarkedVertices(RegionGraph graph) {
        System.out.println("Skipping through unmarked vertices to connect marked nodes directly...");

        int chainsSkipped = 0;
        int nodesRemoved = 0;

        // Find all unmarked nodes (nodes that are neither regions nor chokepoints)
        ArrayList<Node> unmarkedNodes = new ArrayList<>();
        for (Node n : graph.nodes) {
            if (!graph.regionNodes.contains(n) && !graph.chokepointNodes.contains(n)) {
                unmarkedNodes.add(n);
            }
        }

        System.out.println("Found " + unmarkedNodes.size() + " unmarked nodes to potentially skip");

        // Process each unmarked node
        ArrayList<Node> nodesToRemove = new ArrayList<>();
        for (Node unmarked : unmarkedNodes) {
            if (unmarked.neighbours.size() == 2) {
                // This is a degree-2 node that can be skipped
                Node neighbor1 = null;
                Node neighbor2 = null;

                // Get the two neighbors
                Iterator<Node> neighborIter = unmarked.neighbours.iterator();
                neighbor1 = neighborIter.next();
                neighbor2 = neighborIter.next();

                // Connect the neighbors directly, bypassing this unmarked node
                neighbor1.neighbours.remove(unmarked);
                neighbor2.neighbours.remove(unmarked);

                neighbor1.neighbours.add(neighbor2);
                neighbor2.neighbours.add(neighbor1);

                // Mark this node for removal
                nodesToRemove.add(unmarked);
                nodesRemoved++;

                System.out.println("Skipped unmarked node @" + unmarked.position +
                                 ", directly connected @" + neighbor1.position +
                                 " to @" + neighbor2.position);
            }
        }

        // Remove all the skipped nodes from the graph
        for (Node node : nodesToRemove) {
            graph.nodes.remove(node);
        }

        // Count how many chains were simplified
        chainsSkipped = nodesRemoved;

        System.out.println("Unmarked vertex skipping complete:");
        System.out.println("- Nodes removed: " + nodesRemoved);
        System.out.println("- Remaining nodes: " + graph.nodes.size());
        System.out.println("- Remaining regions: " + graph.regionNodes.size());
        System.out.println("- Remaining chokepoints: " + graph.chokepointNodes.size());
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
