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

    private RegionGraph pruneGraph(RegionGraph graph)
    {
        // Mark all nodes with degree != 2 as region candidates BEFORE pruning
        int regionCandidates = 0;
        for(Node n : graph.nodes) {
            int degree = n.neighbours.size();
            if(degree != 2) {
                System.out.println("Marking node @" + n.position + " as region (degree=" + degree + ")");
                graph.setNodeAsRegion(n);
                regionCandidates++;
            }
        }
        System.out.println("Found "+regionCandidates+" region candidate nodes (degree != 2) before pruning");

        // get the list of all leafs (nodes with only one element in the adjacent list)
        LinkedList<Node> nodeToPrune = new LinkedList<>();
        for(Node n:graph.nodes){
            if(n.neighbours.size() == 1){
                nodeToPrune.add(n);
            }
        }

        System.out.println("Beginning region discovery with "+nodeToPrune.size()+" leaf nodes");
        int removed = 0;

        // using leafs as starting points, prune the RegionGraph
        while (!nodeToPrune.isEmpty()) {
            // pop first element
            Node n0 = nodeToPrune.poll();

            // keep isolated nodes
            if (n0.neighbours.isEmpty())  continue;

            System.out.println("processing node @"+n0.position);

            Node n1 = n0.neighbours.iterator().next();

            // find distance to nearest obstacle
            double dist_0 = getDistanceToObstacle(n0);
            System.out.println("Distance to obstacle: "+dist_0);
            double dist_1 = 0;
            boolean deleted = false;
            if(dist_0 < TOO_CLOSE_TO_OBSTACLE){
                System.out.println("Too close, removing!");
                removed++;
                deleted = true;
                graph.removeNode(n0);
            }else{
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
                System.out.println("parent now has degree "+n1.neighbours.size());
                if (n1.neighbours.isEmpty() &&  dist_1 > TOO_CLOSE_TO_OBSTACLE) {
                    // isolated node
                    graph.setNodeAsRegion(n1);
                    System.out.println("marked parent as region");

                } else if (n1.neighbours.size() == 1) { // keep pruning if only one child
                    System.out.println("queued parent for processing");
                    nodeToPrune.add(n1);
                }
            }
        }
        System.out.println("Total nodes removed during discovery "+removed);
        System.out.println("Total nodes retained during discovery: "+graph.nodes.size());
        System.out.println("Total region nodes after pruning: " + graph.regionNodes.size());

        return graph;
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
