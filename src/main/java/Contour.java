import java.util.ArrayList;
import java.util.Collections;

public class Contour {
    ArrayList<Point> points;
    ArrayList<ChokepointContact> chokepointContacts; // Chokepoints that touch this contour
    ArrayList<EdgeContact> edgeContacts; // Map edge contacts on this contour

    public static class ChokepointContact {
        public Node chokepoint;
        public Point contactPoint;
        public int segmentIndex; // Which segment of the contour contains this contact

        public ChokepointContact(Node chokepoint, Point contactPoint, int segmentIndex) {
            this.chokepoint = chokepoint;
            this.contactPoint = contactPoint;
            this.segmentIndex = segmentIndex;
        }
    }

    public static class EdgeContact {
        public Point contactPoint;
        public int segmentIndex; // Which segment of the contour contains this contact
        public ZKTerrainAnalyzer.EdgeSegment edgeSegment; // The specific edge segment we contact

        public EdgeContact(Point contactPoint, int segmentIndex, ZKTerrainAnalyzer.EdgeSegment edgeSegment) {
            this.contactPoint = contactPoint;
            this.segmentIndex = segmentIndex;
            this.edgeSegment = edgeSegment;
        }
    }
    Contour(){
        this.points = new ArrayList<Point>();
        this.chokepointContacts = new ArrayList<>();
        this.edgeContacts = new ArrayList<>();
    }
    Contour(ArrayList<Point>p){
        this.points = p;
        this.chokepointContacts = new ArrayList<>();
        this.edgeContacts = new ArrayList<>();
    }
    Contour(Point[] pts){
        this();
        for(Point p:pts){
            this.points.add(p);
        }
    }
    void addPoint(Point p){
        this.points.add(p);
    }
    void addPoint(int x, int y){
        this.points.add(new Point(x,y));
    }
    ArrayList<Point> getPoints(){
        return this.points;
    }

    int[] getBounds(){
        if(points.size() == 0){
            return null;
        }
        int minX=Integer.MAX_VALUE,minY=Integer.MAX_VALUE,maxX=0, maxY=0;
        for(Point p: points){
            minX = Math.min(p.x,minX);
            maxX = Math.max(p.x,maxX);
            minY = Math.min(p.y,minY);
            maxY = Math.max(p.y,maxY);
        }
        return new int[]{minX, maxX,minY, maxY};
    }

    Point[] getPointsArray(){
        Point[] a = new Point[this.points.size()];
        return this.points.toArray(a);
    }

    Point getPoint(int p){
        return this.points.get(p);
    }
    int length(){
        return this.points.size();
    }

    // Calculate polygon area using shoelace formula
    public double getArea() {
        if (points.size() < 3) return 0.0;

        double area = 0.0;
        int n = points.size();

        for (int i = 0; i < n; i++) {
            Point current = points.get(i);
            Point next = points.get((i + 1) % n);
            area += (current.x * next.y - next.x * current.y);
        }

        return Math.abs(area) / 2.0;
    }

    // Add a chokepoint contact to this contour
    public void addChokepointContact(Node chokepoint, Point contactPoint, int segmentIndex) {
        chokepointContacts.add(new ChokepointContact(chokepoint, contactPoint, segmentIndex));
    }

    // Get all chokepoints that touch this contour
    public ArrayList<ChokepointContact> getChokepointContacts() {
        return chokepointContacts;
    }

    // Find chokepoint contacts sorted by their position along the contour
    public ArrayList<ChokepointContact> getOrderedChokepointContacts() {
        ArrayList<ChokepointContact> ordered = new ArrayList<>(chokepointContacts);
        ordered.sort((a, b) -> Integer.compare(a.segmentIndex, b.segmentIndex));
        return ordered;
    }

    // Get the segment points for a given segment index
    public Point[] getSegmentPoints(int segmentIndex) {
        if (segmentIndex < 0 || segmentIndex >= getSegmentCount()) {
            return null;
        }
        Point startPoint = points.get(segmentIndex);
        Point endPoint = points.get((segmentIndex + 1) % points.size());
        return new Point[]{startPoint, endPoint};
    }

    // Get the total number of segments in this contour
    public int getSegmentCount() {
        return points.size(); // Each point connects to the next, including wrap-around
    }

    // Check if this contour is closed by following segment connectivity
    public boolean isClosedByConnectivity() {
        if (points.size() < 3) {
            return false; // Need at least 3 points for a closed polygon
        }

        // For a closed contour, every point should appear exactly twice in the segment graph:
        // once as a start point and once as an end point
        java.util.HashMap<Point, Integer> pointCount = new java.util.HashMap<>();

        for (int i = 0; i < points.size(); i++) {
            Point startPoint = points.get(i);
            Point endPoint = points.get((i + 1) % points.size());

            // Count each point's appearances
            pointCount.put(startPoint, pointCount.getOrDefault(startPoint, 0) + 1);
            pointCount.put(endPoint, pointCount.getOrDefault(endPoint, 0) + 1);
        }

        // In a closed contour, each point should appear exactly twice (start + end)
        for (Integer count : pointCount.values()) {
            if (count != 2) {
                return false;
            }
        }

        return true;
    }
}
