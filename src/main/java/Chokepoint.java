import java.util.HashSet;

public class Chokepoint {
    public Node node;               // The chokepoint node
    public Point[] crossingPoints;  // Points where this chokepoint crosses obstacles
    public Integer[] obstacleIds;   // Obstacle IDs corresponding to crossing points
    public HashSet<Region> regions; // Connected regions
    public int id;                  // Unique chokepoint identifier

    public Chokepoint(Node node, int id) {
        this.node = node;
        this.id = id;
        this.crossingPoints = null;
        this.obstacleIds = null;
        this.regions = new HashSet<>();
    }

    public Point getCenter() {
        return node.position;
    }

    public double getRadius() {
        return node.radius;
    }

    public void setCrossingPoints(Point[] crossingPoints) {
        this.crossingPoints = crossingPoints;
    }

    public void setCrossingData(Point[] crossingPoints, Integer[] obstacleIds) {
        this.crossingPoints = crossingPoints;
        this.obstacleIds = obstacleIds;
    }

    public void addRegion(Region region) {
        this.regions.add(region);
    }
}