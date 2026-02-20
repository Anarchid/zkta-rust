import java.util.ArrayList;
import java.util.HashSet;

public class Region {
    public Node seedNode;
    public ArrayList<Point> polygon;
    public HashSet<Node> chokepoints;
    public int id;
    public boolean isClosed;

    public Region(Node seedNode, int id) {
        this.seedNode = seedNode;
        this.id = id;
        this.polygon = new ArrayList<>();
        this.chokepoints = new HashSet<>();
        this.isClosed = false;
    }

    public Point getCenter() {
        return seedNode.position;
    }

    public double getRadius() {
        return seedNode.radius;
    }

    public void setPolygon(ArrayList<Point> polygon) {
        this.polygon = new ArrayList<>(polygon);
    }

    public void addChokepoint(Node chokepoint) {
        this.chokepoints.add(chokepoint);
    }
}
