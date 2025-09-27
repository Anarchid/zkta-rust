import java.util.ArrayList;
import java.util.HashSet;

public class Region {
    public Node seedNode;           // The region seed node
    public ArrayList<Point> polygon; // Boundary polygon vertices
    public HashSet<Node> chokepoints; // Connected chokepoints
    public int id;                  // Unique region identifier
    public boolean isClosed;        // Whether the boundary forms a closed polygon
    public boolean isEdgy;          // Whether this region touches map edges
    public ArrayList<ZKTerrainAnalyzer.MapEdgeContact> edgeContacts; // Map edge contacts for this region
    public ArrayList<ArrayList<Point>> partialSegments; // Disconnected boundary segments when boundary construction fails

    public Region(Node seedNode, int id) {
        this.seedNode = seedNode;
        this.id = id;
        this.polygon = new ArrayList<>();
        this.chokepoints = new HashSet<>();
        this.isClosed = false;      // Default to unclosed until proven otherwise
        this.isEdgy = false;        // Default to non-edgy until proven otherwise
        this.edgeContacts = new ArrayList<>();
        this.partialSegments = new ArrayList<>();
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

    public void addEdgeContact(ZKTerrainAnalyzer.MapEdgeContact contact) {
        this.edgeContacts.add(contact);
        this.isEdgy = true;
    }

    public boolean hasEdgeContacts() {
        return !edgeContacts.isEmpty();
    }

    public ArrayList<ZKTerrainAnalyzer.MapEdgeContact> getContactsForEdge(ZKTerrainAnalyzer.MapEdge edge) {
        ArrayList<ZKTerrainAnalyzer.MapEdgeContact> contacts = new ArrayList<>();
        for (ZKTerrainAnalyzer.MapEdgeContact contact : edgeContacts) {
            if (contact.edge == edge) {
                contacts.add(contact);
            }
        }
        return contacts;
    }
}