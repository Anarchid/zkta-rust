import java.util.HashSet;

public class Node {
    public HashSet<Node> neighbours;
    public Point position;
    public double radius; // inscribed circle radius (distance to nearest obstacle)
    public boolean visited; // for BFS traversal
    public Node parent; // traversal parent

    public Node(){
        this.neighbours = new HashSet<>();
        this.position = new Point();
        this.radius = 0.0;
        this.visited = false;
        this.parent = null;
    }
    public Node(Point p){
        this.neighbours = new HashSet<>();
        this.position = p;
        this.radius = 0.0;
        this.visited = false;
        this.parent = null;
    }
    public Node(int x, int y){
        this.neighbours = new HashSet<>();
        this.position = new Point(x,y);
        this.radius = 0.0;
        this.visited = false;
        this.parent = null;
    }
    public void addNeighbour(Node n){
        this.neighbours.add(n);
    }

    // Check if this node is a local maximum (radius > all neighbors)
    public boolean isLocalMaximum() {
        for (Node neighbor : neighbours) {
            if (neighbor.radius >= this.radius) {
                return false;
            }
        }
        return true;
    }

    // Check if this node is a local minimum (radius < all neighbors)
    public boolean isLocalMinimum() {
        for (Node neighbor : neighbours) {
            if (neighbor.radius <= this.radius) {
                return false;
            }
        }
        return true;
    }
}
