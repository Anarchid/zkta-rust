public class Point implements com.goebl.simplify.Point{
    public int x;
    public int y;

    Point(){
        this.x = 0;
        this.y = 0;
    }

    Point(int x, int y){
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "<"+this.x+","+this.y+">";
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof Point){
            Point p = (Point)obj;
            return p.x == x && p.y == y;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return 31 * x + y;
    }
}
