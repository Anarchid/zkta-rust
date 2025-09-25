import org.rogach.jopenvoronoi.Edge;
import org.rogach.jopenvoronoi.HalfEdgeDiagram;
import org.rogach.jopenvoronoi.Vertex;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Map;

public class ZKTAWindow {

        static class SortByX implements Comparator<Point>
        {
            public int compare(Point a, Point b)
            {
                return (int)Math.signum(a.x - b.x);
            }
        }

        public static void fillHole(Contour hole, BufferedImage i){
            Graphics g =i.getGraphics();
            g.setColor(Color.MAGENTA);

            int[] bounds = hole.getBounds();
            boolean inside = false;
            boolean previousClear = false;

            int minX = bounds[0];
            int maxX = bounds[1];
            int minY = bounds[2];
            int maxY = bounds[3];

            /*
            g.drawLine(minX,minY,maxX,minY); // top
            g.drawLine(minX,maxY,maxX,maxY); // bottom

            g.drawLine(minX,minY,minX,maxY); // left
            g.drawLine(maxX,minY,maxX,maxY); // right
            */

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
                g.setColor(Color.MAGENTA);

                if(intersects.length > 1) {
                    inside = false;
                    for (int n = 0; n < intersects.length-1; n += 1) {
                        inside = !inside;

                        int min = intersects[n];
                        int max = intersects[n+1];

                        if (max <= min+1){
                            inside = false;
                        }

                        if(inside){
                            for(int x = min;x<max;x++){
                                g.drawLine(x,y,x,y);
                            }
                        }
                    }
                }
            }
        }

        public static void main(String[] args) {

            BufferedImage img;
            try {
                img = ImageIO.read(new File("trojan.png"));
            } catch (IOException e) {
                System.out.println("Terrain analyser has resigned and is now spectating");
                return;
            }

            ZKTerrainAnalyzer zkta = new ZKTerrainAnalyzer(img);
            ArrayList<Contour> contours = zkta.getContours();
            ArrayList<Contour> simple = zkta.getSimplifiedContours();

            Graphics bg = img.getGraphics();

            bg.setColor(Color.BLACK);
            bg.fillRect ( 0, 0, img.getWidth(), img.getHeight() );


            int[][] lm = zkta.getLabelMap();

            for(int x=0; x < zkta.getWidth(); x++ ){
                for(int y=0; y < zkta.getHeight(); y++ ){
                    if(lm[x][y] != 0){
                        bg.setColor(Color.DARK_GRAY);
                        bg.drawLine(x,y,x,y);
                    }
                }
            }

            bg.setColor(Color.GREEN);
            int fullSizePoints = 0;

            for(int c = 0; c < contours.size();c++){
                Contour contour = contours.get(c);
                for(int p = 0; p < contour.length();p++){
                    fullSizePoints++;
                    int n = p + 1 < contour.length()? p+1 : 0;
                    Point cp = contour.getPoint(p);
                    Point np = contour.getPoint(n);
                    bg.drawLine(cp.x,cp.y,np.x,np.y);
                }
            }



            int simplifiedPoints = 0;

            HalfEdgeDiagram g = zkta.getVoronoid().get_graph_reference();

            int edges = 0;

            // Draw edges from the FINAL region graph instead of original Voronoi
            bg.setColor(Color.LIGHT_GRAY);
            RegionGraph finalGraph = zkta.getRegions();
            for (Node node : finalGraph.nodes) {
                for (Node neighbor : node.neighbours) {
                    // Only draw each edge once
                    if (node.position.x < neighbor.position.x ||
                        (node.position.x == neighbor.position.x && node.position.y < neighbor.position.y)) {
                        bg.drawLine(node.position.x, node.position.y, neighbor.position.x, neighbor.position.y);
                        edges++;
                    }
                }
            }
            RegionGraph r = zkta.getRegions();

            // Draw regular nodes in orange
            bg.setColor(Color.ORANGE);
            for(Node n:r.nodes){
                if(!r.regionNodes.contains(n)){
                    bg.fillOval(n.position.x - 2, n.position.y - 2, 4, 4);
                }
            }

            // Draw region seed nodes in turquoise with numbers
            bg.setColor(new Color(64, 224, 208)); // Turquoise color
            int regionNum = 0;
            for(Node n:r.regionNodes){
                bg.fillOval(n.position.x - 4, n.position.y - 4, 8, 8);
                // Draw region number
                bg.setColor(Color.WHITE);
                bg.drawString("R" + regionNum, n.position.x + 6, n.position.y - 6);
                bg.setColor(new Color(64, 224, 208)); // Reset to turquoise
                regionNum++;
            }

            // Draw chokepoint nodes in red with numbers
            bg.setColor(Color.RED);
            int chokepointNum = 0;
            for(Node n:r.chokepointNodes){
                bg.fillOval(n.position.x - 4, n.position.y - 4, 8, 8);
                // Draw chokepoint number
                bg.setColor(Color.WHITE);
                bg.drawString("C" + chokepointNum, n.position.x + 6, n.position.y - 6);
                bg.setColor(Color.RED); // Reset to red
                chokepointNum++;
            }

            // Draw inscribed circles for regions with low-opacity teal
            Graphics2D g2d = (Graphics2D) bg;
            g2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.2f)); // 20% opacity
            g2d.setColor(new Color(64, 224, 208)); // Turquoise
            for(Node n:r.regionNodes){
                int radius = (int)n.radius;
                g2d.drawOval(n.position.x - radius, n.position.y - radius, 2 * radius, 2 * radius);
            }

            // Draw inscribed circles for chokepoints with low-opacity red
            g2d.setColor(Color.RED);
            for(Node n:r.chokepointNodes){
                int radius = (int)n.radius;
                g2d.drawOval(n.position.x - radius, n.position.y - radius, 2 * radius, 2 * radius);
            }

            // Reset to full opacity for subsequent drawing
            g2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 1.0f));

            // Draw chokepoint segments - red lines between closest obstacles
            bg.setColor(Color.RED);
            int segmentsDrawn = 0;
            int chokepointIndex = 0;
            for(Node chokepoint : r.chokepointNodes){
                Point[] segments = zkta.getChokepointSegments(chokepoint);
                if (segments != null && segments.length == 2) {
                    // Only draw if the two points are actually different
                    if (!segments[0].equals(segments[1])) {
                        bg.drawLine(segments[0].x, segments[0].y, segments[1].x, segments[1].y);
                        segmentsDrawn++;
                    } else {
                        System.out.println("DEBUG: C" + chokepointIndex + " @" + chokepoint.position +
                                         " (r=" + String.format("%.1f", chokepoint.radius) +
                                         ") has identical crossing points: " + segments[0]);
                    }
                } else {
                    System.out.println("DEBUG: C" + chokepointIndex + " @" + chokepoint.position +
                                     " (r=" + String.format("%.1f", chokepoint.radius) +
                                     ") has no segments (null or wrong length)");
                }
                chokepointIndex++;
            }
            System.out.println("Drew " + segmentsDrawn + " chokepoint segments out of " + r.chokepointNodes.size() + " chokepoints");

            // Draw traced region boundaries (nudged toward centroid for visibility)
            int boundariesDrawn = 0;
            java.util.Random colorRandom = new java.util.Random(42); // Fixed seed for consistent colors

            for (Region region : zkta.getRegionList()) {
                if (region.polygon != null && region.polygon.size() >= 3) {
                    // Generate a random bright color for this region
                    float hue = colorRandom.nextFloat();
                    float saturation = 0.7f + colorRandom.nextFloat() * 0.3f; // 70-100% saturation
                    float brightness = 0.8f + colorRandom.nextFloat() * 0.2f; // 80-100% brightness
                    Color regionColor = Color.getHSBColor(hue, saturation, brightness);
                    bg.setColor(regionColor);

                    Point centroid = region.getCenter();
                    int nudgePixels = 3;

                    for (int i = 0; i < region.polygon.size(); i++) {
                        Point current = region.polygon.get(i);
                        Point next = region.polygon.get((i + 1) % region.polygon.size());

                        // Nudge current point toward centroid
                        int dx1 = centroid.x - current.x;
                        int dy1 = centroid.y - current.y;
                        double dist1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
                        if (dist1 > 0) {
                            dx1 = (int) (nudgePixels * dx1 / dist1);
                            dy1 = (int) (nudgePixels * dy1 / dist1);
                        }
                        Point nudgedCurrent = new Point(current.x + dx1, current.y + dy1);

                        // Nudge next point toward centroid
                        int dx2 = centroid.x - next.x;
                        int dy2 = centroid.y - next.y;
                        double dist2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);
                        if (dist2 > 0) {
                            dx2 = (int) (nudgePixels * dx2 / dist2);
                            dy2 = (int) (nudgePixels * dy2 / dist2);
                        }
                        Point nudgedNext = new Point(next.x + dx2, next.y + dy2);

                        bg.drawLine(nudgedCurrent.x, nudgedCurrent.y, nudgedNext.x, nudgedNext.y);
                    }
                    boundariesDrawn++;
                }
            }
            System.out.println("Drew " + boundariesDrawn + " region boundaries");

            bg.setColor(Color.YELLOW);
            for(int i = 0; i < simple.size();i++){
                Contour contour = simple.get(i);
                Point pp = contour.getPoint(0);
                bg.drawString(""+i,pp.x+10,pp.y+10);
                for(int p = 0; p < contour.length();p++){
                    simplifiedPoints++;
                    int n = p + 1 < contour.length()? p+1 : 0;
                    Point cp = contour.getPoint(p);
                    Point np = contour.getPoint(n);
                    bg.drawLine(cp.x,cp.y,np.x,np.y);
                }
            }

            bg.setColor(Color.ORANGE);
            for (LabeledContour contour:zkta.holes) {
                for(int p = 0; p < contour.length();p++){
                    int n = p + 1 < contour.length()? p+1 : 0;
                    Point cp = contour.getPoint(p);
                    Point np = contour.getPoint(n);
                    bg.drawLine(cp.x,cp.y,np.x,np.y);
                }
            }

            if(zkta.errorEdge != null){
                bg.setColor(Color.RED);
                org.rogach.jopenvoronoi.Point p1 = zkta.errorEdge[0].position;
                org.rogach.jopenvoronoi.Point p2 = zkta.errorEdge[1].position;
                int[] v1 = zkta.coordsFromVoronoi(p1);
                int[] v2 = zkta.coordsFromVoronoi(p2);
                bg.drawLine(v1[0],v1[1],v2[0],v2[1]);
                System.out.println("Voronoi errored on edge {<"+p1+">,<"+p2+">}");
            }

            System.out.println("Total contour points: "+fullSizePoints);
            System.out.println("Simplified points: "+simplifiedPoints);
            System.out.println("Voronoi edges: "+edges);



            File outputfile = new File("result.png");
            try {
                ImageIO.write(img, "png", outputfile);
            } catch (IOException e) {
                e.printStackTrace();
            }


            JFrame frame = new JFrame();
            ImageIcon icon = new ImageIcon(img);
            JLabel label = new JLabel(icon);
            frame.add(label);
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.pack();
            frame.setVisible(true);
    }
}
