import org.rogach.jopenvoronoi.Edge;
import org.rogach.jopenvoronoi.HalfEdgeDiagram;
import org.rogach.jopenvoronoi.Vertex;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.plaf.synth.Region;
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

            for (Edge e : g.edges) {
                if (e.valid) {
                    int[] v1 = zkta.coordsFromVoronoi(e.source);
                    int[] v2 = zkta.coordsFromVoronoi(e.target);

                    if(zkta.pruned.contains(e)){
                        bg.setColor(Color.DARK_GRAY);
                    }else{
                        bg.setColor(Color.LIGHT_GRAY);
                        //bg.fillOval(v1[0] - 3, v1[1] - 3, 6, 6);
                        //bg.fillOval(v2[0] - 3, v2[1] - 3, 6, 6);
                    }
                    edges++;
                    bg.drawLine(v1[0],v1[1],v2[0],v2[1]);
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

            // Draw region seed nodes in turquoise
            bg.setColor(new Color(64, 224, 208)); // Turquoise color
            for(Node n:r.regionNodes){
                bg.fillOval(n.position.x - 4, n.position.y - 4, 8, 8);
            }

            // Draw chokepoint nodes in red
            bg.setColor(Color.RED);
            for(Node n:r.chokepointNodes){
                bg.fillOval(n.position.x - 4, n.position.y - 4, 8, 8);
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
