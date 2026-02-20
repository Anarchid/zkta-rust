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
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import com.fasterxml.jackson.databind.ObjectMapper;

public class ZKTAWindow {

        public static void main(String[] args) {

            boolean headless = false;
            for (String arg : args) {
                if (arg.equals("--headless") || arg.equals("--debug") || arg.equals("-d")) {
                    headless = true;
                    System.out.println("Running in headless mode (no GUI window)");
                    break;
                }
            }

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

            Graphics2D bg = (Graphics2D) img.getGraphics();

            bg.setColor(Color.BLACK);
            bg.fillRect(0, 0, img.getWidth(), img.getHeight());

            // Draw region boundaries FIRST (underneath everything)
            Random colorRandom = new Random(42);
            int boundariesDrawn = 0;
            for (Region region : zkta.getRegionList()) {
                if (region.polygon != null && region.polygon.size() >= 3) {
                    float hue = colorRandom.nextFloat();
                    float saturation = 0.7f + colorRandom.nextFloat() * 0.3f;
                    float brightness = 0.8f + colorRandom.nextFloat() * 0.2f;
                    Color regionColor = Color.getHSBColor(hue, saturation, brightness);

                    Point centroid = region.getCenter();
                    int nudgePixels = 3;

                    int[] xPoints = new int[region.polygon.size()];
                    int[] yPoints = new int[region.polygon.size()];

                    for (int i = 0; i < region.polygon.size(); i++) {
                        Point current = region.polygon.get(i);
                        int dx = centroid.x - current.x;
                        int dy = centroid.y - current.y;
                        double dist = Math.sqrt(dx * dx + dy * dy);
                        if (dist > 0) {
                            dx = (int) (nudgePixels * dx / dist);
                            dy = (int) (nudgePixels * dy / dist);
                        }
                        xPoints[i] = current.x + dx;
                        yPoints[i] = current.y + dy;
                    }

                    if (region.isClosed) {
                        Color fillColor = new Color(regionColor.getRed(), regionColor.getGreen(),
                                                   regionColor.getBlue(), 64);
                        bg.setColor(fillColor);
                        bg.fillPolygon(xPoints, yPoints, region.polygon.size());
                        bg.setColor(regionColor);
                        bg.setStroke(new BasicStroke(1.0f));
                        bg.drawPolygon(xPoints, yPoints, region.polygon.size());
                    }

                    boundariesDrawn++;
                }
            }
            System.out.println("Drew " + boundariesDrawn + " region boundaries");

            // Draw label map (obstacle pixels) - renders on top of region fills
            int[][] lm = zkta.getLabelMap();
            for (int y = 0; y < zkta.getHeight(); y++) {
                for (int x = 0; x < zkta.getWidth(); x++) {
                    if (lm[y][x] != 0) {
                        bg.setColor(Color.DARK_GRAY);
                        bg.drawLine(x, y, x, y);
                    }
                }
            }

            // Draw full contours in green
            bg.setColor(Color.GREEN);
            for (Contour contour : contours) {
                for (int p = 0; p < contour.length(); p++) {
                    int n = p + 1 < contour.length() ? p + 1 : 0;
                    Point cp = contour.getPoint(p);
                    Point np = contour.getPoint(n);
                    bg.drawLine(cp.x, cp.y, np.x, np.y);
                }
            }

            // Draw graph edges in light gray
            bg.setColor(Color.LIGHT_GRAY);
            RegionGraph r = zkta.getRegions();
            for (Node node : r.nodes) {
                for (Node neighbor : node.neighbours) {
                    if (node.position.x < neighbor.position.x ||
                        (node.position.x == neighbor.position.x && node.position.y < neighbor.position.y)) {
                        bg.drawLine(node.position.x, node.position.y, neighbor.position.x, neighbor.position.y);
                    }
                }
            }

            // Draw region seed nodes in turquoise with labels
            bg.setColor(new Color(64, 224, 208));
            int regionNum = 0;
            for (Node n : r.regionNodes) {
                bg.fillOval(n.position.x - 4, n.position.y - 4, 8, 8);
                bg.setColor(Color.WHITE);
                bg.drawString("R" + regionNum, n.position.x + 6, n.position.y - 6);
                bg.setColor(new Color(64, 224, 208));
                regionNum++;
            }

            // Draw chokepoint nodes in red with labels
            bg.setColor(Color.RED);
            int chokepointNum = 0;
            for (Node n : r.chokepointNodes) {
                bg.fillOval(n.position.x - 4, n.position.y - 4, 8, 8);
                bg.setColor(Color.WHITE);
                bg.drawString("C" + chokepointNum, n.position.x + 6, n.position.y - 6);
                bg.setColor(Color.RED);
                chokepointNum++;
            }

            // Draw inscribed circles with low opacity
            bg.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.2f));
            bg.setColor(new Color(64, 224, 208));
            for (Node n : r.regionNodes) {
                int rad = (int) n.radius;
                bg.drawOval(n.position.x - rad, n.position.y - rad, 2 * rad, 2 * rad);
            }
            bg.setColor(Color.RED);
            for (Node n : r.chokepointNodes) {
                int rad = (int) n.radius;
                bg.drawOval(n.position.x - rad, n.position.y - rad, 2 * rad, 2 * rad);
            }
            bg.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 1.0f));

            // Draw chokepoint gate lines in red
            bg.setColor(Color.RED);
            int segmentsDrawn = 0;
            for (Node chokepoint : r.chokepointNodes) {
                Point[] segments = zkta.getChokepointSegments(chokepoint);
                if (segments != null && segments.length == 2 && !segments[0].equals(segments[1])) {
                    bg.drawLine(segments[0].x, segments[0].y, segments[1].x, segments[1].y);
                    segmentsDrawn++;
                }
            }
            System.out.println("Drew " + segmentsDrawn + " chokepoint gates");

            // Draw simplified contours in yellow with labels
            bg.setColor(Color.YELLOW);
            for (int i = 0; i < simple.size(); i++) {
                Contour contour = simple.get(i);
                Point pp = contour.getPoint(0);
                bg.drawString("" + i, pp.x + 10, pp.y + 10);
                for (int p = 0; p < contour.length(); p++) {
                    int n = p + 1 < contour.length() ? p + 1 : 0;
                    Point cp = contour.getPoint(p);
                    Point np = contour.getPoint(n);
                    bg.drawLine(cp.x, cp.y, np.x, np.y);
                }
            }

            // Draw holes in orange
            bg.setColor(Color.ORANGE);
            for (LabeledContour contour : zkta.holes) {
                for (int p = 0; p < contour.length(); p++) {
                    int n = p + 1 < contour.length() ? p + 1 : 0;
                    Point cp = contour.getPoint(p);
                    Point np = contour.getPoint(n);
                    bg.drawLine(cp.x, cp.y, np.x, np.y);
                }
            }

            // Draw Voronoi error edge if any
            if (zkta.errorEdge != null) {
                bg.setColor(Color.RED);
                org.rogach.jopenvoronoi.Point p1 = zkta.errorEdge[0].position;
                org.rogach.jopenvoronoi.Point p2 = zkta.errorEdge[1].position;
                int[] v1 = zkta.coordsFromVoronoi(p1);
                int[] v2 = zkta.coordsFromVoronoi(p2);
                bg.drawLine(v1[0], v1[1], v2[0], v2[1]);
                System.out.println("Voronoi errored on edge {<" + p1 + ">,<" + p2 + ">}");
            }

            // Save result image
            File outputfile = new File("result.png");
            try {
                ImageIO.write(img, "png", outputfile);
            } catch (IOException e) {
                e.printStackTrace();
            }

            // Export JSON
            exportToJSON(zkta);

            if (!headless) {
                JFrame frame = new JFrame();
                ImageIcon icon = new ImageIcon(img);
                JLabel label = new JLabel(icon);
                frame.add(label);
                frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                frame.pack();
                frame.setVisible(true);
            } else {
                System.out.println("Analysis complete. Output saved to result.png and result.json");
            }
    }

    private static void exportToJSON(ZKTerrainAnalyzer zkta) {
        try {
            ObjectMapper mapper = new ObjectMapper();
            Map<String, Object> data = new HashMap<>();

            // Export regions
            ArrayList<Map<String, Object>> regions = new ArrayList<>();
            for (Region region : zkta.getRegionList()) {
                Map<String, Object> regionData = new HashMap<>();
                regionData.put("id", region.id);

                Map<String, Integer> center = new HashMap<>();
                center.put("x", region.getCenter().x);
                center.put("y", region.getCenter().y);
                regionData.put("center", center);

                regionData.put("radius", region.getRadius());
                regionData.put("isClosed", region.isClosed);

                ArrayList<Map<String, Integer>> polygon = new ArrayList<>();
                if (region.polygon != null) {
                    for (Point p : region.polygon) {
                        Map<String, Integer> point = new HashMap<>();
                        point.put("x", p.x);
                        point.put("y", p.y);
                        polygon.add(point);
                    }
                }
                regionData.put("polygon", polygon);

                // Export connected chokepoint IDs
                ArrayList<Integer> chokepointIds = new ArrayList<>();
                for (Chokepoint cp : zkta.getChokepointList()) {
                    if (region.chokepoints.contains(cp.node)) {
                        chokepointIds.add(cp.id);
                    }
                }
                regionData.put("chokepoints", chokepointIds);

                regions.add(regionData);
            }
            data.put("regions", regions);

            // Export chokepoints
            ArrayList<Map<String, Object>> chokepoints = new ArrayList<>();
            for (Chokepoint cp : zkta.getChokepointList()) {
                Map<String, Object> cpData = new HashMap<>();
                cpData.put("id", cp.id);

                Map<String, Integer> position = new HashMap<>();
                position.put("x", cp.node.position.x);
                position.put("y", cp.node.position.y);
                cpData.put("position", position);

                cpData.put("radius", cp.node.radius);

                ArrayList<Map<String, Integer>> crossingPoints = new ArrayList<>();
                if (cp.crossingPoints != null) {
                    for (Point p : cp.crossingPoints) {
                        Map<String, Integer> point = new HashMap<>();
                        point.put("x", p.x);
                        point.put("y", p.y);
                        crossingPoints.add(point);
                    }
                }
                cpData.put("crossingPoints", crossingPoints);

                if (cp.obstacleIds != null) {
                    ArrayList<Integer> ids = new ArrayList<>();
                    for (Integer id : cp.obstacleIds) ids.add(id);
                    cpData.put("obstacleIds", ids);
                }

                // Export connected region IDs
                ArrayList<Integer> regionIds = new ArrayList<>();
                for (Region reg : cp.regions) {
                    regionIds.add(reg.id);
                }
                cpData.put("regions", regionIds);

                chokepoints.add(cpData);
            }
            data.put("chokepoints", chokepoints);

            // Export obstacle contours
            ArrayList<Map<String, Object>> obstacleContours = new ArrayList<>();
            ArrayList<Contour> contours = zkta.getSimplifiedContours();
            for (int i = 0; i < contours.size(); i++) {
                Contour contour = contours.get(i);
                Map<String, Object> contourData = new HashMap<>();
                contourData.put("obstacleId", i + 1);

                ArrayList<Map<String, Integer>> points = new ArrayList<>();
                for (Point p : contour.getPoints()) {
                    Map<String, Integer> point = new HashMap<>();
                    point.put("x", p.x);
                    point.put("y", p.y);
                    points.add(point);
                }
                contourData.put("points", points);
                contourData.put("pointCount", points.size());

                obstacleContours.add(contourData);
            }
            data.put("obstacleContours", obstacleContours);

            mapper.writerWithDefaultPrettyPrinter().writeValue(new File("result.json"), data);
            System.out.println("Exported data to result.json");
        } catch (IOException e) {
            System.err.println("Failed to write JSON file: " + e.getMessage());
        }
    }
}
