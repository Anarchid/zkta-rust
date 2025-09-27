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
import java.util.Random;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;

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

            // Check for headless/debug mode
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

            // Draw map edge contact points in magenta
            bg.setColor(Color.MAGENTA);
            ArrayList<ZKTerrainAnalyzer.MapEdgeContact> mapContacts = zkta.getMapEdgeContacts();
            for (ZKTerrainAnalyzer.MapEdgeContact contact : mapContacts) {
                if (contact.visited) {
                    // Draw visited contacts with double radius (bright magenta)
                    bg.setColor(new Color(255, 0, 255)); // Bright magenta
                    bg.fillOval(contact.contactPoint.x - 6, contact.contactPoint.y - 6, 12, 12);
                } else {
                    // Draw unvisited contacts with normal radius (darker magenta)
                    bg.setColor(new Color(128, 0, 128)); // Darker magenta
                    bg.fillOval(contact.contactPoint.x - 3, contact.contactPoint.y - 3, 6, 6);
                }
            }

            // Draw edge segments in bright cyan
            Graphics2D g2d = (Graphics2D) bg;
            ArrayList<ZKTerrainAnalyzer.EdgeSegment> edgeSegments = zkta.getEdgeSegments();
            g2d.setColor(Color.CYAN);
            g2d.setStroke(new BasicStroke(2.0f));

            for (ZKTerrainAnalyzer.EdgeSegment segment : edgeSegments) {
                // Draw the full segment in cyan
                g2d.drawLine(segment.start.x, segment.start.y, segment.end.x, segment.end.y);

                // Draw perpendicular gray notches at the segment endpoints to show joints
                g2d.setColor(Color.GRAY);
                g2d.setStroke(new BasicStroke(1.0f));

                double dx = segment.end.x - segment.start.x;
                double dy = segment.end.y - segment.start.y;
                double length = Math.sqrt(dx * dx + dy * dy);

                if (length > 0) {
                    // Unit vector along the segment
                    double unitX = dx / length;
                    double unitY = dy / length;
                    // Perpendicular vector (rotate 90 degrees)
                    double perpX = -unitY;
                    double perpY = unitX;

                    int notchLength = 4;

                    // Draw notch at start point
                    int startNotchX1 = segment.start.x - (int)(notchLength * perpX / 2);
                    int startNotchY1 = segment.start.y - (int)(notchLength * perpY / 2);
                    int startNotchX2 = segment.start.x + (int)(notchLength * perpX / 2);
                    int startNotchY2 = segment.start.y + (int)(notchLength * perpY / 2);
                    g2d.drawLine(startNotchX1, startNotchY1, startNotchX2, startNotchY2);

                    // Draw notch at end point
                    int endNotchX1 = segment.end.x - (int)(notchLength * perpX / 2);
                    int endNotchY1 = segment.end.y - (int)(notchLength * perpY / 2);
                    int endNotchX2 = segment.end.x + (int)(notchLength * perpX / 2);
                    int endNotchY2 = segment.end.y + (int)(notchLength * perpY / 2);
                    g2d.drawLine(endNotchX1, endNotchY1, endNotchX2, endNotchY2);
                }

                // Reset to cyan and thick stroke for next segment
                g2d.setColor(Color.CYAN);
                g2d.setStroke(new BasicStroke(2.0f));
            }

            // Reset stroke
            g2d.setStroke(new BasicStroke(1.0f));

            // Debug: Draw incomplete/failed region boundaries

            ArrayList<Region> regions = zkta.getRegionList();
            for (Region region : regions) {
                // Skip closed regions - let the fill system handle those
                if (region.isClosed) {
                    continue;
                }

                // Generate a unique color for this region (using same approach as fill system)
                Random regionRandom = new Random(region.id); // Seed with region ID for consistency
                float hue = regionRandom.nextFloat();
                float saturation = 0.7f + regionRandom.nextFloat() * 0.3f; // 70-100% saturation
                float brightness = 0.8f + regionRandom.nextFloat() * 0.2f; // 80-100% brightness
                Color regionColor = Color.getHSBColor(hue, saturation, brightness);
                g2d.setColor(regionColor);

                if (region.partialSegments != null && !region.partialSegments.isEmpty()) {
                    // Draw partial boundary segments with dashed lines
                    float[] dashPattern = {8.0f, 4.0f}; // 8 pixel dash, 4 pixel gap
                    g2d.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_BUTT,
                                                BasicStroke.JOIN_MITER, 10.0f, dashPattern, 0.0f));

                    // Draw each disconnected segment separately
                    for (ArrayList<Point> segment : region.partialSegments) {
                        for (int i = 0; i < segment.size() - 1; i++) {
                            Point p1 = segment.get(i);
                            Point p2 = segment.get(i + 1);
                            g2d.drawLine(p1.x, p1.y, p2.x, p2.y);
                        }
                    }
                }
            }

            // Reset stroke to solid
            g2d.setStroke(new BasicStroke(1.0f));

            // Draw inscribed circles for regions with low-opacity teal
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

                    Point centroid = region.getCenter();
                    int nudgePixels = 3;

                    // Create nudged polygon for drawing
                    int[] xPoints = new int[region.polygon.size()];
                    int[] yPoints = new int[region.polygon.size()];

                    for (int i = 0; i < region.polygon.size(); i++) {
                        Point current = region.polygon.get(i);

                        // Nudge point toward centroid
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
                        // Draw filled polygon with 25% opacity for closed regions
                        Color fillColor = new Color(regionColor.getRed(), regionColor.getGreen(),
                                                   regionColor.getBlue(), 64); // 64/255 = ~25% opacity
                        g2d.setColor(fillColor);
                        g2d.fillPolygon(xPoints, yPoints, region.polygon.size());

                        // Draw solid boundary
                        g2d.setColor(regionColor);
                        g2d.setStroke(new BasicStroke(1.0f)); // Solid line
                        g2d.drawPolygon(xPoints, yPoints, region.polygon.size());
                    } else {
                        // Draw dashed boundary for open regions
                        g2d.setColor(regionColor);
                        float[] openRegionDashPattern = {5.0f, 3.0f}; // 5 pixel dash, 3 pixel gap
                        g2d.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_BUTT,
                                                     BasicStroke.JOIN_MITER, 10.0f, openRegionDashPattern, 0.0f));

                        // Draw the polyline segments
                        for (int i = 0; i < region.polygon.size() - 1; i++) {
                            g2d.drawLine(xPoints[i], yPoints[i], xPoints[i + 1], yPoints[i + 1]);
                        }
                        // Don't automatically close open polygons
                    }

                    boundariesDrawn++;
                }
            }

            // Reset stroke to solid for subsequent drawing
            g2d.setStroke(new BasicStroke(1.0f));

            System.out.println("Drew " + boundariesDrawn + " region boundaries");

            // Draw incomplete region boundaries for debugging
            int incompleteCount = 0;
            for (Region region : zkta.getRegionList()) {
                if (!region.isClosed && region.polygon != null && region.polygon.size() >= 2) {
                    // Alternate colors: odd regions = dashed red, even regions = dashed turquoise
                    Color debugColor = (region.id % 2 == 1) ? Color.RED : Color.CYAN;
                    g2d.setColor(debugColor);

                    // Set up dashed line
                    float[] incompleteBoundaryDashPattern = {6.0f, 6.0f}; // 6 pixel dash, 6 pixel gap
                    g2d.setStroke(new BasicStroke(2.0f, BasicStroke.CAP_BUTT,
                                               BasicStroke.JOIN_MITER, 10.0f, incompleteBoundaryDashPattern, 0.0f));

                    // Draw the incomplete boundary segments
                    if (region.partialSegments != null && !region.partialSegments.isEmpty()) {
                        for (ArrayList<Point> segment : region.partialSegments) {
                            for (int i = 0; i < segment.size() - 1; i++) {
                                Point p1 = segment.get(i);
                                Point p2 = segment.get(i + 1);
                                g2d.drawLine(p1.x, p1.y, p2.x, p2.y);
                            }
                        }
                    }

                    // Draw region ID at centroid
                    Point centroid = region.getCenter();
                    g2d.setColor(Color.WHITE);
                    g2d.fillRect(centroid.x - 10, centroid.y - 8, 20, 16);
                    g2d.setColor(debugColor);
                    g2d.setFont(new Font("Arial", Font.BOLD, 10));
                    g2d.drawString("R" + region.id, centroid.x - 8, centroid.y + 3);

                    incompleteCount++;
                }
            }

            // Reset stroke to solid
            g2d.setStroke(new BasicStroke(1.0f));

            if (incompleteCount > 0) {
                System.out.println("Drew " + incompleteCount + " incomplete region boundaries for debugging");
            }

            // Draw jagged segments for debugging
            ArrayList<ZKTerrainAnalyzer.JaggedSegment> jaggedSegments = zkta.getJaggedSegments();
            if (!jaggedSegments.isEmpty()) {
                // Set up double thick dashed red line
                g2d.setColor(Color.RED);
                float[] jaggedSegmentDashPattern = {8.0f, 4.0f}; // 8 pixel dash, 4 pixel gap
                g2d.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_BUTT,
                                           BasicStroke.JOIN_MITER, 10.0f, jaggedSegmentDashPattern, 0.0f));

                for (ZKTerrainAnalyzer.JaggedSegment segment : jaggedSegments) {
                    g2d.drawLine(segment.start.x, segment.start.y, segment.end.x, segment.end.y);

                    // Draw identifier label at midpoint of segment
                    int midX = (segment.start.x + segment.end.x) / 2;
                    int midY = (segment.start.y + segment.end.y) / 2;
                    g2d.setColor(Color.WHITE);
                    g2d.fillRect(midX - 15, midY - 8, 30, 16);
                    g2d.setColor(Color.RED);
                    g2d.drawString(segment.identifier, midX - 12, midY + 4);
                }

                // Reset stroke
                g2d.setStroke(new BasicStroke(1.0f));
                System.out.println("Drew " + jaggedSegments.size() + " jagged debug segments");
            }

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

            // Export JSON data
            exportToJSON(zkta);

            // Only show window if not in headless mode
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
            ArrayList<Region> regionList = zkta.getRegionList();
            for (Region region : regionList) {
                Map<String, Object> regionData = new HashMap<>();
                regionData.put("id", region.id);

                Map<String, Integer> center = new HashMap<>();
                center.put("x", region.getCenter().x);
                center.put("y", region.getCenter().y);
                regionData.put("center", center);

                regionData.put("radius", region.getRadius());
                regionData.put("isClosed", region.isClosed);

                // Export polygon points
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

                // Export connected chokepoint indices
                ArrayList<Integer> chokepointIndices = new ArrayList<>();
                RegionGraph regionsGraph = zkta.getRegions();
                int cpIndex = 0;
                for (Node cpNode : regionsGraph.chokepointNodes) {
                    if (region.chokepoints.contains(cpNode)) {
                        chokepointIndices.add(cpIndex);
                    }
                    cpIndex++;
                }
                regionData.put("chokepointNodes", chokepointIndices);

                regions.add(regionData);
            }
            data.put("regions", regions);

            // Export chokepoints
            ArrayList<Map<String, Object>> chokepoints = new ArrayList<>();
            RegionGraph regionsGraph = zkta.getRegions();
            for (Chokepoint cp : regionsGraph.chokepoints) {
                Map<String, Object> chokepointData = new HashMap<>();
                chokepointData.put("id", cp.id);

                Map<String, Integer> position = new HashMap<>();
                position.put("x", cp.node.position.x);
                position.put("y", cp.node.position.y);
                chokepointData.put("position", position);

                chokepointData.put("radius", cp.node.radius);

                // Export crossing points
                ArrayList<Map<String, Integer>> crossingPoints = new ArrayList<>();
                if (cp.crossingPoints != null) {
                    for (Point p : cp.crossingPoints) {
                        Map<String, Integer> point = new HashMap<>();
                        point.put("x", p.x);
                        point.put("y", p.y);
                        crossingPoints.add(point);
                    }
                }
                chokepointData.put("crossingPoints", crossingPoints);

                // Export obstacle IDs
                ArrayList<Integer> obstacleIds = new ArrayList<>();
                if (cp.obstacleIds != null) {
                    for (Integer obstacleId : cp.obstacleIds) {
                        obstacleIds.add(obstacleId);
                    }
                }
                chokepointData.put("obstacleIds", obstacleIds);

                // Export connected region indices
                ArrayList<Integer> regionIndices = new ArrayList<>();
                int regIndex = 0;
                for (Node regNode : regionsGraph.regionNodes) {
                    boolean connected = false;
                    for (Node neighbor : cp.node.neighbours) {
                        if (neighbor == regNode) {
                            connected = true;
                            break;
                        }
                    }
                    if (connected) {
                        regionIndices.add(regIndex);
                    }
                    regIndex++;
                }
                chokepointData.put("regionNodes", regionIndices);

                chokepoints.add(chokepointData);
            }
            data.put("chokepoints", chokepoints);

            // Export map edge contacts
            ArrayList<Map<String, Object>> mapEdgeContacts = new ArrayList<>();
            for (ZKTerrainAnalyzer.MapEdgeContact contact : zkta.getMapEdgeContacts()) {
                Map<String, Object> contactData = new HashMap<>();

                // Contact point
                Map<String, Integer> contactPoint = new HashMap<>();
                contactPoint.put("x", contact.contactPoint.x);
                contactPoint.put("y", contact.contactPoint.y);
                contactData.put("contactPoint", contactPoint);

                // Map edge
                contactData.put("edge", contact.edge.toString());

                // Type and related entity
                if (contact.isObstacle) {
                    contactData.put("type", "obstacle");
                    contactData.put("obstacleId", contact.obstacleId);
                } else if (contact.isChokepoint) {
                    contactData.put("type", "chokepoint");
                    Map<String, Integer> chokepointPosition = new HashMap<>();
                    chokepointPosition.put("x", contact.chokepointNode.position.x);
                    chokepointPosition.put("y", contact.chokepointNode.position.y);
                    contactData.put("chokepointPosition", chokepointPosition);
                    contactData.put("chokepointRadius", contact.chokepointNode.radius);
                }

                mapEdgeContacts.add(contactData);
            }
            data.put("mapEdgeContacts", mapEdgeContacts);

            // Export edge contacts from contours
            ArrayList<Map<String, Object>> contourEdgeContacts = new ArrayList<>();
            ArrayList<Contour> contours = zkta.getSimplifiedContours();
            for (int contourIndex = 0; contourIndex < contours.size(); contourIndex++) {
                Contour contour = contours.get(contourIndex);
                int obstacleId = 1000 + contourIndex; // Match the obstacle ID convention

                for (Contour.EdgeContact edgeContact : contour.edgeContacts) {
                    Map<String, Object> contactData = new HashMap<>();

                    // Contact point
                    Map<String, Integer> contactPoint = new HashMap<>();
                    contactPoint.put("x", edgeContact.contactPoint.x);
                    contactPoint.put("y", edgeContact.contactPoint.y);
                    contactData.put("contactPoint", contactPoint);

                    // Edge information from EdgeSegment
                    contactData.put("edge", edgeContact.edgeSegment.edge.toString());

                    // EdgeSegment details for pathfinding
                    Map<String, Integer> segmentStart = new HashMap<>();
                    segmentStart.put("x", edgeContact.edgeSegment.start.x);
                    segmentStart.put("y", edgeContact.edgeSegment.start.y);
                    contactData.put("edgeSegmentStart", segmentStart);

                    Map<String, Integer> segmentEnd = new HashMap<>();
                    segmentEnd.put("x", edgeContact.edgeSegment.end.x);
                    segmentEnd.put("y", edgeContact.edgeSegment.end.y);
                    contactData.put("edgeSegmentEnd", segmentEnd);

                    contactData.put("startPosition", edgeContact.edgeSegment.startPosition);
                    contactData.put("endPosition", edgeContact.edgeSegment.endPosition);

                    // Metadata
                    contactData.put("type", "obstacle");
                    contactData.put("obstacleId", obstacleId);
                    contactData.put("segmentIndex", edgeContact.segmentIndex);
                    contactData.put("source", "contour");

                    contourEdgeContacts.add(contactData);
                }
            }
            data.put("contourEdgeContacts", contourEdgeContacts);

            // Export complete obstacle contours for analysis
            ArrayList<Map<String, Object>> obstacleContours = new ArrayList<>();
            for (int contourIndex = 0; contourIndex < contours.size(); contourIndex++) {
                Contour contour = contours.get(contourIndex);
                int obstacleId = 1000 + contourIndex; // Match the obstacle ID convention

                Map<String, Object> contourData = new HashMap<>();
                contourData.put("obstacleId", obstacleId);

                // Export ALL contour points
                ArrayList<Map<String, Object>> contourPoints = new ArrayList<>();
                for (Point point : contour.getPoints()) {
                    Map<String, Object> pointData = new HashMap<>();
                    pointData.put("x", point.x);
                    pointData.put("y", point.y);
                    contourPoints.add(pointData);
                }
                contourData.put("points", contourPoints);

                // Include counts for debugging
                contourData.put("pointCount", contourPoints.size());
                contourData.put("edgeContactCount", contour.edgeContacts.size());
                contourData.put("chokepointContactCount", contour.chokepointContacts.size());

                obstacleContours.add(contourData);
            }
            data.put("obstacleContours", obstacleContours);

            // Write to file
            mapper.writerWithDefaultPrettyPrinter().writeValue(new File("result.json"), data);
            System.out.println("Exported data to result.json");
        } catch (IOException e) {
            System.err.println("Failed to write JSON file: " + e.getMessage());
        }
    }
}
