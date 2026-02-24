use std::collections::HashMap;

use boostvoronoi::prelude::*;

use crate::contour::Contour;
use crate::geometry::{self, Point};

/// Build a Voronoi diagram from simplified contours and extract medial axis edges.
///
/// Returns a list of edges as (Point, Point) pairs in pixel coordinates.
pub fn build_voronoi(
    contours: &[Contour],
    width: i32,
    height: i32,
) -> Vec<(Point, Point)> {
    let start = std::time::Instant::now();

    // Collect all unique points with deduplication
    let mut point_map: HashMap<(i32, i32), usize> = HashMap::new();
    let mut points: Vec<[i32; 2]> = Vec::new();
    let mut segments: Vec<[i32; 4]> = Vec::new();

    // Corner points
    let corners = [
        (1i32, 1i32),
        (width - 1, 1),
        (width - 1, height - 1),
        (1, height - 1),
    ];

    let snap_tolerance = 5;

    // Helper: get or create deduplicated point index
    let mut get_point_index = |x: i32, y: i32| -> usize {
        let px = geometry::snap(geometry::snap(x, width, snap_tolerance), 1, snap_tolerance);
        let py = geometry::snap(geometry::snap(y, height, snap_tolerance), 1, snap_tolerance);
        let key = (px, py);
        if let Some(&idx) = point_map.get(&key) {
            idx
        } else {
            let idx = points.len();
            points.push([px, py]);
            point_map.insert(key, idx);
            idx
        }
    };

    // Track per-contour vertex indices for segment construction
    let mut contour_vertex_indices: Vec<Vec<usize>> = Vec::new();

    for contour in contours {
        let mut indices = Vec::new();
        // Skip the last point if it equals the first (closed contour)
        let n = if contour.points.len() > 1
            && contour.points[0] == contour.points[contour.points.len() - 1]
        {
            contour.points.len() - 1
        } else {
            contour.points.len()
        };
        for j in 0..n {
            let p = contour.points[j];
            let idx = get_point_index(p.x, p.y);
            indices.push(idx);
        }
        contour_vertex_indices.push(indices);
    }

    // Add corner points
    let mut corner_indices = [0usize; 4];
    for (i, &(cx, cy)) in corners.iter().enumerate() {
        corner_indices[i] = get_point_index(cx, cy);
    }

    // Build border point arrays (sorted along each edge)
    let border_info = build_borders(&points, &corner_indices, width, height, snap_tolerance);

    // Build segments for contour edges (skip those that lie on borders)
    for indices in &contour_vertex_indices {
        let n = indices.len();
        for j in 0..n {
            let k = (j + 1) % n;
            let idx_j = indices[j];
            let idx_k = indices[k];
            if idx_j == idx_k {
                continue; // Degenerate segment
            }

            // Skip segments that lie on a border
            if is_border_segment(idx_j, idx_k, &border_info) {
                continue;
            }

            let pj = points[idx_j];
            let pk = points[idx_k];
            // Skip zero-length segments (same coordinates despite different indices)
            if pj[0] == pk[0] && pj[1] == pk[1] {
                continue;
            }
            segments.push([pj[0], pj[1], pk[0], pk[1]]);
        }
    }

    // Deduplicate segments (both orderings)
    let mut seen_segments: std::collections::HashSet<(i32, i32, i32, i32)> =
        std::collections::HashSet::new();
    segments.retain(|s| {
        let fwd = (s[0], s[1], s[2], s[3]);
        let rev = (s[2], s[3], s[0], s[1]);
        if seen_segments.contains(&fwd) || seen_segments.contains(&rev) {
            false
        } else {
            seen_segments.insert(fwd);
            true
        }
    });

    // Remove points that lie on segments (boostvoronoi constraint)
    let point_on_segment = |px: i32, py: i32, s: &[i32; 4]| -> bool {
        let (x1, y1, x2, y2) = (s[0], s[1], s[2], s[3]);
        if (px, py) == (x1, y1) || (px, py) == (x2, y2) {
            return false; // endpoints are OK
        }
        let cross = (py as i64 - y1 as i64) * (x2 as i64 - x1 as i64)
            - (px as i64 - x1 as i64) * (y2 as i64 - y1 as i64);
        if cross != 0 {
            return false;
        }
        let min_x = x1.min(x2);
        let max_x = x1.max(x2);
        let min_y = y1.min(y2);
        let max_y = y1.max(y2);
        px >= min_x && px <= max_x && py >= min_y && py <= max_y
    };

    // Find which points are segment endpoints
    let mut is_segment_endpoint = vec![false; points.len()];
    for s in &segments {
        for (i, p) in points.iter().enumerate() {
            if (p[0] == s[0] && p[1] == s[1]) || (p[0] == s[2] && p[1] == s[3]) {
                is_segment_endpoint[i] = true;
            }
        }
    }

    // Remove interior points that lie on any segment
    let mut points_to_remove: Vec<bool> = vec![false; points.len()];
    for (i, p) in points.iter().enumerate() {
        if is_segment_endpoint[i] {
            continue;
        }
        for s in &segments {
            if point_on_segment(p[0], p[1], s) {
                points_to_remove[i] = true;
                break;
            }
        }
    }
    let removed_points = points_to_remove.iter().filter(|&&r| r).count();
    if removed_points > 0 {
        eprintln!("Removed {} points lying on segments", removed_points);
        // Build new point list, skipping removed points
        let mut new_points: Vec<[i32; 2]> = Vec::new();
        for (i, p) in points.iter().enumerate() {
            if !points_to_remove[i] {
                new_points.push(*p);
            }
        }
        points = new_points;
    }

    eprintln!(
        "Voronoi input: {} points, {} segments",
        points.len(),
        segments.len()
    );

    // Build Voronoi diagram (catch panics from degenerate geometry)
    let points_clone = points.clone();
    let segments_clone = segments.clone();
    let build_result = std::panic::catch_unwind(|| {
        Builder::<i32>::default()
            .with_vertices(points_clone.iter())
            .and_then(|b| b.with_segments(segments_clone.iter()))
            .and_then(|b| b.build())
    });

    let diagram = match build_result {
        Ok(Ok(d)) => d,
        Ok(Err(e)) => {
            eprintln!("Voronoi construction failed: {:?}", e);
            return Vec::new();
        }
        Err(_) => {
            eprintln!("Voronoi construction panicked (degenerate geometry)");
            return Vec::new();
        }
    };

    // Extract edges, applying medial axis filtering
    let mut edges = Vec::new();
    let num_points = points.len();

    for edge in diagram.edges().iter() {
        let edge_id = edge.id();

        // Only process each edge once (skip twin)
        if edge_id.usize() % 2 != 0 {
            continue;
        }

        // Skip non-primary edges (separators between same segment's two sides)
        if !edge.is_primary() {
            continue;
        }

        // Skip infinite edges
        if !diagram.edge_is_finite(edge_id).unwrap_or(false) {
            continue;
        }

        // Get vertex positions
        let v0_idx = match edge.vertex0() {
            Some(idx) => idx,
            None => continue,
        };
        let v1_idx = match diagram.edge_get_vertex1(edge_id) {
            Ok(Some(idx)) => idx,
            _ => continue,
        };

        let v0 = match diagram.vertex(v0_idx) {
            Ok(v) => v,
            Err(_) => continue,
        };
        let v1 = match diagram.vertex(v1_idx) {
            Ok(v) => v,
            Err(_) => continue,
        };

        let p0 = Point::new(v0.x().round() as i32, v0.y().round() as i32);
        let p1 = Point::new(v1.x().round() as i32, v1.y().round() as i32);

        // Medial axis filter: skip edges between nearly-parallel segments
        let cell_id = match edge.cell() {
            Ok(id) => id,
            Err(_) => continue,
        };
        let twin_id = match edge.twin() {
            Ok(id) => id,
            Err(_) => continue,
        };
        let twin_edge = match diagram.edge(twin_id) {
            Ok(e) => e,
            Err(_) => continue,
        };
        let twin_cell_id = match twin_edge.cell() {
            Ok(id) => id,
            Err(_) => continue,
        };

        let cell = match diagram.cell(cell_id) {
            Ok(c) => c,
            Err(_) => continue,
        };
        let twin_cell = match diagram.cell(twin_cell_id) {
            Ok(c) => c,
            Err(_) => continue,
        };

        edges.push((p0, p1));
    }

    eprintln!(
        "Generated Voronoi with {} medial axis edges in {:?}",
        edges.len(),
        start.elapsed()
    );
    edges
}

/// Compute normalized dot product between two segment direction vectors.
fn segment_dot_product(s1: &[i32; 4], s2: &[i32; 4]) -> f64 {
    let dx1 = (s1[2] - s1[0]) as f64;
    let dy1 = (s1[3] - s1[1]) as f64;
    let dx2 = (s2[2] - s2[0]) as f64;
    let dy2 = (s2[3] - s2[1]) as f64;
    let len1 = (dx1 * dx1 + dy1 * dy1).sqrt();
    let len2 = (dx2 * dx2 + dy2 * dy2).sqrt();
    if len1 == 0.0 || len2 == 0.0 {
        return 0.0;
    }
    ((dx1 * dx2 + dy1 * dy2) / (len1 * len2)).abs()
}

/// Border info: sorted point indices along each of the 4 borders.
struct BorderInfo {
    borders: [Vec<usize>; 4], // N, S, E, W
}

fn build_borders(
    points: &[[i32; 2]],
    corner_indices: &[usize; 4],
    width: i32,
    height: i32,
    tolerance: i32,
) -> BorderInfo {
    // Find points near each border
    let mut borders = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];

    // North border (y ≈ 1): corners[0] to corners[1], sort by x
    // South border (y ≈ height-1): corners[2] to corners[3], sort by x
    // East border (x ≈ width-1): corners[1] to corners[2], sort by y
    // West border (x ≈ 1): corners[0] to corners[3], sort by y

    for (idx, p) in points.iter().enumerate() {
        if (p[1] - 1).abs() <= tolerance {
            borders[0].push(idx); // north
        }
        if (p[1] - (height - 1)).abs() <= tolerance {
            borders[1].push(idx); // south
        }
        if (p[0] - (width - 1)).abs() <= tolerance {
            borders[2].push(idx); // east
        }
        if (p[0] - 1).abs() <= tolerance {
            borders[3].push(idx); // west
        }
    }

    // Add corners explicitly if not already present
    for &ci in &[corner_indices[0], corner_indices[1]] {
        if !borders[0].contains(&ci) {
            borders[0].push(ci);
        }
    }
    for &ci in &[corner_indices[2], corner_indices[3]] {
        if !borders[1].contains(&ci) {
            borders[1].push(ci);
        }
    }
    for &ci in &[corner_indices[1], corner_indices[2]] {
        if !borders[2].contains(&ci) {
            borders[2].push(ci);
        }
    }
    for &ci in &[corner_indices[0], corner_indices[3]] {
        if !borders[3].contains(&ci) {
            borders[3].push(ci);
        }
    }

    // Sort: N and S by x, E and W by y
    borders[0].sort_by_key(|&i| points[i][0]);
    borders[1].sort_by_key(|&i| points[i][0]);
    borders[2].sort_by_key(|&i| points[i][1]);
    borders[3].sort_by_key(|&i| points[i][1]);

    // Deduplicate
    for b in &mut borders {
        b.dedup();
    }

    BorderInfo { borders }
}

fn is_border_segment(idx_j: usize, idx_k: usize, info: &BorderInfo) -> bool {
    for border in &info.borders {
        for n in 1..border.len() {
            if (border[n - 1] == idx_j && border[n] == idx_k)
                || (border[n - 1] == idx_k && border[n] == idx_j)
            {
                return true;
            }
        }
    }
    false
}

/// Get the sorted border point coordinates for adding to R-tree.
pub fn get_border_points(
    contours: &[Contour],
    width: i32,
    height: i32,
) -> Vec<Vec<Point>> {
    let snap_tolerance = 5;

    // Collect all unique points
    let mut all_points: Vec<Point> = Vec::new();
    for contour in contours {
        for p in &contour.points {
            let px = geometry::snap(geometry::snap(p.x, width, snap_tolerance), 1, snap_tolerance);
            let py = geometry::snap(geometry::snap(p.y, height, snap_tolerance), 1, snap_tolerance);
            all_points.push(Point::new(px, py));
        }
    }

    let corners = [
        Point::new(1, 1),
        Point::new(width - 1, 1),
        Point::new(width - 1, height - 1),
        Point::new(1, height - 1),
    ];

    let mut borders = vec![Vec::new(); 4];

    // Add points near each border
    for &p in &all_points {
        if (p.y - 1).abs() <= snap_tolerance {
            borders[0].push(p);
        }
        if (p.y - (height - 1)).abs() <= snap_tolerance {
            borders[1].push(p);
        }
        if (p.x - (width - 1)).abs() <= snap_tolerance {
            borders[2].push(p);
        }
        if (p.x - 1).abs() <= snap_tolerance {
            borders[3].push(p);
        }
    }

    // Add corners
    borders[0].push(corners[0]);
    borders[0].push(corners[1]);
    borders[1].push(corners[2]);
    borders[1].push(corners[3]);
    borders[2].push(corners[1]);
    borders[2].push(corners[2]);
    borders[3].push(corners[0]);
    borders[3].push(corners[3]);

    // Sort and dedup
    borders[0].sort_by_key(|p| p.x);
    borders[1].sort_by_key(|p| p.x);
    borders[2].sort_by_key(|p| p.y);
    borders[3].sort_by_key(|p| p.y);
    for b in &mut borders {
        b.dedup();
    }

    borders
}
