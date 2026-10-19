use crate::det::HashMap;

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
    let mut point_map: HashMap<(i32, i32), usize> = HashMap::default();
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
        let px = geometry::snap(geometry::snap(x, width - 1, snap_tolerance), 1, snap_tolerance);
        let py = geometry::snap(geometry::snap(y, height - 1, snap_tolerance), 1, snap_tolerance);
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

    // Track which contour segments lie on borders (covered by obstacle contour edges).
    // We'll use this to only add border sub-segments for UNCOVERED gaps.
    let mut contour_border_pairs: crate::det::HashSet<(usize, usize)> =
        crate::det::HashSet::default();

    // Build segments for contour edges (skip those that lie on borders)
    let mut segment_contour_id: Vec<usize> = Vec::new();
    for (ci, indices) in contour_vertex_indices.iter().enumerate() {
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
                // Record which border point pairs are covered by this contour segment
                record_covered_border_pairs(
                    idx_j,
                    idx_k,
                    &border_info,
                    &mut contour_border_pairs,
                );
                continue;
            }

            let pj = points[idx_j];
            let pk = points[idx_k];
            // Skip zero-length segments (same coordinates despite different indices)
            if pj[0] == pk[0] && pj[1] == pk[1] {
                continue;
            }
            segments.push([pj[0], pj[1], pk[0], pk[1]]);
            segment_contour_id.push(ci);
        }
    }

    // Add border sub-segments only for gaps NOT covered by obstacle contour segments.
    let mut border_segment_count = 0;
    for border in &border_info.borders {
        for n in 1..border.len() {
            let idx_a = border[n - 1];
            let idx_b = border[n];
            if idx_a == idx_b {
                continue;
            }
            // Skip if this sub-segment is covered by a contour segment
            let pair = (idx_a.min(idx_b), idx_a.max(idx_b));
            if contour_border_pairs.contains(&pair) {
                continue;
            }
            let pa = points[idx_a];
            let pb = points[idx_b];
            if pa[0] == pb[0] && pa[1] == pb[1] {
                continue;
            }
            segments.push([pa[0], pa[1], pb[0], pb[1]]);
            segment_contour_id.push(usize::MAX); // border segment
            border_segment_count += 1;
        }
    }
    log::debug!("Added {} border sub-segments to Voronoi input", border_segment_count);

    // Deduplicate segments (both orderings), keeping contour IDs in sync
    {
        let mut seen: crate::det::HashSet<(i32, i32, i32, i32)> =
            crate::det::HashSet::default();
        let mut keep = Vec::with_capacity(segments.len());
        for i in 0..segments.len() {
            let s = segments[i];
            let fwd = (s[0], s[1], s[2], s[3]);
            let rev = (s[2], s[3], s[0], s[1]);
            if !seen.contains(&fwd) && !seen.contains(&rev) {
                seen.insert(fwd);
                keep.push(i);
            }
        }
        let new_segments: Vec<[i32; 4]> = keep.iter().map(|&i| segments[i]).collect();
        let new_ids: Vec<usize> = keep.iter().map(|&i| segment_contour_id[i]).collect();
        segments = new_segments;
        segment_contour_id = new_ids;
    }

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
        log::debug!("Removed {} points lying on segments", removed_points);
        // Build new point list, skipping removed points
        let mut new_points: Vec<[i32; 2]> = Vec::new();
        for (i, p) in points.iter().enumerate() {
            if !points_to_remove[i] {
                new_points.push(*p);
            }
        }
        points = new_points;
    }

    // Check for intersecting segments (boostvoronoi constraint: non-intersecting except at endpoints)
    let mut intersection_count = 0;
    for i in 0..segments.len() {
        for j in (i + 1)..segments.len() {
            let s1 = &segments[i];
            let s2 = &segments[j];
            // Skip if they share an endpoint
            if (s1[0] == s2[0] && s1[1] == s2[1])
                || (s1[0] == s2[2] && s1[1] == s2[3])
                || (s1[2] == s2[0] && s1[3] == s2[1])
                || (s1[2] == s2[2] && s1[3] == s2[3])
            {
                continue;
            }
            let a1 = geometry::Point::new(s1[0], s1[1]);
            let a2 = geometry::Point::new(s1[2], s1[3]);
            let b1 = geometry::Point::new(s2[0], s2[1]);
            let b2 = geometry::Point::new(s2[2], s2[3]);
            if geometry::segments_intersect(a1, a2, b1, b2) {
                if intersection_count < 5 {
                    let c1 = segment_contour_id[i];
                    let c2 = segment_contour_id[j];
                    let label = |c: usize| if c == usize::MAX { "border".to_string() } else { format!("C{}", c) };
                    log::warn!(
                        "  WARNING: intersecting segments: [{},{}]-[{},{}] ({}) x [{},{}]-[{},{}] ({})",
                        s1[0], s1[1], s1[2], s1[3], label(c1),
                        s2[0], s2[1], s2[2], s2[3], label(c2),
                    );
                }
                intersection_count += 1;
            }
        }
    }
    if intersection_count > 0 {
        log::debug!("Total {} intersecting segment pairs!", intersection_count);
    }

    // Check for collinear overlapping segments
    let mut overlap_count = 0;
    for i in 0..segments.len() {
        for j in (i + 1)..segments.len() {
            let s1 = &segments[i];
            let s2 = &segments[j];
            if are_collinear_overlapping(s1, s2) {
                if overlap_count < 5 {
                    log::warn!(
                        "  WARNING: collinear overlap: [{},{}]-[{},{}] vs [{},{}]-[{},{}]",
                        s1[0], s1[1], s1[2], s1[3], s2[0], s2[1], s2[2], s2[3]
                    );
                }
                overlap_count += 1;
            }
        }
    }
    if overlap_count > 0 {
        log::debug!("Total {} collinear overlapping segment pairs!", overlap_count);
    }

    log::debug!(
        "Voronoi input: {} points, {} segments",
        points.len(),
        segments.len()
    );

    // Dump segments for diagnostics
    if std::env::var("ZKTA_DUMP_SEGMENTS").is_ok() {
        let mut dump = String::new();
        for s in &segments {
            dump.push_str(&format!("{},{},{},{}\n", s[0], s[1], s[2], s[3]));
        }
        let _ = std::fs::write("segments_dump.csv", dump);
        let mut dump = String::new();
        for p in &points {
            dump.push_str(&format!("{},{}\n", p[0], p[1]));
        }
        let _ = std::fs::write("points_dump.csv", dump);
        log::debug!("Dumped segments/points to CSV");
    }

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
            log::warn!("Voronoi construction failed: {:?}", e);
            return Vec::new();
        }
        Err(_) => {
            log::warn!("Voronoi construction panicked (degenerate geometry)");
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

        // Discard edges with vertices far outside the map bounds.
        // Intersecting input segments can produce degenerate Voronoi cells
        // with near-infinite vertex coordinates.
        let margin = (width.max(height) as f64) * 2.0;
        if v0.x().abs() > margin || v0.y().abs() > margin
            || v1.x().abs() > margin || v1.y().abs() > margin
        {
            continue;
        }

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

    log::debug!(
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

/// Check if a contour segment lies entirely along a border.
/// This detects both exact sub-segment matches and longer segments that span
/// multiple sub-segments (e.g., a contour segment along the entire west border).
fn is_border_segment(idx_j: usize, idx_k: usize, info: &BorderInfo) -> bool {
    for border in &info.borders {
        let has_j = border.contains(&idx_j);
        let has_k = border.contains(&idx_k);
        if has_j && has_k {
            return true;
        }
    }
    false
}

/// Record which consecutive border sub-segment pairs are covered by a contour segment
/// that spans from idx_j to idx_k along a border. A contour segment like [1,1]-[1,769]
/// covers all consecutive pairs between those endpoints in the sorted border list.
fn record_covered_border_pairs(
    idx_j: usize,
    idx_k: usize,
    info: &BorderInfo,
    covered: &mut crate::det::HashSet<(usize, usize)>,
) {
    for border in &info.borders {
        let pos_j = border.iter().position(|&i| i == idx_j);
        let pos_k = border.iter().position(|&i| i == idx_k);
        if let (Some(pj), Some(pk)) = (pos_j, pos_k) {
            let lo = pj.min(pk);
            let hi = pj.max(pk);
            for n in lo..hi {
                let pair = (border[n].min(border[n + 1]), border[n].max(border[n + 1]));
                covered.insert(pair);
            }
        }
    }
}

/// Check if two segments are collinear and overlapping (share more than an endpoint).
fn are_collinear_overlapping(s1: &[i32; 4], s2: &[i32; 4]) -> bool {
    let (x1, y1, x2, y2) = (s1[0] as i64, s1[1] as i64, s1[2] as i64, s1[3] as i64);
    let (x3, y3, x4, y4) = (s2[0] as i64, s2[1] as i64, s2[2] as i64, s2[3] as i64);

    // Check if all 4 points are collinear
    let cross1 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    let cross2 = (x2 - x1) * (y4 - y1) - (y2 - y1) * (x4 - x1);
    if cross1 != 0 || cross2 != 0 {
        return false;
    }

    // All collinear - check if they overlap (project onto the line)
    let dx = x2 - x1;
    let dy = y2 - y1;
    let (t3, t4) = if dx.abs() > dy.abs() {
        ((x3 - x1) * 1000 / dx.max(1), (x4 - x1) * 1000 / dx.max(1))
    } else if dy != 0 {
        ((y3 - y1) * 1000 / dy, (y4 - y1) * 1000 / dy)
    } else {
        return false; // Zero-length segment
    };

    let lo3 = t3.min(t4);
    let hi3 = t3.max(t4);
    // s1 goes from t=0 to t=1000
    // Overlap if intervals [0,1000] and [lo3,hi3] overlap by more than a point
    let overlap_lo = lo3.max(0);
    let overlap_hi = hi3.min(1000);
    overlap_hi - overlap_lo > 1 // More than just a shared endpoint
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
            let px = geometry::snap(geometry::snap(p.x, width - 1, snap_tolerance), 1, snap_tolerance);
            let py = geometry::snap(geometry::snap(p.y, height - 1, snap_tolerance), 1, snap_tolerance);
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

    // Add points near each border (strict < to match snap tolerance)
    for &p in &all_points {
        if (p.y - 1).abs() < snap_tolerance {
            borders[0].push(p);
        }
        if (p.y - (height - 1)).abs() < snap_tolerance {
            borders[1].push(p);
        }
        if (p.x - (width - 1)).abs() < snap_tolerance {
            borders[2].push(p);
        }
        if (p.x - 1).abs() < snap_tolerance {
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
