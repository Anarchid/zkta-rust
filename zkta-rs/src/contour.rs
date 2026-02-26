use crate::geometry::Point;
use geo::{LineString, Simplify, Area, Coord};

/// A closed contour polygon (obstacle or hole boundary).
#[derive(Debug, Clone)]
pub struct Contour {
    pub points: Vec<Point>,
}

/// A contour with an associated obstacle label ID.
#[derive(Debug, Clone)]
pub struct LabeledContour {
    pub points: Vec<Point>,
    pub label: i32,
}

impl Contour {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub fn from_points(pts: Vec<Point>) -> Self {
        Self { points: pts }
    }

    pub fn add_point(&mut self, p: Point) {
        self.points.push(p);
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Returns [min_x, max_x, min_y, max_y].
    pub fn bounds(&self) -> Option<[i32; 4]> {
        if self.points.is_empty() {
            return None;
        }
        let mut min_x = i32::MAX;
        let mut max_x = i32::MIN;
        let mut min_y = i32::MAX;
        let mut max_y = i32::MIN;
        for p in &self.points {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        Some([min_x, max_x, min_y, max_y])
    }

    /// Shoelace area formula.
    pub fn area(&self) -> f64 {
        if self.points.len() < 3 {
            return 0.0;
        }
        let n = self.points.len();
        let mut a = 0.0f64;
        for i in 0..n {
            let cur = self.points[i];
            let next = self.points[(i + 1) % n];
            a += (cur.x as f64) * (next.y as f64) - (next.x as f64) * (cur.y as f64);
        }
        a.abs() / 2.0
    }
}

/// Direction offsets for Moore neighborhood tracing (8-connected).
/// Index 0..7: (dy, dx) pairs.
const SEARCH_DIRECTION: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

/// Result of a single tracer step.
struct TracePoint {
    x: i32,
    y: i32,
    direction: usize,
}

/// Trace all obstacle contours and holes from the image.
///
/// Returns (contours, holes, label_map).
pub fn trace_contours(
    width: u32,
    height: u32,
    is_obstacle: &dyn Fn(u32, u32) -> bool,
) -> (Vec<Contour>, Vec<LabeledContour>, Vec<Vec<i32>>) {
    let w = width as i32;
    let h = height as i32;
    let mut label_map = vec![vec![0i32; width as usize]; height as usize];
    let mut contours = Vec::new();
    let mut holes = Vec::new();
    let mut connected_components_count = 0i32;

    for cx in 0..w {
        let mut label_id = 0i32;
        for cy in 0..h {
            if is_obstacle(cx as u32, cy as u32) {
                if label_id != 0 {
                    label_map[cy as usize][cx as usize] = label_id;
                } else {
                    label_id = label_map[cy as usize][cx as usize];
                    if label_id == 0 {
                        connected_components_count += 1;
                        label_id = connected_components_count;
                        let contour = trace_contour(
                            cx, cy, label_id, 0, w, h, is_obstacle, &mut label_map,
                        );
                        contours.push(contour);
                        label_map[cy as usize][cx as usize] = label_id;
                    }
                }
            } else if label_id != 0 {
                if label_map[cy as usize][cx as usize] == 0 {
                    let hole_contour = trace_contour(
                        cx,
                        cy - 1,
                        label_id,
                        1,
                        w,
                        h,
                        is_obstacle,
                        &mut label_map,
                    );
                    flood_fill_hole(cx, cy, w, h, &mut label_map, is_obstacle);
                    holes.push(LabeledContour {
                        points: hole_contour.points.clone(),
                        label: label_id,
                    });
                }
                label_id = 0;
            }
        }
    }

    eprintln!(
        "Traced {} contours, {} holes",
        contours.len(),
        holes.len()
    );
    (contours, holes, label_map)
}

/// Dump the first aborted contour to a diagnostic image.
/// Only writes once (uses a static flag to avoid spamming).
fn dump_aborted_contour(contour: &Contour, w: i32, h: i32, sx: i32, sy: i32) {
    use std::sync::atomic::{AtomicBool, Ordering};
    static DUMPED: AtomicBool = AtomicBool::new(false);
    if DUMPED.swap(true, Ordering::Relaxed) {
        return; // already dumped one
    }

    let wu = w as u32;
    let hu = h as u32;
    let mut img = image::RgbImage::new(wu, hu);

    // Draw the contour path: fade from green (start) to red (end)
    let n = contour.points.len();
    for i in 0..n {
        let t = i as f32 / n as f32;
        let r = (t * 255.0) as u8;
        let g = ((1.0 - t) * 255.0) as u8;
        let p = contour.points[i];
        if p.x >= 0 && p.x < w && p.y >= 0 && p.y < h {
            img.put_pixel(p.x as u32, p.y as u32, image::Rgb([r, g, 0]));
        }
    }

    // Mark the start pixel in bright cyan
    if sx >= 0 && sx < w && sy >= 0 && sy < h {
        for dy in -2i32..=2 {
            for dx in -2i32..=2 {
                let px = sx + dx;
                let py = sy + dy;
                if px >= 0 && px < w && py >= 0 && py < h {
                    img.put_pixel(px as u32, py as u32, image::Rgb([0, 255, 255]));
                }
            }
        }
    }

    let path = "aborted_contour.png";
    match img.save(path) {
        Ok(_) => eprintln!("    Dumped aborted contour ({} pts) to {}", n, path),
        Err(e) => eprintln!("    Failed to dump aborted contour: {}", e),
    }
}

fn trace_contour(
    cx: i32,
    cy: i32,
    label_id: i32,
    tracing_direction: usize,
    w: i32,
    h: i32,
    is_obstacle: &dyn Fn(u32, u32) -> bool,
    label_map: &mut Vec<Vec<i32>>,
) -> Contour {
    let mut contour = Contour::new();
    let sx = cx;
    let sy = cy;

    let p = tracer(cx, cy, tracing_direction, w, h, is_obstacle, label_map);
    contour.add_point(Point::new(p.x, p.y));

    if p.x != sx || p.y != sy {
        let fx = p.x;
        let fy = p.y;
        let mut px = p.x;
        let mut py = p.y;
        let mut pdir = p.direction;
        let mut tracing_stop_flag = false;
        let mut keep_searching = true;

        const MAX_CONTOUR_POINTS: usize = 100_000;
        while keep_searching {
            label_map[py as usize][px as usize] = label_id;
            let np = tracer(px, py, (pdir + 6) % 8, w, h, is_obstacle, label_map);
            contour.add_point(Point::new(np.x, np.y));
            px = np.x;
            py = np.y;
            pdir = np.direction;

            if contour.len() > MAX_CONTOUR_POINTS {
                eprintln!("    WARNING: contour starting at ({},{}) exceeded {} pts, aborting trace",
                    sx, sy, MAX_CONTOUR_POINTS);
                dump_aborted_contour(&contour, w, h, sx, sy);
                break;
            }

            if px == sx && py == sy {
                tracing_stop_flag = true;
            } else if tracing_stop_flag {
                if px == fx && py == fy {
                    keep_searching = false;
                } else {
                    tracing_stop_flag = false;
                }
            }
        }
    }
    contour
}

fn tracer(
    cx: i32,
    cy: i32,
    mut direction: usize,
    w: i32,
    h: i32,
    is_obstacle: &dyn Fn(u32, u32) -> bool,
    label_map: &mut Vec<Vec<i32>>,
) -> TracePoint {
    for _ in 0..7 {
        let dy = SEARCH_DIRECTION[direction].0;
        let dx = SEARCH_DIRECTION[direction].1;
        let y = cy + dy;
        let x = cx + dx;

        if x < 0 || x >= w || y < 0 || y >= h {
            direction = (direction + 1) % 8;
            continue;
        }

        if is_obstacle(x as u32, y as u32) {
            return TracePoint {
                x,
                y,
                direction,
            };
        } else {
            label_map[y as usize][x as usize] = -1;
            direction = (direction + 1) % 8;
        }
    }
    TracePoint {
        x: cx,
        y: cy,
        direction,
    }
}

/// BFS flood fill from the hole entry point.
///
/// Marks all passable pixels reachable from (start_x, start_y) as -1,
/// stopping at obstacle pixels.  This correctly handles nested obstacles
/// inside holes: the flood cannot cross obstacle boundaries, so passable
/// areas enclosed by inner obstacles remain at label_map == 0 and will be
/// detected as holes by the column scanner when it reaches them.
fn flood_fill_hole(
    start_x: i32,
    start_y: i32,
    w: i32,
    h: i32,
    label_map: &mut Vec<Vec<i32>>,
    is_obstacle: &dyn Fn(u32, u32) -> bool,
) {
    use std::collections::VecDeque;

    if start_x < 0 || start_x >= w || start_y < 0 || start_y >= h {
        return;
    }
    if label_map[start_y as usize][start_x as usize] != 0 {
        return;
    }
    if is_obstacle(start_x as u32, start_y as u32) {
        return;
    }

    label_map[start_y as usize][start_x as usize] = -1;
    let mut queue = VecDeque::new();
    queue.push_back((start_x, start_y));

    while let Some((x, y)) = queue.pop_front() {
        for &(dx, dy) in &[(0i32, 1i32), (0, -1), (1, 0), (-1, 0)] {
            let nx = x + dx;
            let ny = y + dy;
            if nx < 0 || nx >= w || ny < 0 || ny >= h {
                continue;
            }
            if label_map[ny as usize][nx as usize] != 0 {
                continue;
            }
            if is_obstacle(nx as u32, ny as u32) {
                continue;
            }
            label_map[ny as usize][nx as usize] = -1;
            queue.push_back((nx, ny));
        }
    }
}

/// Simplify hole contours (inner boundaries of obstacles) using Douglas-Peucker.
/// Same filtering as simplify_contours: >= 4 points and area >= 100.
pub fn simplify_holes(holes: &[LabeledContour], tolerance: f64) -> Vec<Contour> {
    let as_contours: Vec<Contour> = holes
        .iter()
        .map(|h| Contour::from_points(h.points.clone()))
        .collect();
    let result = simplify_contours_inner(&as_contours, tolerance, false);
    eprintln!("Simplified {} holes to {} hole contours", holes.len(), result.len());
    result
}

/// Fix simplified contours that became non-manifold after Douglas-Peucker:
///
/// 1. **Self-intersecting contours** (bowtie shapes): split at the intersection
///    point into two separate contours.  Each part is kept if it passes the size
///    filter (>3 pts, area >100).
///
/// 2. **Hole protrusion**: hole edges that cross their parent obstacle boundary.
///    The intersection point is inserted into the hole contour.
///
/// Returns the updated outer_count (splitting may add/remove outer contours).
pub fn make_contours_manifold(
    simplified_contours: &mut Vec<Contour>,
    outer_count: usize,
) -> usize {
    use crate::geometry::{closest_point_on_segment, point_in_polygon, segments_intersect};

    // Phase 1: Split self-intersecting contours
    let mut new_outers: Vec<Contour> = Vec::new();
    let mut new_holes: Vec<Contour> = Vec::new();
    let mut splits = 0usize;

    for (ci, contour) in simplified_contours.iter().enumerate() {
        let parts = split_self_intersections(contour);
        let is_hole = ci >= outer_count;
        if parts.len() > 1 {
            splits += parts.len() - 1;
            eprintln!(
                "  Split self-intersecting C{} ({}) into {} parts",
                ci,
                if is_hole { "hole" } else { "outer" },
                parts.len()
            );
        }
        for part in parts {
            if part.len() >= 3 && part.area() > 100.0 {
                if is_hole {
                    new_holes.push(part);
                } else {
                    new_outers.push(part);
                }
            }
        }
    }

    let new_outer_count = new_outers.len();
    simplified_contours.clear();
    simplified_contours.extend(new_outers);
    simplified_contours.extend(new_holes);

    if splits > 0 {
        eprintln!(
            "Made manifold: {} splits, {} outer + {} holes",
            splits, new_outer_count,
            simplified_contours.len() - new_outer_count
        );
    }

    // Phase 2: Clip holes to parent obstacle boundaries
    let mut total_snapped = 0usize;
    let mut total_inserted = 0usize;

    for hi in new_outer_count..simplified_contours.len() {
        let hole_points = &simplified_contours[hi].points;
        if hole_points.is_empty() {
            continue;
        }

        // Find parent: which outer contour contains this hole's centroid?
        let cx = hole_points.iter().map(|p| p.x as f64).sum::<f64>()
            / hole_points.len() as f64;
        let cy = hole_points.iter().map(|p| p.y as f64).sum::<f64>()
            / hole_points.len() as f64;
        let centroid = Point::new(cx.round() as i32, cy.round() as i32);

        // Find the smallest enclosing outer contour (not just the first)
        let mut parent_idx = None;
        let mut parent_area = f64::MAX;
        for oi in 0..new_outer_count {
            if point_in_polygon(centroid, &simplified_contours[oi].points) {
                let area = simplified_contours[oi].area();
                if area < parent_area {
                    parent_area = area;
                    parent_idx = Some(oi);
                }
            }
        }

        let Some(pi) = parent_idx else { continue };

        let parent = simplified_contours[pi].points.clone();

        // Snap protruding vertices
        let hole = &mut simplified_contours[hi].points;
        for p in hole.iter_mut() {
            if !point_in_polygon(*p, &parent) {
                let mut min_dist = f64::MAX;
                let mut nearest = *p;
                for j in 0..parent.len() {
                    let a = parent[j];
                    let b = parent[(j + 1) % parent.len()];
                    let cp = closest_point_on_segment(*p, a, b);
                    let d = p.dist(&cp);
                    if d < min_dist {
                        min_dist = d;
                        nearest = cp;
                    }
                }
                *p = nearest;
                total_snapped += 1;
            }
        }

        // Insert intersection points where hole edges cross parent edges
        let mut insertions: Vec<(usize, Point)> = Vec::new();
        let hole = &simplified_contours[hi].points;
        let hn = hole.len();
        for i in 0..hn {
            let h1 = hole[i];
            let h2 = hole[(i + 1) % hn];
            for j in 0..parent.len() {
                let p1 = parent[j];
                let p2 = parent[(j + 1) % parent.len()];
                if h1 == p1 || h1 == p2 || h2 == p1 || h2 == p2 {
                    continue;
                }
                if segments_intersect(h1, h2, p1, p2) {
                    if let Some(ix) = segment_intersection_point(h1, h2, p1, p2) {
                        insertions.push((i, ix));
                    }
                }
            }
        }

        insertions.sort_by_key(|(idx, _)| std::cmp::Reverse(*idx));
        insertions.dedup_by_key(|(idx, _)| *idx);
        let hole = &mut simplified_contours[hi].points;
        for (after_idx, point) in &insertions {
            hole.insert(after_idx + 1, *point);
            total_inserted += 1;
        }
    }

    if total_snapped > 0 || total_inserted > 0 {
        eprintln!(
            "Clipped holes: {} vertices snapped, {} intersection points inserted",
            total_snapped, total_inserted
        );
    }

    new_outer_count
}

/// Split a self-intersecting contour into non-intersecting parts.
fn split_self_intersections(contour: &Contour) -> Vec<Contour> {
    use crate::geometry::segments_intersect;

    let pts = &contour.points;
    let n = pts.len();
    if n < 4 {
        return vec![contour.clone()];
    }

    // Find first self-intersection
    for i in 0..n {
        let i_next = (i + 1) % n;
        for j in (i + 2)..n {
            let j_next = (j + 1) % n;
            // Skip adjacent edges (including wrap-around)
            if j_next == i {
                continue;
            }

            let a1 = pts[i];
            let a2 = pts[i_next];
            let b1 = pts[j];
            let b2 = pts[j_next];

            if a1 == b1 || a1 == b2 || a2 == b1 || a2 == b2 {
                continue;
            }

            if segments_intersect(a1, a2, b1, b2) {
                if let Some(ix) = segment_intersection_point(a1, a2, b1, b2) {
                    // Split into two loops at the intersection point
                    // Loop 1: ix → i+1 → ... → j → ix
                    let mut loop1 = vec![ix];
                    let mut k = i_next;
                    loop {
                        loop1.push(pts[k]);
                        if k == j {
                            break;
                        }
                        k = (k + 1) % n;
                    }

                    // Loop 2: ix → j+1 → ... → i → ix
                    let mut loop2 = vec![ix];
                    let mut k = j_next;
                    loop {
                        loop2.push(pts[k]);
                        if k == i {
                            break;
                        }
                        k = (k + 1) % n;
                    }

                    // Recurse on each loop (they might still self-intersect)
                    let c1 = Contour::from_points(loop1);
                    let c2 = Contour::from_points(loop2);
                    let mut result = split_self_intersections(&c1);
                    result.extend(split_self_intersections(&c2));
                    return result;
                }
            }
        }
    }

    vec![contour.clone()]
}

/// Compute the intersection point of two line segments.
fn segment_intersection_point(a1: Point, a2: Point, b1: Point, b2: Point) -> Option<Point> {
    let d1x = (a2.x - a1.x) as f64;
    let d1y = (a2.y - a1.y) as f64;
    let d2x = (b2.x - b1.x) as f64;
    let d2y = (b2.y - b1.y) as f64;
    let denom = d1x * d2y - d1y * d2x;
    if denom.abs() < 1e-10 {
        return None; // parallel
    }
    let t = ((b1.x - a1.x) as f64 * d2y - (b1.y - a1.y) as f64 * d2x) / denom;
    let ix = a1.x as f64 + t * d1x;
    let iy = a1.y as f64 + t * d1y;
    Some(Point::new(ix.round() as i32, iy.round() as i32))
}

/// Seal dropped holes in the label_map.
///
/// Holes that don't survive simplification still have their interior marked as
/// -1 in the label_map (from BFS flood fill during tracing).  The graph builder
/// checks `label_map > 0` to reject obstacle vertices, so -1 pixels let Voronoi
/// vertices slip through — creating phantom graph nodes inside obstacles.
///
/// This function re-simplifies each original hole to determine if it survived,
/// and for those that didn't, BFS-fills their -1 interior back to the parent
/// obstacle label.
pub fn seal_dropped_holes(
    holes: &[LabeledContour],
    tolerance: f64,
    label_map: &mut [Vec<i32>],
    width: u32,
    height: u32,
) {
    use std::collections::VecDeque;

    let w = width as i32;
    let h = height as i32;
    let mut sealed = 0;
    let mut total_pixels = 0usize;

    for hole in holes {
        // Re-simplify to check if this hole would survive
        let coords: Vec<Coord<f64>> = hole.points.iter().map(|p| Coord {
            x: p.x as f64,
            y: p.y as f64,
        }).collect();
        let ls = LineString::new(coords);
        let simplified = ls.simplify(&tolerance);
        let pts: Vec<Point> = simplified
            .0
            .iter()
            .map(|c| Point::new(c.x.round() as i32, c.y.round() as i32))
            .collect();
        let contour = Contour::from_points(pts);
        if contour.len() >= 3 && contour.area() > 100.0 {
            continue; // Survived simplification — keep as passable
        }

        // This hole was dropped.  Find a seed pixel inside it.
        let cx = hole.points.iter().map(|p| p.x as f64).sum::<f64>()
            / hole.points.len() as f64;
        let cy = hole.points.iter().map(|p| p.y as f64).sum::<f64>()
            / hole.points.len() as f64;
        let seed_x = cx.round() as i32;
        let seed_y = cy.round() as i32;

        let seed = if seed_x >= 0 && seed_x < w && seed_y >= 0 && seed_y < h
            && label_map[seed_y as usize][seed_x as usize] == -1
        {
            Some((seed_x, seed_y))
        } else {
            // Centroid outside hole (concave shape) — scan bounding box
            let bounds = Contour::from_points(hole.points.clone()).bounds();
            bounds.and_then(|[min_x, max_x, min_y, max_y]| {
                for y in min_y.max(0)..=max_y.min(h - 1) {
                    for x in min_x.max(0)..=max_x.min(w - 1) {
                        if label_map[y as usize][x as usize] == -1 {
                            return Some((x, y));
                        }
                    }
                }
                None
            })
        };

        let Some((sx, sy)) = seed else { continue };

        // BFS flood-fill: replace -1 with parent obstacle label.
        // The -1 area is bounded by positive labels (obstacle boundary),
        // so the fill stays within the hole interior.
        let fill_label = hole.label;
        let mut count = 0usize;
        label_map[sy as usize][sx as usize] = fill_label;
        let mut queue = VecDeque::new();
        queue.push_back((sx, sy));
        while let Some((x, y)) = queue.pop_front() {
            count += 1;
            for &(dx, dy) in &[(0i32, 1i32), (0, -1), (1, 0), (-1, 0)] {
                let nx = x + dx;
                let ny = y + dy;
                if nx < 0 || nx >= w || ny < 0 || ny >= h {
                    continue;
                }
                if label_map[ny as usize][nx as usize] != -1 {
                    continue;
                }
                label_map[ny as usize][nx as usize] = fill_label;
                queue.push_back((nx, ny));
            }
        }
        sealed += 1;
        total_pixels += count;
    }

    if sealed > 0 {
        eprintln!("Sealed {} dropped holes ({} pixels) in label_map", sealed, total_pixels);
    }
}

/// Clear label-map pixels belonging to obstacle contours that were dropped
/// during simplification (too few points or too small area).
///
/// Analogous to `seal_dropped_holes`: where that function *fills* dropped holes
/// to mark them as obstacle, this function *clears* dropped obstacles to mark
/// them as passable — keeping the label_map consistent with the simplified
/// contours that the Voronoi diagram actually sees.
pub fn clear_dropped_obstacles(
    contours: &[Contour],
    tolerance: f64,
    label_map: &mut [Vec<i32>],
    width: u32,
    height: u32,
) {
    use std::collections::HashSet;

    let mut dropped_labels: HashSet<i32> = HashSet::new();

    for (i, c) in contours.iter().enumerate() {
        let label = (i + 1) as i32; // label_map is 1-indexed
        let coords: Vec<Coord<f64>> = c.points.iter().map(|p| Coord {
            x: p.x as f64,
            y: p.y as f64,
        }).collect();
        let ls = LineString::new(coords);
        let simplified = ls.simplify(&tolerance);
        let mut pts: Vec<Point> = simplified
            .0
            .iter()
            .map(|c| Point::new(c.x.round() as i32, c.y.round() as i32))
            .collect();
        if pts.len() > 1 && pts.first() == pts.last() {
            pts.pop();
        }
        let contour = Contour::from_points(pts);
        if contour.len() >= 3 && contour.area() > 100.0 {
            continue; // Survived simplification
        }
        dropped_labels.insert(label);
    }

    if dropped_labels.is_empty() {
        return;
    }

    let mut cleared = 0usize;
    for y in 0..height as usize {
        for x in 0..width as usize {
            if dropped_labels.contains(&label_map[y][x]) {
                label_map[y][x] = 0;
                cleared += 1;
            }
        }
    }

    eprintln!(
        "Cleared {} pixels from {} dropped obstacles in label_map",
        cleared,
        dropped_labels.len()
    );
}

/// Simplify contours using Douglas-Peucker algorithm.
/// Returns only contours with >= 3 points and area >= 100.
pub fn simplify_contours(contours: &[Contour], tolerance: f64) -> Vec<Contour> {
    simplify_contours_inner(contours, tolerance, true)
}

fn simplify_contours_inner(contours: &[Contour], tolerance: f64, rescue_thin: bool) -> Vec<Contour> {
    let mut result = Vec::new();
    for c in contours {
        let contour = simplify_one(c, tolerance);
        if contour.len() >= 3 && contour.area() > 100.0 {
            result.push(contour);
            continue;
        }
        // Thin but important obstacle: re-simplify with lower tolerance
        // until we get a polygon.  Only for outer contours — rescuing thin
        // holes changes topology and causes regressions.
        if rescue_thin && c.area() > 100.0 {
            let mut tol = tolerance / 2.0;
            while tol >= 1.0 {
                let retry = simplify_one(c, tol);
                if retry.len() >= 3 {
                    result.push(retry);
                    break;
                }
                tol /= 2.0;
            }
        }
    }
    eprintln!("Simplified to {} contours", result.len());
    result
}

fn simplify_one(c: &Contour, tolerance: f64) -> Contour {
    let coords: Vec<Coord<f64>> = c.points.iter().map(|p| Coord {
        x: p.x as f64,
        y: p.y as f64,
    }).collect();
    let ls = LineString::new(coords);
    let simplified = ls.simplify(&tolerance);
    let mut pts: Vec<Point> = simplified
        .0
        .iter()
        .map(|c| Point::new(c.x.round() as i32, c.y.round() as i32))
        .collect();
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    Contour::from_points(pts)
}
