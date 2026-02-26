use std::collections::{HashMap, HashSet};

use crate::contour::Contour;
use crate::geometry::{
    add_bidirectional_edge, closest_point_on_segment, dijkstra, distance_to_segment,
    point_in_polygon, segments_intersect, Point,
};
use crate::graph::RegionGraph;
use crate::rtree::ObstacleRTree;

/// A navigable region.
#[derive(Debug, Clone)]
pub struct Region {
    pub id: usize,
    pub seed_node_id: usize,
    pub center: Point,
    pub radius: f64,
    pub polygon: Vec<Point>,
    pub holes: Vec<Vec<Point>>,
    pub is_closed: bool,
    /// The boundary was traced but invades an obstacle.
    pub invading: bool,
    /// Diagnostic: eliminated arcs (invaded) and retained arcs per obstacle.
    pub debug_arcs: Vec<DebugArc>,
    pub chokepoint_node_ids: HashSet<usize>,
}

/// Diagnostic info about arc selection on an obstacle.
#[derive(Debug, Clone)]
pub struct DebugArc {
    pub obstacle_id: usize,
    /// The arc that was eliminated (caused invasion).
    pub eliminated: Vec<Point>,
    /// The arc that was retained (the other one).
    pub retained: Vec<Point>,
}

/// A chokepoint (narrow passage between regions).
#[derive(Debug, Clone)]
pub struct Chokepoint {
    pub id: usize,
    pub node_id: usize,
    pub position: Point,
    pub radius: f64,
    pub crossing_points: Option<[Point; 2]>,
    pub obstacle_ids: Option<[usize; 2]>,
    pub region_ids: HashSet<usize>,
}

/// Crossing data for a chokepoint.
struct CrossingData {
    crossing_points: [Point; 2],
    obstacle_ids: [usize; 2],
}

/// Compute chokepoint gates and create Region/Chokepoint objects.
pub fn compute_chokepoint_gates(
    graph: &RegionGraph,
    rtree: &ObstacleRTree,
    width: i32,
    height: i32,
) -> (Vec<Region>, Vec<Chokepoint>) {
    let mut regions: Vec<Region> = Vec::new();
    let mut chokepoints: Vec<Chokepoint> = Vec::new();

    // Map from node_id -> region index for quick lookup
    let mut node_to_region: HashMap<usize, usize> = HashMap::new();

    let mut cp_id = 0;
    for &cp_node_id in &graph.chokepoint_nodes {
        let node = &graph.nodes[&cp_node_id];
        let mut cp = Chokepoint {
            id: cp_id,
            node_id: cp_node_id,
            position: node.position,
            radius: node.radius,
            crossing_points: None,
            obstacle_ids: None,
            region_ids: HashSet::new(),
        };
        cp_id += 1;

        // Find crossing points
        if let Some(crossing) = find_crossing_points(node.position, rtree, width, height) {
            cp.crossing_points = Some(crossing.crossing_points);
            cp.obstacle_ids = Some(crossing.obstacle_ids);
        }

        // Link to adjacent region nodes
        for &neighbor_id in graph.adjacency.get(&cp_node_id).unwrap_or(&HashSet::new()) {
            if graph.region_nodes.contains(&neighbor_id) {
                let region_idx = find_or_create_region(
                    &mut regions,
                    &mut node_to_region,
                    neighbor_id,
                    graph,
                );
                cp.region_ids.insert(region_idx);
                regions[region_idx].chokepoint_node_ids.insert(cp_node_id);
            }
        }

        chokepoints.push(cp);
    }

    // Ensure all region nodes have a Region object
    for &r_node_id in &graph.region_nodes {
        find_or_create_region(&mut regions, &mut node_to_region, r_node_id, graph);
    }

    eprintln!("Computed {} chokepoint gates, {} regions", chokepoints.len(), regions.len());
    (regions, chokepoints)
}

fn find_or_create_region(
    regions: &mut Vec<Region>,
    node_to_region: &mut HashMap<usize, usize>,
    node_id: usize,
    graph: &RegionGraph,
) -> usize {
    if let Some(&idx) = node_to_region.get(&node_id) {
        return idx;
    }
    let node = &graph.nodes[&node_id];
    let idx = regions.len();
    regions.push(Region {
        id: idx,
        seed_node_id: node_id,
        center: node.position,
        radius: node.radius,
        polygon: Vec::new(),
        holes: Vec::new(),
        is_closed: false,
        invading: false,
        debug_arcs: Vec::new(),
        chokepoint_node_ids: HashSet::new(),
    });
    node_to_region.insert(node_id, idx);
    idx
}

fn find_crossing_points(position: Point, rtree: &ObstacleRTree, _w: i32, _h: i32) -> Option<CrossingData> {
    let entries = rtree.nearest_n(position, 20);
    if entries.len() < 2 {
        return None;
    }

    // First pick: nearest segment (any, including border)
    let first = &entries[0];
    let cp1 = closest_point_on_segment(
        position,
        Point::new(first.1.p1[0].round() as i32, first.1.p1[1].round() as i32),
        Point::new(first.1.p2[0].round() as i32, first.1.p2[1].round() as i32),
    );

    // Direction from chokepoint center to first crossing point
    let d1x = cp1.x as f64 - position.x as f64;
    let d1y = cp1.y as f64 - position.y as f64;

    // Second pick: nearest segment whose closest point is on the opposite side
    // of the chokepoint (dot product with d1 is negative, i.e., angle > 90°).
    let mut best: Option<(usize, Point, f64)> = None; // (entry_index, crossing_point, distance)
    for (i, (_, seg, dist)) in entries.iter().enumerate() {
        if i == 0 {
            continue;
        }
        let cp_candidate = closest_point_on_segment(
            position,
            Point::new(seg.p1[0].round() as i32, seg.p1[1].round() as i32),
            Point::new(seg.p2[0].round() as i32, seg.p2[1].round() as i32),
        );
        // Skip if same crossing point
        if cp_candidate == cp1 {
            continue;
        }
        let d2x = cp_candidate.x as f64 - position.x as f64;
        let d2y = cp_candidate.y as f64 - position.y as f64;
        let dot = d1x * d2x + d1y * d2y;
        if dot < 0.0 {
            // Opposite side — accept the nearest one
            if best.is_none() || *dist < best.as_ref().unwrap().2 {
                best = Some((i, cp_candidate, *dist));
            }
            break; // entries are sorted by distance, so first opposite match is nearest
        }
    }

    let (second_idx, cp2, _) = best?;
    let second = &entries[second_idx];

    Some(CrossingData {
        crossing_points: [cp1, cp2],
        obstacle_ids: [first.0, second.0],
    })
}

/// Insert synthetic chokepoints for narrow gaps between unbridged obstacle pairs.
pub fn insert_synthetic_chokepoints(
    graph: &mut RegionGraph,
    regions: &mut Vec<Region>,
    chokepoints: &mut Vec<Chokepoint>,
    contours: &[Contour],
    outer_count: usize,
    rtree: &ObstacleRTree,
) {
    let gap_threshold = 8.0;

    // Collect already-bridged obstacle pairs
    let mut bridged_pairs: HashSet<u64> = HashSet::new();
    for cp in chokepoints.iter() {
        if let Some(ids) = cp.obstacle_ids {
            bridged_pairs.insert(pair_key(ids[0], ids[1]));
        }
    }

    let mut inserted = 0;
    // Only scan outer obstacle contours — hole contours are already Voronoi
    // input edges, so the medial axis handles them natively.  Treating holes
    // as independent obstacles creates spurious synthetic chokepoints (e.g.
    // between a hole wall and the map border).
    for (i, contour) in contours[..outer_count].iter().enumerate() {
        let obstacle_id = i + 1;
        for p_idx in 0..contour.len() {
            let pt = contour.points[p_idx];

            if let Some((other_id, seg, dist)) =
                rtree.nearest_from_different_obstacle(pt, obstacle_id, 10)
            {
                if dist >= gap_threshold {
                    continue;
                }

                let key = pair_key(obstacle_id, other_id);
                if bridged_pairs.contains(&key) {
                    continue;
                }

                let cp1 = pt;
                let cp2 = closest_point_on_segment(
                    pt,
                    Point::new(seg.p1[0].round() as i32, seg.p1[1].round() as i32),
                    Point::new(seg.p2[0].round() as i32, seg.p2[1].round() as i32),
                );
                let midpoint = Point::new((cp1.x + cp2.x) / 2, (cp1.y + cp2.y) / 2);

                let node_id = graph.allocate_id();
                graph.add_node_with_id(node_id, midpoint);
                graph.nodes.get_mut(&node_id).unwrap().radius = dist / 2.0;
                graph.chokepoint_nodes.insert(node_id);

                let mut synthetic_cp = Chokepoint {
                    id: chokepoints.len(),
                    node_id,
                    position: midpoint,
                    radius: dist / 2.0,
                    crossing_points: Some([cp1, cp2]),
                    obstacle_ids: Some([obstacle_id, other_id]),
                    region_ids: HashSet::new(),
                };

                // Link to nearest region on each side of gate line
                let gate_par_x = (cp2.x - cp1.x) as f64;
                let gate_par_y = (cp2.y - cp1.y) as f64;
                let gate_perp_x = -gate_par_y;
                let gate_perp_y = gate_par_x;
                let gate_len = (gate_par_x * gate_par_x + gate_par_y * gate_par_y).sqrt();
                let (gpx, gpy, gppx, gppy) = if gate_len > 0.0 {
                    (
                        gate_par_x / gate_len,
                        gate_par_y / gate_len,
                        gate_perp_x / gate_len,
                        gate_perp_y / gate_len,
                    )
                } else {
                    (0.0, 0.0, 0.0, 0.0)
                };

                let mut nearest_pos: Option<usize> = None;
                let mut nearest_neg: Option<usize> = None;
                let mut dist_pos = f64::MAX;
                let mut dist_neg = f64::MAX;

                for (ri, r) in regions.iter().enumerate() {
                    // Skip if line of sight crosses an existing chokepoint gate
                    if crosses_chokepoint_gate(midpoint, r.center, chokepoints) {
                        continue;
                    }

                    let dx = (r.center.x - midpoint.x) as f64;
                    let dy = (r.center.y - midpoint.y) as f64;
                    let perp_comp = dx * gppx + dy * gppy;
                    let par_comp = dx * gpx + dy * gpy;
                    let manhattan = perp_comp.abs() + par_comp.abs();

                    if perp_comp >= 0.0 {
                        if manhattan < dist_pos {
                            dist_pos = manhattan;
                            nearest_pos = Some(ri);
                        }
                    } else if manhattan < dist_neg {
                        dist_neg = manhattan;
                        nearest_neg = Some(ri);
                    }
                }

                if let Some(ri) = nearest_pos {
                    synthetic_cp.region_ids.insert(ri);
                    regions[ri].chokepoint_node_ids.insert(node_id);
                }
                if let Some(ri) = nearest_neg {
                    synthetic_cp.region_ids.insert(ri);
                    regions[ri].chokepoint_node_ids.insert(node_id);
                }

                eprintln!(
                    "Synthetic chokepoint C{} between O{} and O{} (gap={:.1}px)",
                    synthetic_cp.id, obstacle_id, other_id, dist
                );

                chokepoints.push(synthetic_cp);
                bridged_pairs.insert(key);
                inserted += 1;
            }
        }
    }
    if inserted > 0 {
        eprintln!("Inserted {} synthetic chokepoints", inserted);
    }
}

fn pair_key(a: usize, b: usize) -> u64 {
    let lo = a.min(b) as u64;
    let hi = a.max(b) as u64;
    (lo << 32) | hi
}

fn crosses_chokepoint_gate(from: Point, to: Point, chokepoints: &[Chokepoint]) -> bool {
    for cp in chokepoints {
        if let Some(cps) = cp.crossing_points {
            if segments_intersect(from, to, cps[0], cps[1]) {
                return true;
            }
        }
    }
    false
}

/// Split contours at gate points.
pub fn split_contours_at_gate_points(
    contours: &mut [Contour],
    chokepoints: &[Chokepoint],
) {
    for cp in chokepoints {
        let Some(crossing) = cp.crossing_points else {
            continue;
        };
        let Some(obstacle_ids) = cp.obstacle_ids else {
            continue;
        };

        for i in 0..2 {
            let point = crossing[i];
            let obstacle_id = obstacle_ids[i];
            if obstacle_id == 0 || obstacle_id > contours.len() {
                continue;
            }
            insert_point_into_contour(&mut contours[obstacle_id - 1], point);
        }
    }
}

fn insert_point_into_contour(contour: &mut Contour, point: Point) {
    // Check if point already exists
    if contour.points.contains(&point) {
        return;
    }

    let mut min_dist = f64::MAX;
    let mut best_idx: Option<usize> = None;

    for i in 0..contour.points.len() {
        let a = contour.points[i];
        let b = contour.points[(i + 1) % contour.points.len()];
        let dist = distance_to_segment(point, a, b);
        if dist < min_dist {
            min_dist = dist;
            best_idx = Some(i);
        }
    }

    if let Some(idx) = best_idx {
        if min_dist < 5.0 {
            contour.points.insert(idx + 1, point);
        }
    }
}

struct BoundaryResult {
    polygon: Option<Vec<Point>>,
    invading: bool,
    debug_arcs: Vec<DebugArc>,
}

/// Construct region boundaries.
pub fn construct_region_boundaries(
    regions: &mut [Region],
    chokepoints: &[Chokepoint],
    contours: &[Contour],
    width: i32,
    height: i32,
) {
    for i in 0..regions.len() {
        let result = build_region_boundary(i, regions, chokepoints, contours, width, height);
        regions[i].debug_arcs = result.debug_arcs;
        if let Some(b) = result.polygon {
            if b.len() >= 3 {
                regions[i].polygon = b;
                if result.invading {
                    regions[i].invading = true;
                } else {
                    regions[i].is_closed = true;
                }
            }
        }
    }
    let closed = regions.iter().filter(|r| r.is_closed).count();
    let invading = regions.iter().filter(|r| r.invading).count();
    if invading > 0 {
        eprintln!("  ({} invading regions kept for diagnostics)", invading);
    }

    // Detect interior obstacles: obstacle contours entirely inside a region
    // polygon, with no chokepoint gates on them.  These become holes.
    let gated_obstacles: HashSet<usize> = chokepoints
        .iter()
        .filter_map(|cp| cp.obstacle_ids)
        .flat_map(|ids| ids.into_iter())
        .collect();

    let mut total_holes = 0usize;
    for region in regions.iter_mut() {
        if !region.is_closed || region.polygon.len() < 3 {
            continue;
        }
        for (ci, contour) in contours.iter().enumerate() {
            let obs_id = ci + 1; // 1-based obstacle IDs
            if gated_obstacles.contains(&obs_id) {
                continue; // obstacle has chokepoint gates, not fully interior
            }
            if contour.points.is_empty() {
                continue;
            }
            // Check if the contour centroid is inside the region polygon
            let cx = contour.points.iter().map(|p| p.x as f64).sum::<f64>()
                / contour.points.len() as f64;
            let cy = contour.points.iter().map(|p| p.y as f64).sum::<f64>()
                / contour.points.len() as f64;
            let centroid = Point::new(cx.round() as i32, cy.round() as i32);
            if point_in_polygon(centroid, &region.polygon) {
                region.holes.push(contour.points.clone());
                total_holes += 1;
            }
        }
    }
    if total_holes > 0 {
        eprintln!("Found {} interior obstacle holes in regions", total_holes);
    }

    eprintln!("Built boundaries: {}/{} closed", closed, regions.len());
}

fn build_region_boundary(
    region_idx: usize,
    regions: &[Region],
    chokepoints: &[Chokepoint],
    contours: &[Contour],
    width: i32,
    height: i32,
) -> BoundaryResult {
    let region = &regions[region_idx];

    // Collect gates for this region
    let mut gates: Vec<[Point; 2]> = Vec::new();
    let mut gate_obstacle_ids: Vec<[usize; 2]> = Vec::new();

    for cp in chokepoints {
        if !region.chokepoint_node_ids.contains(&cp.node_id) {
            continue;
        }
        if let (Some(cps), Some(oids)) = (cp.crossing_points, cp.obstacle_ids) {
            gates.push(cps);
            gate_obstacle_ids.push(oids);
        }
    }

    if gates.is_empty() {
        BoundaryResult {
            polygon: build_single_contour_boundary(region, contours, width, height),
            invading: false,
            debug_arcs: Vec::new(),
        }
    } else {
        build_gated_boundary(region, &gates, &gate_obstacle_ids, contours, width, height)
    }
}

fn build_single_contour_boundary(
    region: &Region,
    contours: &[Contour],
    width: i32,
    height: i32,
) -> Option<Vec<Point>> {
    // Find nearest obstacle contour
    let mut min_dist = f64::MAX;
    let mut nearest_contour_idx = 0;
    let mut nearest_point_idx = 0;

    for (ci, c) in contours.iter().enumerate() {
        for (pi, p) in c.points.iter().enumerate() {
            let d = p.dist_sq(&region.center);
            if d < min_dist {
                min_dist = d;
                nearest_contour_idx = ci;
                nearest_point_idx = pi;
            }
        }
    }

    if contours.is_empty() {
        return None;
    }

    let nearest = &contours[nearest_contour_idx];
    let n = nearest.points.len();

    // Classify vertices as on/off map edge
    let on_edge: Vec<bool> = nearest
        .points
        .iter()
        .map(|p| is_on_map_edge(*p, width, height))
        .collect();
    let edge_count = on_edge.iter().filter(|&&e| e).count();

    if edge_count == 0 || edge_count == n {
        return Some(nearest.points.clone());
    }

    // Find interior arcs
    let mut arcs: Vec<(usize, usize)> = Vec::new();
    for i in 0..n {
        let next = (i + 1) % n;
        if on_edge[i] && !on_edge[next] {
            // Exit point — scan forward to find enter
            let mut enter_idx = None;
            let mut j = next;
            loop {
                if on_edge[j] {
                    enter_idx = Some(j);
                    break;
                }
                j = (j + 1) % n;
                if j == i {
                    break;
                }
            }
            if let Some(ei) = enter_idx {
                arcs.push((i, ei));
            }
        }
    }

    if arcs.is_empty() {
        return Some(nearest.points.clone());
    }

    // Pick arc containing/nearest to closest contour point
    let mut best_arc = arcs[0];
    if arcs.len() > 1 {
        for &arc in &arcs {
            let mut i = arc.0;
            let mut contains = false;
            loop {
                if i == nearest_point_idx {
                    contains = true;
                    break;
                }
                if i == arc.1 {
                    break;
                }
                i = (i + 1) % n;
            }
            if contains {
                best_arc = arc;
                break;
            }
        }
    }

    // Extract arc
    let mut obstacle_arc = Vec::new();
    let mut i = best_arc.0;
    loop {
        obstacle_arc.push(nearest.points[i]);
        if i == best_arc.1 {
            break;
        }
        i = (i + 1) % n;
    }

    // Close with map edge path
    let arc_end = *obstacle_arc.last().unwrap();
    let arc_start = obstacle_arc[0];
    if let Some(map_path) = get_map_edge_path(arc_end, arc_start, width, height) {
        if map_path.len() > 1 {
            for p in &map_path[1..] {
                obstacle_arc.push(*p);
            }
        }
    }

    Some(obstacle_arc)
}

/// BFS result with path and set of chokepoint indices crossed during traversal.
struct BfsResult {
    path: Vec<Point>,
    visited_chokes: HashSet<usize>,
}

/// BFS with forced gate crossing.
///
/// When the traversal reaches a vertex that is a gate point of an unvisited
/// chokepoint belonging to this region, the only edge expanded is the gate
/// edge to the partner point.  This forces the boundary path through every
/// chokepoint gate.
///
/// * `gate_map` — maps each gate point to `(partner_point, gate_index)`
/// * `initial_visited` — chokepoints already considered crossed (start choke)
/// * `prohibited_edges` — directed edges that must not be traversed
fn bfs_forced_gates(
    graph: &HashMap<Point, Vec<Point>>,
    start: Point,
    goal: Point,
    gate_map: &HashMap<Point, Vec<(Point, usize)>>,
    initial_visited: &HashSet<usize>,
    prohibited_edges: &HashSet<(Point, Point)>,
) -> Option<BfsResult> {
    use std::collections::VecDeque;

    let mut prev: HashMap<Point, Point> = HashMap::new();
    let mut visited_chokes = initial_visited.clone();
    let mut queue: VecDeque<Point> = VecDeque::new();
    let mut processed: HashSet<Point> = HashSet::new();

    queue.push_back(start);

    while let Some(point) = queue.pop_front() {
        if processed.contains(&point) {
            continue;
        }
        processed.insert(point);

        if point == goal {
            let mut path = Vec::new();
            let mut p = goal;
            loop {
                path.push(p);
                match prev.get(&p) {
                    Some(&pr) => p = pr,
                    None => break,
                }
            }
            path.reverse();
            return Some(BfsResult { path, visited_chokes });
        }

        // Check for forced gate crossing: if this point is a gate point of an
        // unvisited chokepoint, only expand the gate edge.
        let mut forced = false;
        if let Some(entries) = gate_map.get(&point) {
            for &(partner, gate_idx) in entries {
                if !visited_chokes.contains(&gate_idx) {
                    visited_chokes.insert(gate_idx);
                    prev.insert(partner, point);
                    // Allow re-exploration from the far side even if previously
                    // reached from a different direction.
                    processed.remove(&partner);
                    queue.push_back(partner);
                    forced = true;
                    break; // cross one gate at a time
                }
            }
        }
        if forced {
            continue;
        }

        // Normal BFS expansion
        if let Some(neighbors) = graph.get(&point) {
            for &neighbor in neighbors {
                if processed.contains(&neighbor) {
                    continue;
                }
                if prohibited_edges.contains(&(point, neighbor)) {
                    continue;
                }
                // Only enqueue if not yet reached (start has no prev entry)
                if !prev.contains_key(&neighbor) && neighbor != start {
                    prev.insert(neighbor, point);
                    queue.push_back(neighbor);
                }
            }
        }
    }

    None
}

fn build_gated_boundary(
    region: &Region,
    gates: &[[Point; 2]],
    gate_obstacle_ids: &[[usize; 2]],
    contours: &[Contour],
    width: i32,
    height: i32,
) -> BoundaryResult {
    let center = region.center;

    // Collect relevant obstacles
    let mut relevant_obstacles: HashSet<usize> = HashSet::new();
    for ids in gate_obstacle_ids {
        for &id in ids {
            if id > 0 && id <= contours.len() {
                relevant_obstacles.insert(id);
            }
        }
    }

    let mut adj_graph: HashMap<Point, Vec<Point>> = HashMap::new();

    // 1. For each obstacle, add region-facing arcs
    for &obstacle_id in &relevant_obstacles {
        let c = &contours[obstacle_id - 1];

        // Find gate points on this obstacle
        let mut gate_indices: Vec<usize> = Vec::new();
        for (g, ids) in gate_obstacle_ids.iter().enumerate() {
            for side in 0..2 {
                if ids[side] == obstacle_id {
                    if let Some(idx) = find_point_in_contour(c, gates[g][side]) {
                        if !gate_indices.contains(&idx) {
                            gate_indices.push(idx);
                        }
                    }
                }
            }
        }

        if gate_indices.len() < 2 {
            if gate_indices.len() == 1 {
                // 1 gate point: add only the region-facing arc.
                let gi = gate_indices[0];
                let n = c.points.len();
                let next_fwd = (gi + 1) % n;
                let next_rev = (gi + n - 1) % n;
                let d_fwd = c.points[next_fwd].dist_sq(&center);
                let d_rev = c.points[next_rev].dist_sq(&center);
                let opposite = (gi + n / 2) % n;
                let chosen = if d_fwd < d_rev {
                    extract_arc(c, gi, opposite)
                } else {
                    extract_arc(c, opposite, gi)
                };
                for j in 0..chosen.len() - 1 {
                    add_bidirectional_edge(&mut adj_graph, chosen[j], chosen[j + 1]);
                }
            } else {
                // 0 gate points: break contour at farthest vertex from center
                let mut farthest_idx = 0;
                let mut max_dist = 0.0f64;
                for (i, p) in c.points.iter().enumerate() {
                    let d = p.dist_sq(&center);
                    if d > max_dist {
                        max_dist = d;
                        farthest_idx = i;
                    }
                }
                let break_next = (farthest_idx + 1) % c.points.len();
                for i in 0..c.points.len() {
                    let next = (i + 1) % c.points.len();
                    if i == farthest_idx && next == break_next {
                        continue;
                    }
                    add_bidirectional_edge(&mut adj_graph, c.points[i], c.points[next]);
                }
            }
            continue;
        }

        gate_indices.sort();

        // For each consecutive pair of gate points, pick region-facing arc
        for i in 0..gate_indices.len() {
            let idx_a = gate_indices[i];
            let idx_b = gate_indices[(i + 1) % gate_indices.len()];

            let arc_fwd = extract_arc(c, idx_a, idx_b);
            let arc_rev = extract_arc(c, idx_b, idx_a);

            let centroid_fwd = arc_centroid(&arc_fwd);
            let centroid_rev = arc_centroid(&arc_rev);
            let d_fwd = centroid_fwd.dist(&center);
            let d_rev = centroid_rev.dist(&center);

            let chosen = if d_fwd < d_rev { &arc_fwd } else { &arc_rev };
            for j in 0..chosen.len() - 1 {
                add_bidirectional_edge(&mut adj_graph, chosen[j], chosen[j + 1]);
            }
        }
    }

    // 2. Add gate edges
    for gate in gates {
        add_bidirectional_edge(&mut adj_graph, gate[0], gate[1]);
    }

    // 3. Add map boundary edges
    add_map_boundary_edges(&mut adj_graph, width, height);

    // Build gate lookup: each gate point → [(partner, gate_index), ...]
    let mut gate_map: HashMap<Point, Vec<(Point, usize)>> = HashMap::new();
    for (i, gate) in gates.iter().enumerate() {
        gate_map.entry(gate[0]).or_default().push((gate[1], i));
        gate_map.entry(gate[1]).or_default().push((gate[0], i));
    }

    // Pick starting chokepoint: prefer one that bridges two different obstacles.
    let start_gate_idx = gate_obstacle_ids
        .iter()
        .position(|ids| ids[0] != ids[1])
        .unwrap_or(0);
    let start = gates[start_gate_idx][0];
    let goal = gates[start_gate_idx][1];

    // Start chokepoint is pre-visited; its direct edge is prohibited so BFS
    // must go the long way around.
    let mut initial_visited: HashSet<usize> = HashSet::new();
    initial_visited.insert(start_gate_idx);
    let all_gate_indices: HashSet<usize> = (0..gates.len()).collect();

    let mut prohibited_edges: HashSet<(Point, Point)> = HashSet::new();
    prohibited_edges.insert((start, goal));
    prohibited_edges.insert((goal, start));

    // --- Phase 1: BFS with forced gate crossing + rewind ---
    let max_rewinds = gates.len() + 2;
    let mut best_path: Option<Vec<Point>> = None;
    let mut best_visited: HashSet<usize> = HashSet::new();

    for rewind in 0..max_rewinds {
        let result = bfs_forced_gates(
            &adj_graph, start, goal, &gate_map, &initial_visited, &prohibited_edges,
        );

        let Some(bfs) = result else { break };
        if bfs.path.len() < 3 {
            break;
        }

        let all_visited = all_gate_indices.iter().all(|i| bfs.visited_chokes.contains(i));

        // Keep the best result (most gates visited)
        if best_path.is_none() || bfs.visited_chokes.len() > best_visited.len() {
            best_visited = bfs.visited_chokes.clone();
            best_path = Some(bfs.path.clone());
        }

        if all_visited {
            break;
        }

        // Rewind: find the last gate crossing in the path and prohibit the
        // edge after it, forcing BFS to explore the other direction next time.
        let last_crossing = find_last_gate_crossing(&bfs.path, gates, start_gate_idx);
        match last_crossing {
            Some(path_idx) => {
                let far_side = bfs.path[path_idx + 1];
                if path_idx + 2 < bfs.path.len() {
                    let wrong_next = bfs.path[path_idx + 2];
                    eprintln!(
                        "  R{}: rewind {}: prohibiting ({},{}) -> ({},{}) after gate crossing",
                        region.id, rewind + 1,
                        far_side.x, far_side.y, wrong_next.x, wrong_next.y,
                    );
                    prohibited_edges.insert((far_side, wrong_next));
                    prohibited_edges.insert((wrong_next, far_side));
                } else {
                    break;
                }
            }
            None => break,
        }
    }

    let Some(mut path) = best_path else {
        eprintln!("  R{}: BFS failed to find path", region.id);
        return BoundaryResult {
            polygon: None,
            invading: false,
            debug_arcs: Vec::new(),
        };
    };

    // --- Phase 2: invasion check + arc swap retries ---
    let max_invasion_retries = gates.len() + 2;
    let mut last_invaded_path: Option<Vec<Point>> = None;
    let mut debug_arcs: Vec<DebugArc> = Vec::new();

    for attempt in 0..max_invasion_retries {
        let path_set: HashSet<Point> = path.iter().copied().collect();
        let mut invaded_obstacle = None;
        for &obstacle_id in &relevant_obstacles {
            let c = &contours[obstacle_id - 1];
            let any_inside = c.points.iter()
                .filter(|p| !path_set.contains(p))
                .any(|p| point_in_polygon(*p, &path));
            if any_inside {
                invaded_obstacle = Some(obstacle_id);
                break;
            }
        }

        let Some(invaded_id) = invaded_obstacle else {
            // No invasion — success
            return BoundaryResult {
                polygon: Some(path),
                invading: false,
                debug_arcs,
            };
        };

        let invaded = &contours[invaded_id - 1];

        // Identify traversed vs non-traversed edges on the invaded obstacle
        let mut path_edges: HashSet<(Point, Point)> = HashSet::new();
        for i in 0..path.len() - 1 {
            path_edges.insert((path[i], path[i + 1]));
            path_edges.insert((path[i + 1], path[i]));
        }

        let mut eliminated_arc: Vec<Point> = Vec::new();
        let mut retained_arc: Vec<Point> = Vec::new();
        for i in 0..invaded.points.len() {
            let a = invaded.points[i];
            let b = invaded.points[(i + 1) % invaded.points.len()];
            if path_edges.contains(&(a, b)) {
                if eliminated_arc.is_empty() || *eliminated_arc.last().unwrap() != a {
                    eliminated_arc.push(a);
                }
                eliminated_arc.push(b);
            } else {
                if retained_arc.is_empty() || *retained_arc.last().unwrap() != a {
                    retained_arc.push(a);
                }
                retained_arc.push(b);
            }
        }

        eprintln!(
            "  R{}: invasion of O{} ({} contour pts), eliminated {} edges, retained {} edges (attempt {})",
            region.id, invaded_id, invaded.points.len(),
            eliminated_arc.len().saturating_sub(1),
            retained_arc.len().saturating_sub(1),
            attempt + 1
        );

        debug_arcs.push(DebugArc {
            obstacle_id: invaded_id,
            eliminated: eliminated_arc,
            retained: retained_arc,
        });

        // Swap arcs: remove all edges of the invaded obstacle, then re-add
        // only the non-traversed edges.
        for i in 0..invaded.points.len() {
            let p1 = invaded.points[i];
            let p2 = invaded.points[(i + 1) % invaded.points.len()];
            if let Some(neighbors) = adj_graph.get_mut(&p1) {
                neighbors.retain(|p| *p != p2);
            }
            if let Some(neighbors) = adj_graph.get_mut(&p2) {
                neighbors.retain(|p| *p != p1);
            }
        }
        for i in 0..invaded.points.len() {
            let a = invaded.points[i];
            let b = invaded.points[(i + 1) % invaded.points.len()];
            if !path_edges.contains(&(a, b)) {
                add_bidirectional_edge(&mut adj_graph, a, b);
            }
        }
        last_invaded_path = Some(path);

        // Re-run BFS on the modified graph
        match bfs_forced_gates(
            &adj_graph, start, goal, &gate_map, &initial_visited, &prohibited_edges,
        ) {
            Some(bfs) if bfs.path.len() >= 3 => {
                path = bfs.path;
            }
            _ => break,
        }
    }

    // Return the last invaded path for diagnostic rendering
    if let Some(p) = last_invaded_path {
        eprintln!("  R{}: invading boundary kept after retries", region.id);
        return BoundaryResult {
            polygon: Some(p),
            invading: true,
            debug_arcs,
        };
    }

    eprintln!("  R{}: boundary failed after retries", region.id);
    BoundaryResult {
        polygon: None,
        invading: false,
        debug_arcs,
    }
}

/// Find the last gate crossing in a path (excluding the start gate).
/// Returns the path index where path[idx] → path[idx+1] is a gate edge.
fn find_last_gate_crossing(
    path: &[Point],
    gates: &[[Point; 2]],
    start_gate_idx: usize,
) -> Option<usize> {
    let mut gate_edges: HashSet<(Point, Point)> = HashSet::new();
    for (i, gate) in gates.iter().enumerate() {
        if i == start_gate_idx {
            continue;
        }
        gate_edges.insert((gate[0], gate[1]));
        gate_edges.insert((gate[1], gate[0]));
    }

    let mut last = None;
    for i in 0..path.len() - 1 {
        if gate_edges.contains(&(path[i], path[i + 1])) {
            last = Some(i);
        }
    }
    last
}

fn find_point_in_contour(c: &Contour, p: Point) -> Option<usize> {
    // Exact match first
    if let Some(idx) = c.points.iter().position(|&cp| cp == p) {
        return Some(idx);
    }
    // Nearest within tolerance
    let mut min_dist = f64::MAX;
    let mut best = None;
    for (i, cp) in c.points.iter().enumerate() {
        let d = cp.dist(&p);
        if d < min_dist {
            min_dist = d;
            best = Some(i);
        }
    }
    if min_dist < 3.0 {
        best
    } else {
        None
    }
}

fn arc_centroid(arc: &[Point]) -> Point {
    let n = arc.len() as f64;
    let cx = arc.iter().map(|p| p.x as f64).sum::<f64>() / n;
    let cy = arc.iter().map(|p| p.y as f64).sum::<f64>() / n;
    Point::new(cx.round() as i32, cy.round() as i32)
}

fn extract_arc(c: &Contour, from: usize, to: usize) -> Vec<Point> {
    let n = c.points.len();
    let mut arc = Vec::new();
    let mut i = from;
    loop {
        arc.push(c.points[i]);
        if i == to {
            break;
        }
        i = (i + 1) % n;
    }
    arc
}

fn is_on_map_edge(p: Point, width: i32, height: i32) -> bool {
    p.x <= 1 || p.x >= width - 2 || p.y <= 1 || p.y >= height - 2
}

fn get_map_edge_path(from: Point, to: Point, width: i32, height: i32) -> Option<Vec<Point>> {
    let mut map_graph: HashMap<Point, Vec<Point>> = HashMap::new();
    let corners = [
        Point::new(1, 1),
        Point::new(width - 2, 1),
        Point::new(width - 2, height - 2),
        Point::new(1, height - 2),
    ];

    let mut all_edge_points = vec![from, to];
    all_edge_points.extend_from_slice(&corners);

    connect_edge_points(&mut map_graph, &all_edge_points, &corners, 0, width, height);
    connect_edge_points(&mut map_graph, &all_edge_points, &corners, 1, width, height);
    connect_edge_points(&mut map_graph, &all_edge_points, &corners, 2, width, height);
    connect_edge_points(&mut map_graph, &all_edge_points, &corners, 3, width, height);

    dijkstra(&map_graph, from, to, None)
}

fn add_map_boundary_edges(adj_graph: &mut HashMap<Point, Vec<Point>>, width: i32, height: i32) {
    let edge_points: Vec<Point> = adj_graph
        .keys()
        .copied()
        .filter(|p| is_on_map_edge(*p, width, height))
        .collect();

    let corners = [
        Point::new(1, 1),
        Point::new(width - 2, 1),
        Point::new(width - 2, height - 2),
        Point::new(1, height - 2),
    ];

    connect_edge_points(adj_graph, &edge_points, &corners, 0, width, height);
    connect_edge_points(adj_graph, &edge_points, &corners, 1, width, height);
    connect_edge_points(adj_graph, &edge_points, &corners, 2, width, height);
    connect_edge_points(adj_graph, &edge_points, &corners, 3, width, height);
}

fn connect_edge_points(
    adj_graph: &mut HashMap<Point, Vec<Point>>,
    edge_points: &[Point],
    corners: &[Point; 4],
    edge_index: usize,
    width: i32,
    height: i32,
) {
    let mut on_this_edge: Vec<Point> = Vec::new();

    for &p in edge_points {
        let on_edge = match edge_index {
            0 => p.y <= 1,
            1 => p.x >= width - 2,
            2 => p.y >= height - 2,
            3 => p.x <= 1,
            _ => false,
        };
        if on_edge {
            on_this_edge.push(p);
        }
    }

    if on_this_edge.is_empty() {
        return;
    }

    match edge_index {
        0 => {
            on_this_edge.push(corners[0]);
            on_this_edge.push(corners[1]);
            on_this_edge.sort_by_key(|p| p.x);
        }
        1 => {
            on_this_edge.push(corners[1]);
            on_this_edge.push(corners[2]);
            on_this_edge.sort_by_key(|p| p.y);
        }
        2 => {
            on_this_edge.push(corners[2]);
            on_this_edge.push(corners[3]);
            on_this_edge.sort_by_key(|p| p.x);
        }
        3 => {
            on_this_edge.push(corners[3]);
            on_this_edge.push(corners[0]);
            on_this_edge.sort_by_key(|p| p.y);
        }
        _ => {}
    }

    on_this_edge.dedup();

    for i in 0..on_this_edge.len() - 1 {
        add_bidirectional_edge(adj_graph, on_this_edge[i], on_this_edge[i + 1]);
    }
}

/// Remove duplicate regions with identical polygons.
pub fn remove_duplicate_regions(regions: &mut Vec<Region>, chokepoints: &mut [Chokepoint]) {
    let mut to_remove: Vec<usize> = Vec::new();

    for i in 0..regions.len() {
        if to_remove.contains(&i) || !regions[i].is_closed {
            continue;
        }
        for j in (i + 1)..regions.len() {
            if to_remove.contains(&j) || !regions[j].is_closed {
                continue;
            }
            if polygons_equal(&regions[i].polygon, &regions[j].polygon) {
                let loser = if regions[i].radius >= regions[j].radius {
                    j
                } else {
                    i
                };
                to_remove.push(loser);
                eprintln!(
                    "Duplicate territory: R{} vs R{} -> removing R{}",
                    regions[i].id, regions[j].id, regions[loser].id
                );
            }
        }
    }

    // Remove in reverse order to preserve indices
    to_remove.sort();
    to_remove.dedup();
    for &idx in to_remove.iter().rev() {
        let removed_id = regions[idx].id;
        regions.remove(idx);
        for cp in chokepoints.iter_mut() {
            cp.region_ids.remove(&removed_id);
        }
    }
}

fn polygons_equal(a: &[Point], b: &[Point]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b.iter()).all(|(a, b)| a == b)
}
