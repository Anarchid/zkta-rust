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
    pub is_closed: bool,
    pub chokepoint_node_ids: HashSet<usize>,
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
        if let Some(crossing) = find_crossing_points(node.position, rtree) {
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

    eprintln!("Computed {} chokepoint gates", chokepoints.len());
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
        is_closed: false,
        chokepoint_node_ids: HashSet::new(),
    });
    node_to_region.insert(node_id, idx);
    idx
}

fn find_crossing_points(position: Point, rtree: &ObstacleRTree) -> Option<CrossingData> {
    let entries = rtree.nearest_n(position, 10);
    if entries.len() < 2 {
        return None;
    }

    let first = &entries[0];
    let second = entries[1..]
        .iter()
        .find(|(oid, _, _)| *oid != first.0)?;

    let cp1 = closest_point_on_segment(
        position,
        Point::new(first.1.p1[0].round() as i32, first.1.p1[1].round() as i32),
        Point::new(first.1.p2[0].round() as i32, first.1.p2[1].round() as i32),
    );
    let cp2 = closest_point_on_segment(
        position,
        Point::new(second.1.p1[0].round() as i32, second.1.p1[1].round() as i32),
        Point::new(second.1.p2[0].round() as i32, second.1.p2[1].round() as i32),
    );

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
    for (i, contour) in contours.iter().enumerate() {
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

/// Construct region boundaries.
pub fn construct_region_boundaries(
    regions: &mut [Region],
    chokepoints: &[Chokepoint],
    contours: &[Contour],
    width: i32,
    height: i32,
) {
    for i in 0..regions.len() {
        let boundary = build_region_boundary(i, regions, chokepoints, contours, width, height);
        if let Some(b) = boundary {
            if b.len() >= 3 {
                regions[i].polygon = b;
                regions[i].is_closed = true;
            }
        }
    }
    let closed = regions.iter().filter(|r| r.is_closed).count();
    eprintln!("Built boundaries: {}/{} closed", closed, regions.len());
}

fn build_region_boundary(
    region_idx: usize,
    regions: &[Region],
    chokepoints: &[Chokepoint],
    contours: &[Contour],
    width: i32,
    height: i32,
) -> Option<Vec<Point>> {
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
        build_single_contour_boundary(region, contours, width, height)
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

fn build_gated_boundary(
    region: &Region,
    gates: &[[Point; 2]],
    gate_obstacle_ids: &[[usize; 2]],
    contours: &[Contour],
    width: i32,
    height: i32,
) -> Option<Vec<Point>> {
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
            // 0 or 1 gate points — break contour at farthest vertex
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
            continue;
        }

        gate_indices.sort();

        // For each consecutive pair of gate points, pick region-facing arc
        for i in 0..gate_indices.len() {
            let idx_a = gate_indices[i];
            let idx_b = gate_indices[(i + 1) % gate_indices.len()];

            let arc_fwd = extract_arc(c, idx_a, idx_b);
            let arc_rev = extract_arc(c, idx_b, idx_a);

            let mid_fwd = arc_fwd[arc_fwd.len() / 2];
            let mid_rev = arc_rev[arc_rev.len() / 2];
            let d_fwd = mid_fwd.dist(&center);
            let d_rev = mid_rev.dist(&center);

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

    // Dijkstra with invasion detection
    let start = gates[0][0];
    let goal = gates[0][1];

    for attempt in 0..4 {
        let path = dijkstra(&adj_graph, start, goal, Some(gates[0]))?;
        if path.len() < 3 {
            return None;
        }

        // Check for obstacle invasions
        let mut invaded_obstacle = None;
        for &obstacle_id in &relevant_obstacles {
            let c = &contours[obstacle_id - 1];
            let inside_count = c.points.iter().filter(|p| point_in_polygon(**p, &path)).count();
            if inside_count > c.points.len() / 2 {
                invaded_obstacle = Some(obstacle_id);
                break;
            }
        }

        if invaded_obstacle.is_none() {
            return Some(path);
        }

        let invaded_id = invaded_obstacle.unwrap();
        let invaded = &contours[invaded_id - 1];

        // Remove all edges of invaded obstacle
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

        // Re-add region-facing arc
        let mut gate_indices: Vec<usize> = Vec::new();
        for (g, ids) in gate_obstacle_ids.iter().enumerate() {
            for side in 0..2 {
                if ids[side] == invaded_id {
                    if let Some(idx) = find_point_in_contour(invaded, gates[g][side]) {
                        if !gate_indices.contains(&idx) {
                            gate_indices.push(idx);
                        }
                    }
                }
            }
        }

        if !gate_indices.is_empty() {
            let idx_a = gate_indices[0];
            let idx_b = if gate_indices.len() >= 2 {
                gate_indices[1]
            } else {
                (idx_a + invaded.points.len() / 2) % invaded.points.len()
            };

            let arc_fwd = extract_arc(invaded, idx_a, idx_b);
            let arc_rev = extract_arc(invaded, idx_b, idx_a);
            let mid_fwd = arc_fwd[arc_fwd.len() / 2];
            let mid_rev = arc_rev[arc_rev.len() / 2];
            let d_fwd = mid_fwd.dist_sq(&center);
            let d_rev = mid_rev.dist_sq(&center);
            let chosen = if d_fwd < d_rev { &arc_fwd } else { &arc_rev };
            for j in 0..chosen.len() - 1 {
                add_bidirectional_edge(&mut adj_graph, chosen[j], chosen[j + 1]);
            }
        }

        eprintln!(
            "  Invasion of O{} detected in R{}, replacing with region-facing arc (attempt {})",
            invaded_id,
            region.id,
            attempt + 1
        );
    }

    None
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
