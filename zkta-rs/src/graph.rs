use std::collections::VecDeque;

use crate::det::{HashMap, HashSet};

use crate::geometry::Point;
use crate::rtree::ObstacleRTree;

/// A node in the medial axis graph.
#[derive(Debug, Clone)]
pub struct Node {
    pub position: Point,
    pub radius: f64,
    pub visited: bool,
    pub parent: Option<usize>,
}

/// The region graph: nodes indexed by ID, with adjacency and classification sets.
pub struct RegionGraph {
    pub nodes: HashMap<usize, Node>,
    pub adjacency: HashMap<usize, HashSet<usize>>,
    pub region_nodes: HashSet<usize>,
    pub chokepoint_nodes: HashSet<usize>,
    next_id: usize,
}

impl RegionGraph {
    pub fn new() -> Self {
        Self {
            nodes: HashMap::default(),
            adjacency: HashMap::default(),
            region_nodes: HashSet::default(),
            chokepoint_nodes: HashSet::default(),
            next_id: 0,
        }
    }

    pub fn add_node(&mut self, position: Point) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        self.nodes.insert(
            id,
            Node {
                position,
                radius: 0.0,
                visited: false,
                parent: None,
            },
        );
        self.adjacency.insert(id, HashSet::default());
        id
    }

    pub fn add_node_with_id(&mut self, id: usize, position: Point) {
        self.nodes.insert(
            id,
            Node {
                position,
                radius: 0.0,
                visited: false,
                parent: None,
            },
        );
        self.adjacency.entry(id).or_default();
        if id >= self.next_id {
            self.next_id = id + 1;
        }
    }

    pub fn add_edge(&mut self, a: usize, b: usize) {
        if a == b {
            return;
        }
        self.adjacency.entry(a).or_default().insert(b);
        self.adjacency.entry(b).or_default().insert(a);
    }

    pub fn remove_node(&mut self, id: usize) {
        if let Some(neighbors) = self.adjacency.remove(&id) {
            for n in &neighbors {
                if let Some(adj) = self.adjacency.get_mut(n) {
                    adj.remove(&id);
                }
            }
        }
        self.nodes.remove(&id);
        self.region_nodes.remove(&id);
        self.chokepoint_nodes.remove(&id);
    }

    pub fn degree(&self, id: usize) -> usize {
        self.adjacency.get(&id).map_or(0, |s| s.len())
    }

    pub fn neighbors(&self, id: usize) -> Vec<usize> {
        self.adjacency
            .get(&id)
            .map(|s| s.iter().copied().collect())
            .unwrap_or_default()
    }

    pub fn set_region(&mut self, id: usize) {
        self.region_nodes.insert(id);
        self.chokepoint_nodes.remove(&id);
    }

    pub fn set_chokepoint(&mut self, id: usize) {
        self.chokepoint_nodes.insert(id);
        self.region_nodes.remove(&id);
    }

    pub fn node_ids(&self) -> Vec<usize> {
        self.nodes.keys().copied().collect()
    }

    pub fn allocate_id(&mut self) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
}

/// Build the node graph from Voronoi edges, pruning vertices inside obstacles
/// or outside image bounds.
pub fn build_graph(
    edges: &[(Point, Point)],
    label_map: &[Vec<i32>],
    width: i32,
    height: i32,
) -> RegionGraph {
    let mut graph = RegionGraph::new();
    let mut point_to_id: HashMap<Point, usize> = HashMap::default();

    let get_or_create = |graph: &mut RegionGraph,
                         point_to_id: &mut HashMap<Point, usize>,
                         p: Point|
     -> usize {
        if let Some(&id) = point_to_id.get(&p) {
            return id;
        }
        let id = graph.add_node(p);
        point_to_id.insert(p, id);
        id
    };

    for &(p1, p2) in edges {
        // Skip vertices outside bounds (with 5px snap tolerance)
        let c1x = crate::geometry::snap(crate::geometry::snap(p1.x, 0, 5), width, 5);
        let c1y = crate::geometry::snap(crate::geometry::snap(p1.y, 0, 5), height, 5);
        let c2x = crate::geometry::snap(crate::geometry::snap(p2.x, 0, 5), width, 5);
        let c2y = crate::geometry::snap(crate::geometry::snap(p2.y, 0, 5), height, 5);

        if c1x < 0 || c1x >= width || c1y < 0 || c1y >= height {
            continue;
        }
        if c2x < 0 || c2x >= width || c2y < 0 || c2y >= height {
            continue;
        }
        if label_map[c1y as usize][c1x as usize] > 0 {
            continue;
        }
        if label_map[c2y as usize][c2x as usize] > 0 {
            continue;
        }

        let id1 = get_or_create(&mut graph, &mut point_to_id, p1);
        let id2 = get_or_create(&mut graph, &mut point_to_id, p2);
        graph.add_edge(id1, id2);
    }

    log::debug!("Built graph with {} nodes", graph.nodes.len());
    graph
}

/// Remove leaf nodes (degree 1) that are too close to obstacles.
pub fn prune_leaves(graph: &mut RegionGraph, rtree: &ObstacleRTree, threshold: f64) {
    let mut to_prune: VecDeque<usize> = VecDeque::new();
    for &id in graph.nodes.keys() {
        if graph.degree(id) == 1 {
            to_prune.push_back(id);
        }
    }

    let mut removed = 0;
    while let Some(id) = to_prune.pop_front() {
        if !graph.nodes.contains_key(&id) {
            continue;
        }
        let neighbors = graph.neighbors(id);
        if neighbors.is_empty() {
            continue;
        }
        let n1 = neighbors[0];

        let pos = graph.nodes[&id].position;
        let dist = rtree.nearest_distance(pos);

        if dist < threshold {
            removed += 1;
            graph.remove_node(id);
            if graph.degree(n1) == 1 {
                to_prune.push_back(n1);
            }
        }
    }
    log::debug!(
        "Pruned {} leaf nodes, {} remaining",
        removed,
        graph.nodes.len()
    );
}

/// Calculate inscribed circle radii for all nodes.
pub fn calculate_radii(graph: &mut RegionGraph, rtree: &ObstacleRTree) {
    let ids: Vec<usize> = graph.nodes.keys().copied().collect();
    for id in ids {
        let pos = graph.nodes[&id].position;
        let r = rtree.nearest_distance(pos);
        graph.nodes.get_mut(&id).unwrap().radius = r;
    }
}

/// BWTA2-style node classification: region (local max) or chokepoint (local min).
pub fn classify_nodes(graph: &mut RegionGraph) {
    // Reset traversal state
    for node in graph.nodes.values_mut() {
        node.visited = false;
        node.parent = None;
    }

    let mut to_process: VecDeque<usize> = VecDeque::new();

    // Start from leaf nodes
    let ids: Vec<usize> = graph.node_ids();
    for &id in &ids {
        if graph.degree(id) == 1 {
            to_process.push_back(id);
            graph.set_region(id);
            graph.nodes.get_mut(&id).unwrap().visited = true;
        }
    }

    // Fallback for leafless graphs (e.g. cyclic medial axes in ring-shaped
    // passable areas like ship tier water).  Seed from unvisited nodes with
    // the largest radius until all components are covered.
    if to_process.is_empty() {
        if let Some(&seed) = ids.iter()
            .filter(|id| !graph.nodes[id].visited)
            .max_by(|a, b| graph.nodes[a].radius.partial_cmp(&graph.nodes[b].radius).unwrap())
        {
            to_process.push_back(seed);
            graph.set_region(seed);
            graph.nodes.get_mut(&seed).unwrap().visited = true;
        }
    }

    while let Some(current) = to_process.pop_front() {
        let neighbors = graph.neighbors(current);
        for nid in neighbors {
            if graph.nodes.get(&nid).map_or(true, |n| n.visited) {
                continue;
            }
            graph.nodes.get_mut(&nid).unwrap().visited = true;
            graph.nodes.get_mut(&nid).unwrap().parent = Some(current);
            to_process.push_back(nid);

            let degree = graph.degree(nid);

            if degree != 2 {
                graph.set_region(nid);
            } else {
                let is_local_min = is_local_minimum(graph, nid);
                let is_local_max = is_local_maximum(graph, nid);

                if is_local_min {
                    let current_is_local_min = is_local_minimum(graph, current);
                    if current_is_local_min {
                        let nr = graph.nodes[&nid].radius;
                        let cr = graph.nodes[&current].radius;
                        if nr < cr {
                            graph.set_chokepoint(nid);
                        }
                    } else {
                        graph.set_chokepoint(nid);
                    }
                } else if is_local_max {
                    let current_is_local_max = is_local_maximum(graph, current);
                    if current_is_local_max {
                        let nr = graph.nodes[&nid].radius;
                        let cr = graph.nodes[&current].radius;
                        if nr > cr {
                            graph.set_region(nid);
                        }
                    } else {
                        graph.set_region(nid);
                    }
                }
            }
        }
    }

    let unmarked = graph.nodes.len() - graph.region_nodes.len() - graph.chokepoint_nodes.len();
    log::debug!(
        "Classified: {} regions, {} chokepoints, {} unmarked",
        graph.region_nodes.len(),
        graph.chokepoint_nodes.len(),
        unmarked,
    );

    // Diagnostic: dump all classified nodes with neighbor IDs + radii
    if std::env::var("ZKTA_DUMP_CLASSIFY").is_ok() {
        let mut all_nodes: Vec<_> = graph.nodes.iter().collect();
        all_nodes.sort_by_key(|(&id, _)| id);
        for (&id, node) in &all_nodes {
            let deg = graph.degree(id);
            let class = if graph.region_nodes.contains(&id) {
                "REGION"
            } else if graph.chokepoint_nodes.contains(&id) {
                "CHOKE"
            } else {
                "unmarked"
            };
            let neighbor_info: Vec<String> = graph.adjacency.get(&id).unwrap_or(&HashSet::default())
                .iter()
                .filter_map(|&nid| graph.nodes.get(&nid).map(|n| (nid, n)))
                .map(|(nid, n)| format!("N{}:{:.1}", nid, n.radius))
                .collect();
            log::debug!(
                "  N{}: ({},{}) deg={} r={:.1} [{}] neighbors=[{}]",
                id, node.position.x, node.position.y, deg, node.radius, class,
                neighbor_info.join(", ")
            );
        }
    }
}

fn is_local_minimum(graph: &RegionGraph, id: usize) -> bool {
    let r = graph.nodes[&id].radius;
    for &nid in graph.adjacency.get(&id).unwrap_or(&HashSet::default()) {
        if let Some(n) = graph.nodes.get(&nid) {
            if n.radius < r {
                return false;
            }
        }
    }
    true
}

fn is_local_maximum(graph: &RegionGraph, id: usize) -> bool {
    let r = graph.nodes[&id].radius;
    for &nid in graph.adjacency.get(&id).unwrap_or(&HashSet::default()) {
        if let Some(n) = graph.nodes.get(&nid) {
            if n.radius > r {
                return false;
            }
        }
    }
    true
}

/// Contract graph: skip unmarked nodes, then merge contiguous same-type nodes.
pub fn contract_graph(graph: &mut RegionGraph) {
    // Phase 1: Skip unmarked vertices
    let marked: HashSet<usize> = graph
        .region_nodes
        .union(&graph.chokepoint_nodes)
        .copied()
        .collect();

    let mut new_neighbors: HashMap<usize, HashSet<usize>> = HashMap::default();
    for &m in &marked {
        new_neighbors.insert(m, HashSet::default());
    }

    for &m in &marked {
        let neighbors = graph.neighbors(m);
        let mut visited: HashSet<usize> = HashSet::default();
        visited.insert(m);

        let mut queue: VecDeque<usize> = VecDeque::new();

        for n in &neighbors {
            if marked.contains(n) {
                new_neighbors.get_mut(&m).unwrap().insert(*n);
            } else if visited.insert(*n) {
                queue.push_back(*n);
            }
        }

        while let Some(current) = queue.pop_front() {
            for n in graph.neighbors(current) {
                if !visited.insert(n) {
                    continue;
                }
                if marked.contains(&n) {
                    new_neighbors.get_mut(&m).unwrap().insert(n);
                } else {
                    queue.push_back(n);
                }
            }
        }
    }

    // Apply new adjacency and remove unmarked
    let to_remove: Vec<usize> = graph
        .nodes
        .keys()
        .copied()
        .filter(|id| !marked.contains(id))
        .collect();
    for id in to_remove {
        graph.nodes.remove(&id);
        graph.adjacency.remove(&id);
    }
    for (&m, neighbors) in &new_neighbors {
        graph.adjacency.insert(m, neighbors.clone());
    }

    // Phase 2: Merge contiguous same-type nodes
    merge_contiguous(graph);

    log::debug!(
        "After contraction: {} regions, {} chokepoints, {} total",
        graph.region_nodes.len(),
        graph.chokepoint_nodes.len(),
        graph.nodes.len(),
    );
}

fn merge_contiguous(graph: &mut RegionGraph) {
    let mut merged = true;
    while merged {
        merged = false;

        // Merge adjacent regions: keep larger radius, keep survivor's original position
        let region_ids: Vec<usize> = graph.region_nodes.iter().copied().collect();
        'region_merge: for &r in &region_ids {
            if !graph.region_nodes.contains(&r) {
                continue;
            }
            let neighbors = graph.neighbors(r);
            for n in neighbors {
                if graph.region_nodes.contains(&n) && n != r {
                    let r_radius = graph.nodes[&r].radius;
                    let n_radius = graph.nodes[&n].radius;
                    let (keep, remove) = if r_radius >= n_radius {
                        (r, n)
                    } else {
                        (n, r)
                    };

                    // Transfer neighbors from remove to keep
                    let remove_neighbors: Vec<usize> = graph.neighbors(remove);
                    for nn in remove_neighbors {
                        if nn != keep {
                            if let Some(adj) = graph.adjacency.get_mut(&nn) {
                                adj.remove(&remove);
                                adj.insert(keep);
                            }
                            graph.adjacency.entry(keep).or_default().insert(nn);
                        }
                    }
                    graph.adjacency.entry(keep).or_default().remove(&remove);
                    graph.region_nodes.remove(&remove);
                    graph.nodes.remove(&remove);
                    graph.adjacency.remove(&remove);
                    merged = true;
                    break 'region_merge;
                }
            }
        }
        if merged {
            continue;
        }

        // Merge adjacent chokepoints: keep smaller radius (narrowest)
        let cp_ids: Vec<usize> = graph.chokepoint_nodes.iter().copied().collect();
        'cp_merge: for &c in &cp_ids {
            if !graph.chokepoint_nodes.contains(&c) {
                continue;
            }
            let neighbors = graph.neighbors(c);
            for n in neighbors {
                if graph.chokepoint_nodes.contains(&n) && n != c {
                    let c_radius = graph.nodes[&c].radius;
                    let n_radius = graph.nodes[&n].radius;
                    let (keep, remove) = if c_radius <= n_radius {
                        (c, n)
                    } else {
                        (n, c)
                    };

                    let remove_neighbors: Vec<usize> = graph.neighbors(remove);
                    for nn in remove_neighbors {
                        if nn != keep {
                            if let Some(adj) = graph.adjacency.get_mut(&nn) {
                                adj.remove(&remove);
                                adj.insert(keep);
                            }
                            graph.adjacency.entry(keep).or_default().insert(nn);
                        }
                    }
                    graph.adjacency.entry(keep).or_default().remove(&remove);
                    graph.chokepoint_nodes.remove(&remove);
                    graph.nodes.remove(&remove);
                    graph.adjacency.remove(&remove);
                    merged = true;
                    break 'cp_merge;
                }
            }
        }
    }
}
