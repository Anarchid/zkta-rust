use crate::det::HashMap;

/// Integer 2D point (pixel coordinates).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    pub fn dist_sq(&self, other: &Point) -> f64 {
        let dx = (self.x - other.x) as f64;
        let dy = (self.y - other.y) as f64;
        dx * dx + dy * dy
    }

    pub fn dist(&self, other: &Point) -> f64 {
        self.dist_sq(other).sqrt()
    }
}

/// Perpendicular distance from point `p` to line segment `(a, b)`.
pub fn distance_to_segment(p: Point, a: Point, b: Point) -> f64 {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    if dx == 0.0 && dy == 0.0 {
        return p.dist(&a);
    }
    let t = ((p.x - a.x) as f64 * dx + (p.y - a.y) as f64 * dy) / (dx * dx + dy * dy);
    let t = t.clamp(0.0, 1.0);
    let cx = a.x as f64 + t * dx;
    let cy = a.y as f64 + t * dy;
    ((p.x as f64 - cx).powi(2) + (p.y as f64 - cy).powi(2)).sqrt()
}

/// Closest point on line segment `(a, b)` to point `p`.
pub fn closest_point_on_segment(p: Point, a: Point, b: Point) -> Point {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    if dx == 0.0 && dy == 0.0 {
        return a;
    }
    let t = ((p.x - a.x) as f64 * dx + (p.y - a.y) as f64 * dy) / (dx * dx + dy * dy);
    let t = t.clamp(0.0, 1.0);
    Point::new(
        (a.x as f64 + t * dx).round() as i32,
        (a.y as f64 + t * dy).round() as i32,
    )
}

/// Cross product of vectors (o->a) and (o->b).
pub fn cross(o: Point, a: Point, b: Point) -> f64 {
    (a.x - o.x) as f64 * (b.y - o.y) as f64 - (a.y - o.y) as f64 * (b.x - o.x) as f64
}

/// Test if segments (a1,a2) and (b1,b2) properly intersect (strict crossing).
pub fn segments_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> bool {
    let d1 = cross(b1, b2, a1);
    let d2 = cross(b1, b2, a2);
    let d3 = cross(a1, a2, b1);
    let d4 = cross(a1, a2, b2);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Ray-casting point-in-polygon test.
pub fn point_in_polygon(p: Point, polygon: &[Point]) -> bool {
    let mut inside = false;
    let n = polygon.len();
    let mut j = n - 1;
    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];
        if (pi.y > p.y) != (pj.y > p.y)
            && (p.x as i64)
                < (pj.x as i64 - pi.x as i64) * (p.y as i64 - pi.y as i64)
                    / (pj.y as i64 - pi.y as i64)
                    + pi.x as i64
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Snap `value` to `attractor` if within `tolerance`.
pub fn snap(value: i32, attractor: i32, tolerance: i32) -> i32 {
    if (value - attractor).abs() < tolerance {
        attractor
    } else {
        value
    }
}

/// Dijkstra shortest path on a point-based adjacency graph.
/// `prohibited_gate`: if set, the direct edge between those two points is forbidden.
pub fn dijkstra(
    graph: &HashMap<Point, Vec<Point>>,
    start: Point,
    goal: Point,
    prohibited_gate: Option<[Point; 2]>,
) -> Option<Vec<Point>> {
    use std::cmp::Ordering;
    use std::collections::BinaryHeap;

    #[derive(PartialEq)]
    struct State {
        cost: f64,
        point: Point,
    }
    impl Eq for State {}
    impl PartialOrd for State {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for State {
        fn cmp(&self, other: &Self) -> Ordering {
            other
                .cost
                .partial_cmp(&self.cost)
                .unwrap_or(Ordering::Equal)
        }
    }

    let mut dist: HashMap<Point, f64> = HashMap::default();
    let mut prev: HashMap<Point, Point> = HashMap::default();
    let mut heap = BinaryHeap::new();

    dist.insert(start, 0.0);
    heap.push(State {
        cost: 0.0,
        point: start,
    });

    while let Some(State { cost, point }) = heap.pop() {
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
            return Some(path);
        }

        if cost > *dist.get(&point).unwrap_or(&f64::MAX) {
            continue;
        }

        if let Some(neighbors) = graph.get(&point) {
            for &neighbor in neighbors {
                // Check prohibited gate
                if let Some(gate) = prohibited_gate {
                    if (point == gate[0] && neighbor == gate[1])
                        || (point == gate[1] && neighbor == gate[0])
                    {
                        continue;
                    }
                }

                let edge_dist = point.dist(&neighbor);
                let new_dist = cost + edge_dist;
                if new_dist < *dist.get(&neighbor).unwrap_or(&f64::MAX) {
                    dist.insert(neighbor, new_dist);
                    prev.insert(neighbor, point);
                    heap.push(State {
                        cost: new_dist,
                        point: neighbor,
                    });
                }
            }
        }
    }

    None
}

/// Add a bidirectional edge to a point-based adjacency graph.
pub fn add_bidirectional_edge(graph: &mut HashMap<Point, Vec<Point>>, a: Point, b: Point) {
    graph.entry(a).or_default().push(b);
    graph.entry(b).or_default().push(a);
}
