use rstar::{RTree, RTreeObject, PointDistance, AABB};
use crate::geometry::{Point, distance_to_segment};
use crate::contour::Contour;

/// A line segment stored in the R-tree, tagged with an obstacle ID.
#[derive(Debug, Clone)]
pub struct ObstacleSegment {
    pub p1: [f64; 2],
    pub p2: [f64; 2],
    pub obstacle_id: usize,
}

impl RTreeObject for ObstacleSegment {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [self.p1[0].min(self.p2[0]), self.p1[1].min(self.p2[1])],
            [self.p1[0].max(self.p2[0]), self.p1[1].max(self.p2[1])],
        )
    }
}

impl PointDistance for ObstacleSegment {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let p = Point::new(point[0].round() as i32, point[1].round() as i32);
        let a = Point::new(self.p1[0].round() as i32, self.p1[1].round() as i32);
        let b = Point::new(self.p2[0].round() as i32, self.p2[1].round() as i32);
        let d = distance_to_segment(p, a, b);
        d * d
    }
}

/// Wrapper around rstar::RTree for obstacle distance queries.
pub struct ObstacleRTree {
    tree: RTree<ObstacleSegment>,
}

impl ObstacleRTree {
    /// Build the R-tree from simplified contours.
    pub fn build(contours: &[Contour]) -> Self {
        let mut segments = Vec::new();
        for (i, c) in contours.iter().enumerate() {
            let obstacle_id = i + 1; // 1-based
            for j in 1..c.points.len() {
                let p1 = c.points[j - 1];
                let p2 = c.points[j];
                segments.push(ObstacleSegment {
                    p1: [p1.x as f64, p1.y as f64],
                    p2: [p2.x as f64, p2.y as f64],
                    obstacle_id,
                });
            }
            // Closing edge
            if c.points.len() > 2 {
                let last = c.points[c.points.len() - 1];
                let first = c.points[0];
                if last != first {
                    segments.push(ObstacleSegment {
                        p1: [last.x as f64, last.y as f64],
                        p2: [first.x as f64, first.y as f64],
                        obstacle_id,
                    });
                }
            }
        }

        log::debug!("Built R-tree with {} segments", segments.len());
        Self {
            tree: RTree::bulk_load(segments),
        }
    }

    /// Add border segments (obstacle_id = 0).
    pub fn with_border_segments(mut self, width: i32, height: i32, border_points: &[Vec<Point>]) -> Self {
        // Add border segments from the border point arrays
        for border in border_points {
            for i in 1..border.len() {
                let p1 = border[i - 1];
                let p2 = border[i];
                self.tree.insert(ObstacleSegment {
                    p1: [p1.x as f64, p1.y as f64],
                    p2: [p2.x as f64, p2.y as f64],
                    obstacle_id: 0,
                });
            }
        }
        // Also add the 4 map edges as continuous border segments
        let corners = [
            Point::new(1, 1),
            Point::new(width - 1, 1),
            Point::new(width - 1, height - 1),
            Point::new(1, height - 1),
        ];
        for i in 0..4 {
            let p1 = corners[i];
            let p2 = corners[(i + 1) % 4];
            self.tree.insert(ObstacleSegment {
                p1: [p1.x as f64, p1.y as f64],
                p2: [p2.x as f64, p2.y as f64],
                obstacle_id: 0,
            });
        }
        self
    }

    /// Distance from point to nearest obstacle segment.
    pub fn nearest_distance(&self, p: Point) -> f64 {
        let query = [p.x as f64, p.y as f64];
        match self.tree.nearest_neighbor(&query) {
            Some(seg) => seg.distance_2(&query).sqrt(),
            None => f64::MAX,
        }
    }

    /// Get N nearest segments to a point, returning (obstacle_id, segment, distance).
    pub fn nearest_n(&self, p: Point, n: usize) -> Vec<(usize, ObstacleSegment, f64)> {
        let query = [p.x as f64, p.y as f64];
        self.tree
            .nearest_neighbor_iter(&query)
            .take(n)
            .map(|seg| {
                let d = seg.distance_2(&query).sqrt();
                (seg.obstacle_id, seg.clone(), d)
            })
            .collect()
    }

    /// Get the nearest segment from a different obstacle than `exclude_id`.
    pub fn nearest_from_different_obstacle(
        &self,
        p: Point,
        exclude_id: usize,
        max_count: usize,
    ) -> Option<(usize, ObstacleSegment, f64)> {
        let query = [p.x as f64, p.y as f64];
        self.tree
            .nearest_neighbor_iter(&query)
            .take(max_count)
            .find(|seg| seg.obstacle_id != exclude_id && seg.obstacle_id != 0)
            .map(|seg| {
                let d = seg.distance_2(&query).sqrt();
                (seg.obstacle_id, seg.clone(), d)
            })
    }
}
