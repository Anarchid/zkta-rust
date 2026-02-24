use image::RgbaImage;

use crate::contour::{self, Contour, LabeledContour};
use crate::geometry::Point;
use crate::graph::{self, RegionGraph};
use crate::region::{self, Chokepoint, Region};
use crate::rtree::ObstacleRTree;
use crate::voronoi;

const TOO_CLOSE_TO_OBSTACLE: f64 = 15.0;

/// Result of running the terrain analysis pipeline.
pub struct AnalysisResult {
    pub regions: Vec<Region>,
    pub chokepoints: Vec<Chokepoint>,
    pub simplified_contours: Vec<Contour>,
    pub full_contours: Vec<Contour>,
    pub holes: Vec<LabeledContour>,
    pub label_map: Vec<Vec<i32>>,
    pub graph: RegionGraph,
    pub voronoi_edges: Vec<(Point, Point)>,
    pub width: u32,
    pub height: u32,
}

/// Run the full terrain analysis pipeline.
pub fn analyze(
    img: &RgbaImage,
    is_obstacle: &dyn Fn(image::Rgba<u8>) -> bool,
) -> AnalysisResult {
    let width = img.width();
    let height = img.height();

    // Stage 1: Trace contours
    let pixel_test = |x: u32, y: u32| -> bool { is_obstacle(*img.get_pixel(x, y)) };
    let (full_contours, holes, label_map) = contour::trace_contours(width, height, &pixel_test);

    // Stage 2: Simplify contours
    let mut simplified_contours = contour::simplify_contours(&full_contours, 8.0);

    // Stage 3: Build R-tree
    let border_points = voronoi::get_border_points(&simplified_contours, width as i32, height as i32);
    let rtree = ObstacleRTree::build(&simplified_contours)
        .with_border_segments(width as i32, height as i32, &border_points);

    // Stage 4: Generate Voronoi + medial axis filter
    let voronoi_edges = voronoi::build_voronoi(&simplified_contours, width as i32, height as i32);

    // Stage 5: Build node graph (prune vertices inside obstacles / out of bounds)
    let mut graph = graph::build_graph(&voronoi_edges, &label_map, width as i32, height as i32);

    // Stage 6: Prune leaf nodes
    graph::prune_leaves(&mut graph, &rtree, TOO_CLOSE_TO_OBSTACLE);

    // Stage 7: Calculate radii
    graph::calculate_radii(&mut graph, &rtree);

    // Stage 8: Classify nodes (BWTA2)
    graph::classify_nodes(&mut graph);

    // Stage 9: Contract graph
    graph::contract_graph(&mut graph);

    // Stage 10: Compute chokepoint gates
    let (mut regions, mut chokepoints) = region::compute_chokepoint_gates(&graph, &rtree);

    // Stage 10b: Insert synthetic chokepoints
    region::insert_synthetic_chokepoints(
        &mut graph,
        &mut regions,
        &mut chokepoints,
        &simplified_contours,
        &rtree,
    );

    // Stage 11: Split contours at gate points
    region::split_contours_at_gate_points(&mut simplified_contours, &chokepoints);

    // Stage 12: Construct region boundaries
    region::construct_region_boundaries(
        &mut regions,
        &chokepoints,
        &simplified_contours,
        width as i32,
        height as i32,
    );

    // Stage 13: Remove duplicate regions
    region::remove_duplicate_regions(&mut regions, &mut chokepoints);

    eprintln!(
        "Analysis complete: {} regions, {} chokepoints",
        regions.len(),
        chokepoints.len()
    );

    AnalysisResult {
        regions,
        chokepoints,
        simplified_contours,
        full_contours,
        holes,
        label_map,
        graph,
        voronoi_edges,
        width,
        height,
    }
}
