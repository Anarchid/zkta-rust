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

/// Callback for rendering intermediate stages.
pub trait StageRenderer {
    fn on_contours_traced(&self, _full: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
    fn on_contours_simplified(&self, _simplified: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
    fn on_voronoi_built(&self, _edges: &[(Point, Point)], _simplified: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
    fn on_graph_built(&self, _graph: &RegionGraph, _voronoi_edges: &[(Point, Point)], _simplified: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
    fn on_graph_pruned(&self, _graph: &RegionGraph, _simplified: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
    fn on_classified(&self, _graph: &RegionGraph, _simplified: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
    fn on_contracted(&self, _graph: &RegionGraph, _simplified: &[Contour], _label_map: &[Vec<i32>], _w: u32, _h: u32) {}
}

/// No-op renderer (no diagnostics).
pub struct NoopRenderer;
impl StageRenderer for NoopRenderer {}

/// Run terrain analysis from a boolean obstacle bitmap.
///
/// Each cell in `obstacle_map` is `true` for obstacle, `false` for passable.
/// Layout: row-major, `obstacle_map[y * width + x]`.
pub fn analyze_bitmap(
    obstacle_map: &[bool],
    width: u32,
    height: u32,
    renderer: &dyn StageRenderer,
) -> AnalysisResult {
    eprintln!("  [0/13] Tracing contours on {}x{} bitmap...", width, height);
    let pixel_test = |x: u32, y: u32| -> bool {
        obstacle_map[(y * width + x) as usize]
    };
    let (full_contours, holes, label_map) = contour::trace_contours(width, height, &pixel_test);
    eprintln!("  [0/13] Contour tracing done");
    analyze_inner(full_contours, holes, label_map, width, height, renderer)
}

/// Run the full terrain analysis pipeline from an RGBA image.
pub fn analyze(
    img: &RgbaImage,
    is_obstacle: &dyn Fn(image::Rgba<u8>) -> bool,
    renderer: &dyn StageRenderer,
) -> AnalysisResult {
    let width = img.width();
    let height = img.height();

    let pixel_test = |x: u32, y: u32| -> bool { is_obstacle(*img.get_pixel(x, y)) };
    let (full_contours, holes, label_map) = contour::trace_contours(width, height, &pixel_test);
    analyze_inner(full_contours, holes, label_map, width, height, renderer)
}

/// Shared analysis pipeline — from traced contours through to final regions.
fn analyze_inner(
    full_contours: Vec<Contour>,
    holes: Vec<LabeledContour>,
    mut label_map: Vec<Vec<i32>>,
    width: u32,
    height: u32,
    renderer: &dyn StageRenderer,
) -> AnalysisResult {
    let total_pts: usize = full_contours.iter().map(|c| c.len()).sum();
    eprintln!("  [1/13] Traced {} contours ({} points)", full_contours.len(), total_pts);
    renderer.on_contours_traced(&full_contours, &label_map, width, height);

    // Stage 2: Simplify contours (outer boundaries + hole inner boundaries)
    let mut simplified_contours = contour::simplify_contours(&full_contours, 8.0);
    let outer_count = simplified_contours.len();
    let simplified_holes = contour::simplify_holes(&holes, 8.0);
    let hole_count = simplified_holes.len();
    simplified_contours.extend(simplified_holes);
    let simp_pts: usize = simplified_contours.iter().map(|c| c.len()).sum();
    eprintln!("  [2/13] Simplified to {} contours ({} outer + {} holes, {} points)",
        simplified_contours.len(), outer_count, hole_count, simp_pts);

    // Stage 2b: Paint border pixels in labelMap so graph builder rejects vertices
    // on the map edge. Don't add a separate border contour — border sub-segments are
    // added directly to the Voronoi input (split at obstacle contact points).
    let border_label = simplified_contours.len() as i32 + 1;
    let w = width as i32;
    let h = height as i32;
    for x in 0..w {
        label_map[0][x as usize] = border_label;
        label_map[(h - 1) as usize][x as usize] = border_label;
    }
    for y in 0..h {
        label_map[y as usize][0] = border_label;
        label_map[y as usize][(w - 1) as usize] = border_label;
    }
    renderer.on_contours_simplified(&simplified_contours, &label_map, width, height);

    // Stage 3: Build R-tree (border contour is now part of simplified_contours)
    eprintln!("  [3/13] Building R-tree...");
    let border_points = voronoi::get_border_points(&simplified_contours, w, h);
    let rtree = ObstacleRTree::build(&simplified_contours)
        .with_border_segments(w, h, &border_points);

    // Stage 4: Generate Voronoi + medial axis filter
    eprintln!("  [4/13] Generating Voronoi diagram...");
    let voronoi_edges = voronoi::build_voronoi(&simplified_contours, width as i32, height as i32);
    eprintln!("  [4/13] Voronoi: {} edges", voronoi_edges.len());
    renderer.on_voronoi_built(&voronoi_edges, &simplified_contours, &label_map, width, height);

    // Stage 5: Build node graph (prune vertices inside obstacles / out of bounds)
    eprintln!("  [5/13] Building node graph...");
    let mut graph = graph::build_graph(&voronoi_edges, &label_map, width as i32, height as i32);
    eprintln!("  [5/13] Graph: {} nodes", graph.nodes.len());
    renderer.on_graph_built(&graph, &voronoi_edges, &simplified_contours, &label_map, width, height);

    // Stage 6: Prune leaf nodes
    eprintln!("  [6/13] Pruning leaves...");
    graph::prune_leaves(&mut graph, &rtree, TOO_CLOSE_TO_OBSTACLE);
    eprintln!("  [6/13] After pruning: {} nodes", graph.nodes.len());
    renderer.on_graph_pruned(&graph, &simplified_contours, &label_map, width, height);

    // Stage 7: Calculate radii
    eprintln!("  [7/13] Calculating radii...");
    graph::calculate_radii(&mut graph, &rtree);

    // Stage 8: Classify nodes (BWTA2)
    eprintln!("  [8/13] Classifying nodes...");
    graph::classify_nodes(&mut graph);
    renderer.on_classified(&graph, &simplified_contours, &label_map, width, height);

    // Stage 9: Contract graph
    eprintln!("  [9/13] Contracting graph...");
    graph::contract_graph(&mut graph);
    renderer.on_contracted(&graph, &simplified_contours, &label_map, width, height);

    // Stage 10: Compute chokepoint gates
    eprintln!("  [10/13] Computing chokepoints...");
    let (mut regions, mut chokepoints) = region::compute_chokepoint_gates(&graph, &rtree, w, h);

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
