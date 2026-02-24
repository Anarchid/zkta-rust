mod analyzer;
mod contour;
mod geometry;
mod graph;
mod region;
mod render;
mod rtree;
mod voronoi;

use std::fs;
use std::path::PathBuf;

use image::Rgba;
use serde::Serialize;

use analyzer::AnalysisResult;
use geometry::Point;
use render::DiagnosticRenderer;

// ---- JSON output structures ----

#[derive(Serialize)]
struct JsonOutput {
    regions: Vec<JsonRegion>,
    chokepoints: Vec<JsonChokepoint>,
    #[serde(rename = "obstacleContours")]
    obstacle_contours: Vec<JsonContour>,
}

#[derive(Serialize)]
struct JsonRegion {
    id: usize,
    center: JsonPoint,
    radius: f64,
    #[serde(rename = "isClosed")]
    is_closed: bool,
    polygon: Vec<JsonPoint>,
    chokepoints: Vec<usize>,
}

#[derive(Serialize)]
struct JsonChokepoint {
    id: usize,
    position: JsonPoint,
    radius: f64,
    #[serde(rename = "crossingPoints")]
    crossing_points: Vec<JsonPoint>,
    #[serde(rename = "obstacleIds")]
    obstacle_ids: Vec<usize>,
    regions: Vec<usize>,
}

#[derive(Serialize)]
struct JsonContour {
    #[serde(rename = "obstacleId")]
    obstacle_id: usize,
    points: Vec<JsonPoint>,
    #[serde(rename = "pointCount")]
    point_count: usize,
}

#[derive(Serialize)]
struct JsonPoint {
    x: i32,
    y: i32,
}

fn export_to_json(result: &AnalysisResult, output_path: &str) {
    let mut json_regions = Vec::new();
    for r in &result.regions {
        let cp_ids: Vec<usize> = result
            .chokepoints
            .iter()
            .filter(|cp| r.chokepoint_node_ids.contains(&cp.node_id))
            .map(|cp| cp.id)
            .collect();

        json_regions.push(JsonRegion {
            id: r.id,
            center: JsonPoint {
                x: r.center.x,
                y: r.center.y,
            },
            radius: r.radius,
            is_closed: r.is_closed,
            polygon: r.polygon.iter().map(|p| JsonPoint { x: p.x, y: p.y }).collect(),
            chokepoints: cp_ids,
        });
    }

    let mut json_chokepoints = Vec::new();
    for cp in &result.chokepoints {
        let crossing = cp
            .crossing_points
            .map(|cps| {
                vec![
                    JsonPoint { x: cps[0].x, y: cps[0].y },
                    JsonPoint { x: cps[1].x, y: cps[1].y },
                ]
            })
            .unwrap_or_default();

        let obstacle_ids = cp
            .obstacle_ids
            .map(|ids| vec![ids[0], ids[1]])
            .unwrap_or_default();

        let region_ids: Vec<usize> = cp.region_ids.iter().copied().collect();

        json_chokepoints.push(JsonChokepoint {
            id: cp.id,
            position: JsonPoint { x: cp.position.x, y: cp.position.y },
            radius: cp.radius,
            crossing_points: crossing,
            obstacle_ids,
            regions: region_ids,
        });
    }

    let mut json_contours = Vec::new();
    for (i, c) in result.simplified_contours.iter().enumerate() {
        let points: Vec<JsonPoint> = c.points.iter().map(|p| JsonPoint { x: p.x, y: p.y }).collect();
        json_contours.push(JsonContour {
            obstacle_id: i + 1,
            point_count: points.len(),
            points,
        });
    }

    let output = JsonOutput {
        regions: json_regions,
        chokepoints: json_chokepoints,
        obstacle_contours: json_contours,
    };

    match serde_json::to_string_pretty(&output) {
        Ok(json) => {
            if let Err(e) = fs::write(output_path, json) {
                eprintln!("Failed to write {}: {}", output_path, e);
            } else {
                eprintln!("Exported data to {}", output_path);
            }
        }
        Err(e) => eprintln!("JSON serialization failed: {}", e),
    }
}

// ---- Final result rendering ----

fn render_final_result(result: &AnalysisResult, output_path: &str) {
    let (w, h) = (result.width, result.height);
    let mut img = image::RgbImage::new(w, h);

    for pixel in img.pixels_mut() {
        *pixel = image::Rgb([0, 0, 0]);
    }

    // Draw label map in dark gray
    for y in 0..h {
        for x in 0..w {
            if result.label_map[y as usize][x as usize] != 0 {
                img.put_pixel(x, y, image::Rgb([50, 50, 50]));
            }
        }
    }

    // Region boundary fills (bottom layer, drawn first so everything else goes on top)
    let mut rng_state = 42u64;
    let mut region_colors: Vec<([u8; 3], [u8; 3])> = Vec::new();
    for r in &result.regions {
        let hue = pseudo_random(&mut rng_state);
        let color = hsv_to_rgb(hue, 0.8, 0.9);
        let fill_color = [color[0] / 4, color[1] / 4, color[2] / 4];
        region_colors.push((color, fill_color));
        if r.is_closed && r.polygon.len() >= 3 {
            render::draw_filled_polygon(&mut img, &r.polygon, fill_color);
        }
    }

    // Region boundary outlines
    for (i, r) in result.regions.iter().enumerate() {
        if r.is_closed && r.polygon.len() >= 3 {
            let color = region_colors[i].0;
            for j in 0..r.polygon.len() {
                let k = (j + 1) % r.polygon.len();
                render::draw_line(&mut img, r.polygon[j], r.polygon[k], color);
            }
        }
    }

    // Full contours in green
    for contour in &result.full_contours {
        for i in 0..contour.len() {
            let j = (i + 1) % contour.len();
            render::draw_line(&mut img, contour.points[i], contour.points[j], [0, 180, 0]);
        }
    }

    // Graph edges in light gray
    for (&id, neighbors) in &result.graph.adjacency {
        if let Some(node) = result.graph.nodes.get(&id) {
            for &nid in neighbors {
                if nid > id {
                    if let Some(neighbor) = result.graph.nodes.get(&nid) {
                        render::draw_line(&mut img, node.position, neighbor.position, [180, 180, 180]);
                    }
                }
            }
        }
    }

    // Simplified contours in yellow + obstacle labels
    for (idx, contour) in result.simplified_contours.iter().enumerate() {
        for i in 0..contour.len() {
            let j = (i + 1) % contour.len();
            render::draw_line(&mut img, contour.points[i], contour.points[j], [255, 255, 0]);
        }
        if !contour.points.is_empty() {
            let cx = contour.points.iter().map(|p| p.x as f64).sum::<f64>() / contour.points.len() as f64;
            let cy = contour.points.iter().map(|p| p.y as f64).sum::<f64>() / contour.points.len() as f64;
            let centroid = geometry::Point::new(cx.round() as i32, cy.round() as i32);
            let label = format!("O{}", idx + 1);
            render::draw_label_centered(&mut img, centroid, &label, [255, 255, 100]);
        }
    }

    // Chokepoint gate lines + markers + labels
    for cp in &result.chokepoints {
        if let Some(cps) = cp.crossing_points {
            if cps[0] != cps[1] {
                render::draw_line(&mut img, cps[0], cps[1], [255, 0, 0]);
            }
        }
        render::draw_filled_circle(&mut img, cp.position, 4, [255, 0, 0]);
        let label = format!("C{}", cp.id);
        render::draw_label(&mut img, cp.position, &label, [255, 100, 100]);
    }

    // Region seed nodes + labels (topmost layer)
    for r in &result.regions {
        render::draw_filled_circle(&mut img, r.center, 4, [64, 224, 208]);
        let label = format!("R{}", r.id);
        render::draw_label(&mut img, r.center, &label, [64, 224, 208]);
    }

    if let Err(e) = img.save(output_path) {
        eprintln!("Failed to save {}: {}", output_path, e);
    } else {
        eprintln!("Saved {}", output_path);
    }
}

fn pseudo_random(state: &mut u64) -> f32 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    ((*state >> 33) as f32) / (u32::MAX as f32)
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> [u8; 3] {
    let c = v * s;
    let x = c * (1.0 - ((h * 6.0) % 2.0 - 1.0).abs());
    let m = v - c;
    let (r, g, b) = match (h * 6.0) as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    [((r + m) * 255.0) as u8, ((g + m) * 255.0) as u8, ((b + m) * 255.0) as u8]
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input_path = args
        .iter()
        .find(|a| !a.starts_with('-') && *a != &args[0])
        .map(|s| s.as_str())
        .unwrap_or("trojan.png");

    let headless = args.iter().any(|a| a == "--headless" || a == "-d");
    if headless {
        eprintln!("Running in headless mode");
    }

    // Create results directory
    fs::create_dir_all("results").ok();

    // Load image
    let img = match image::open(input_path) {
        Ok(img) => img.to_rgba8(),
        Err(e) => {
            eprintln!("Failed to load {}: {}", input_path, e);
            return;
        }
    };

    let bot_obstacle = |pixel: Rgba<u8>| -> bool { pixel[0] > 0 };
    let vehicle_obstacle = |pixel: Rgba<u8>| -> bool { pixel[0] > 0 || pixel[1] > 0 };

    // Bot tier analysis with diagnostic rendering
    eprintln!("=== Bot tier analysis ===");
    let bot_renderer = DiagnosticRenderer::new(PathBuf::from("results/bot"));
    let bot_result = analyzer::analyze(&img, &bot_obstacle, &bot_renderer);
    render_final_result(&bot_result, "results/bot/08_final.png");
    export_to_json(&bot_result, "results/bot/result.json");

    // Vehicle tier analysis with diagnostic rendering
    eprintln!("=== Vehicle tier analysis ===");
    let vehicle_renderer = DiagnosticRenderer::new(PathBuf::from("results/vehicle"));
    let vehicle_result = analyzer::analyze(&img, &vehicle_obstacle, &vehicle_renderer);
    render_final_result(&vehicle_result, "results/vehicle/08_final.png");
    export_to_json(&vehicle_result, "results/vehicle/result.json");

    eprintln!("Analysis complete. Results in results/bot/ and results/vehicle/");
}
