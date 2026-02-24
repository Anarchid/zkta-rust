mod analyzer;
mod contour;
mod geometry;
mod graph;
mod region;
mod rtree;
mod voronoi;

use std::fs;

use image::Rgba;
use serde::Serialize;

use analyzer::AnalysisResult;
use geometry::Point;

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
                    JsonPoint {
                        x: cps[0].x,
                        y: cps[0].y,
                    },
                    JsonPoint {
                        x: cps[1].x,
                        y: cps[1].y,
                    },
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
            position: JsonPoint {
                x: cp.position.x,
                y: cp.position.y,
            },
            radius: cp.radius,
            crossing_points: crossing,
            obstacle_ids,
            regions: region_ids,
        });
    }

    let mut json_contours = Vec::new();
    for (i, c) in result.simplified_contours.iter().enumerate() {
        let points: Vec<JsonPoint> = c
            .points
            .iter()
            .map(|p| JsonPoint { x: p.x, y: p.y })
            .collect();
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

// ---- Rendering ----

fn render_tier_result(result: &AnalysisResult, _base_img: &image::RgbaImage, output_path: &str) {
    let (w, h) = (result.width, result.height);
    let mut img = image::RgbImage::new(w, h);

    // Black background
    for pixel in img.pixels_mut() {
        *pixel = image::Rgb([0, 0, 0]);
    }

    // Draw label map (obstacle pixels) in dark gray
    for y in 0..h {
        for x in 0..w {
            if result.label_map[y as usize][x as usize] != 0 {
                img.put_pixel(x, y, image::Rgb([50, 50, 50]));
            }
        }
    }

    // Draw full contours in green
    for contour in &result.full_contours {
        for i in 0..contour.len() {
            let j = (i + 1) % contour.len();
            draw_line(
                &mut img,
                contour.points[i],
                contour.points[j],
                [0, 180, 0],
            );
        }
    }

    // Draw graph edges in light gray
    for (&id, neighbors) in &result.graph.adjacency {
        if let Some(node) = result.graph.nodes.get(&id) {
            for &nid in neighbors {
                if nid > id {
                    if let Some(neighbor) = result.graph.nodes.get(&nid) {
                        draw_line(
                            &mut img,
                            node.position,
                            neighbor.position,
                            [180, 180, 180],
                        );
                    }
                }
            }
        }
    }

    // Draw region seed nodes
    for r in &result.regions {
        draw_filled_circle(&mut img, r.center, 4, [64, 224, 208]);
    }

    // Draw chokepoint nodes
    for cp in &result.chokepoints {
        draw_filled_circle(&mut img, cp.position, 4, [255, 0, 0]);
    }

    // Draw chokepoint gate lines
    for cp in &result.chokepoints {
        if let Some(cps) = cp.crossing_points {
            if cps[0] != cps[1] {
                draw_line(&mut img, cps[0], cps[1], [255, 0, 0]);
            }
        }
    }

    // Draw simplified contours in yellow
    for contour in &result.simplified_contours {
        for i in 0..contour.len() {
            let j = (i + 1) % contour.len();
            draw_line(
                &mut img,
                contour.points[i],
                contour.points[j],
                [255, 255, 0],
            );
        }
    }

    // Draw region boundaries with random colors
    let mut rng_state = 42u64;
    for r in &result.regions {
        if r.is_closed && r.polygon.len() >= 3 {
            let hue = pseudo_random(&mut rng_state);
            let color = hsv_to_rgb(hue, 0.8, 0.9);

            // Draw filled polygon with dimmed color
            let fill_color = [color[0] / 4, color[1] / 4, color[2] / 4];
            draw_filled_polygon(&mut img, &r.polygon, fill_color);

            // Draw boundary outline
            for i in 0..r.polygon.len() {
                let j = (i + 1) % r.polygon.len();
                draw_line(&mut img, r.polygon[i], r.polygon[j], color);
            }
        }
    }

    if let Err(e) = img.save(output_path) {
        eprintln!("Failed to save {}: {}", output_path, e);
    } else {
        eprintln!("Saved {}", output_path);
    }
}

// ---- Simple drawing helpers ----

fn draw_line(img: &mut image::RgbImage, p1: Point, p2: Point, color: [u8; 3]) {
    let (w, h) = img.dimensions();
    // Bresenham's line algorithm
    let dx = (p2.x - p1.x).abs();
    let dy = -(p2.y - p1.y).abs();
    let sx = if p1.x < p2.x { 1 } else { -1 };
    let sy = if p1.y < p2.y { 1 } else { -1 };
    let mut err = dx + dy;
    let mut x = p1.x;
    let mut y = p1.y;

    loop {
        if x >= 0 && x < w as i32 && y >= 0 && y < h as i32 {
            img.put_pixel(x as u32, y as u32, image::Rgb(color));
        }
        if x == p2.x && y == p2.y {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}

fn draw_filled_circle(img: &mut image::RgbImage, center: Point, radius: i32, color: [u8; 3]) {
    let (w, h) = img.dimensions();
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            if dx * dx + dy * dy <= radius * radius {
                let x = center.x + dx;
                let y = center.y + dy;
                if x >= 0 && x < w as i32 && y >= 0 && y < h as i32 {
                    img.put_pixel(x as u32, y as u32, image::Rgb(color));
                }
            }
        }
    }
}

fn draw_filled_polygon(img: &mut image::RgbImage, polygon: &[Point], color: [u8; 3]) {
    if polygon.is_empty() {
        return;
    }
    let min_y = polygon.iter().map(|p| p.y).min().unwrap().max(0);
    let max_y = polygon
        .iter()
        .map(|p| p.y)
        .max()
        .unwrap()
        .min(img.height() as i32 - 1);

    for y in min_y..=max_y {
        let mut intersections: Vec<i32> = Vec::new();
        let n = polygon.len();
        let mut j = n - 1;
        for i in 0..n {
            let pi = polygon[i];
            let pj = polygon[j];
            if (pi.y > y) != (pj.y > y) {
                let x = pi.x as f64
                    + (y as f64 - pi.y as f64) * (pj.x as f64 - pi.x as f64)
                        / (pj.y as f64 - pi.y as f64);
                intersections.push(x.round() as i32);
            }
            j = i;
        }
        intersections.sort();
        let w = img.width() as i32;
        for pair in intersections.chunks(2) {
            if pair.len() == 2 {
                let x1 = pair[0].max(0);
                let x2 = pair[1].min(w - 1);
                for x in x1..=x2 {
                    img.put_pixel(x as u32, y as u32, image::Rgb(color));
                }
            }
        }
    }
}

fn pseudo_random(state: &mut u64) -> f32 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
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
    [
        ((r + m) * 255.0) as u8,
        ((g + m) * 255.0) as u8,
        ((b + m) * 255.0) as u8,
    ]
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

    // Bot tier analysis
    eprintln!("=== Bot tier analysis ===");
    let bot_result = analyzer::analyze(&img, &bot_obstacle);
    render_tier_result(&bot_result, &img, "result-bot.png");
    export_to_json(&bot_result, "result-bot.json");

    // Vehicle tier analysis
    eprintln!("=== Vehicle tier analysis ===");
    let vehicle_result = analyzer::analyze(&img, &vehicle_obstacle);
    render_tier_result(&vehicle_result, &img, "result-vehicle.png");
    export_to_json(&vehicle_result, "result-vehicle.json");

    eprintln!("Analysis complete. Output saved to result-bot.png/json, result-vehicle.png/json");
}
