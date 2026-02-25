use std::collections::HashSet;
use std::path::PathBuf;

use crate::analyzer::StageRenderer;
use crate::contour::{Contour, LabeledContour};
use crate::geometry::Point;
use crate::graph::RegionGraph;

/// Diagnostic renderer that saves each pipeline stage to a directory.
pub struct DiagnosticRenderer {
    pub dir: PathBuf,
}

impl DiagnosticRenderer {
    pub fn new(dir: PathBuf) -> Self {
        std::fs::create_dir_all(&dir).ok();
        Self { dir }
    }

    fn save(&self, name: &str, img: &image::RgbImage) {
        let path = self.dir.join(format!("{}.png", name));
        if let Err(e) = img.save(&path) {
            eprintln!("Failed to save {:?}: {}", path, e);
        }
    }

    fn make_base(&self, label_map: &[Vec<i32>], w: u32, h: u32) -> image::RgbImage {
        let mut img = image::RgbImage::new(w, h);
        for pixel in img.pixels_mut() {
            *pixel = image::Rgb([0, 0, 0]);
        }
        // Draw obstacle fill in dark gray
        for y in 0..h {
            for x in 0..w {
                if label_map[y as usize][x as usize] > 0 {
                    img.put_pixel(x, y, image::Rgb([35, 35, 35]));
                }
            }
        }
        img
    }
}

impl StageRenderer for DiagnosticRenderer {
    fn on_contours_traced(&self, full: &[Contour], holes: &[LabeledContour], label_map: &[Vec<i32>], w: u32, h: u32) {
        let mut img = self.make_base(label_map, w, h);

        // Draw each contour in a distinct color, with obstacle ID labels
        for (i, contour) in full.iter().enumerate() {
            let color = obstacle_color(i + 1);
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], color);
            }
            // Label at first point
            if let Some(&p) = contour.points.first() {
                draw_label(&mut img, p, &format!("O{}", i + 1), color);
            }
        }

        // Draw hole contours in orange
        for (i, hole) in holes.iter().enumerate() {
            let color = [255, 140, 0];
            for j in 0..hole.points.len() {
                let k = (j + 1) % hole.points.len();
                draw_line(&mut img, hole.points[j], hole.points[k], color);
            }
            if let Some(&p) = hole.points.first() {
                draw_label(&mut img, p, &format!("H{}", i + 1), color);
            }
        }

        self.save("01_contours_traced", &img);
    }

    fn on_contours_simplified(&self, simplified: &[Contour], outer_count: usize, label_map: &[Vec<i32>], w: u32, h: u32) {
        let mut img = self.make_base(label_map, w, h);

        for (i, contour) in simplified.iter().enumerate() {
            let is_hole = i >= outer_count;
            let color = if is_hole { [255, 140, 0] } else { obstacle_color(i + 1) };
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], color);
            }
            // Draw vertices as dots
            for p in &contour.points {
                draw_filled_circle(&mut img, *p, 2, [255, 255, 255]);
            }
            // Label: H for holes, O for outer
            if let Some(&p) = contour.points.first() {
                let label = if is_hole {
                    format!("H{} ({}v)", i - outer_count + 1, contour.len())
                } else {
                    format!("O{} ({}v)", i + 1, contour.len())
                };
                draw_label(&mut img, p, &label, color);
            }
        }

        self.save("02_contours_simplified", &img);
    }

    fn on_voronoi_built(
        &self,
        edges: &[(Point, Point)],
        simplified: &[Contour],
        label_map: &[Vec<i32>],
        w: u32,
        h: u32,
    ) {
        let mut img = self.make_base(label_map, w, h);

        // Draw ALL Voronoi edges in blue (the complete diagram)
        for &(p1, p2) in edges {
            draw_line(&mut img, p1, p2, [60, 120, 220]);
        }

        // Overlay simplified contours in yellow
        for contour in simplified {
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], [255, 220, 0]);
            }
            // Draw vertices as bright dots
            for p in &contour.points {
                draw_filled_circle(&mut img, *p, 2, [255, 255, 0]);
            }
        }

        self.save("03_voronoi_full", &img);
    }

    fn on_graph_built(
        &self,
        graph: &RegionGraph,
        voronoi_edges: &[(Point, Point)],
        simplified: &[Contour],
        label_map: &[Vec<i32>],
        w: u32,
        h: u32,
    ) {
        let mut img = self.make_base(label_map, w, h);

        // Draw raw Voronoi edges in dim blue (scaffold)
        for &(p1, p2) in voronoi_edges {
            draw_line_alpha(&mut img, p1, p2, [40, 80, 160], 0.3);
        }

        // Draw surviving graph edges in bright white
        for (&id, neighbors) in &graph.adjacency {
            if let Some(node) = graph.nodes.get(&id) {
                for &nid in neighbors {
                    if nid > id {
                        if let Some(neighbor) = graph.nodes.get(&nid) {
                            draw_line(&mut img, node.position, neighbor.position, [200, 200, 200]);
                        }
                    }
                }
            }
        }

        // Draw contours in yellow
        for contour in simplified {
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], [255, 220, 0]);
            }
        }

        // Draw graph nodes as green dots
        for node in graph.nodes.values() {
            draw_filled_circle(&mut img, node.position, 2, [0, 220, 0]);
        }

        self.save("04_graph_built", &img);
    }

    fn on_graph_pruned(
        &self,
        graph: &RegionGraph,
        simplified: &[Contour],
        label_map: &[Vec<i32>],
        w: u32,
        h: u32,
    ) {
        let mut img = self.make_base(label_map, w, h);

        // Draw contours in yellow
        for contour in simplified {
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], [255, 220, 0]);
            }
        }

        // Draw graph edges
        for (&id, neighbors) in &graph.adjacency {
            if let Some(node) = graph.nodes.get(&id) {
                for &nid in neighbors {
                    if nid > id {
                        if let Some(neighbor) = graph.nodes.get(&nid) {
                            draw_line(&mut img, node.position, neighbor.position, [200, 200, 200]);
                        }
                    }
                }
            }
        }

        for node in graph.nodes.values() {
            draw_filled_circle(&mut img, node.position, 2, [0, 220, 0]);
        }

        self.save("05_graph_pruned", &img);
    }

    fn on_classified(
        &self,
        graph: &RegionGraph,
        simplified: &[Contour],
        label_map: &[Vec<i32>],
        w: u32,
        h: u32,
    ) {
        let mut img = self.make_base(label_map, w, h);

        // Draw contours dim
        for contour in simplified {
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], [120, 110, 0]);
            }
        }

        // Draw edges (color-coded by endpoint classification)
        for (&id, neighbors) in &graph.adjacency {
            if let Some(node) = graph.nodes.get(&id) {
                for &nid in neighbors {
                    if nid > id {
                        if let Some(neighbor) = graph.nodes.get(&nid) {
                            draw_line(&mut img, node.position, neighbor.position, [100, 100, 100]);
                        }
                    }
                }
            }
        }

        // Draw classified nodes with labels
        for &id in &graph.region_nodes {
            if let Some(node) = graph.nodes.get(&id) {
                draw_filled_circle(&mut img, node.position, 4, [64, 224, 208]);
                let r = node.radius as i32;
                if r > 3 {
                    draw_circle_outline(&mut img, node.position, r, [30, 100, 90]);
                }
                draw_label(&mut img, node.position, &format!("N{} r{:.0}", id, node.radius), [64, 224, 208]);
            }
        }
        for &id in &graph.chokepoint_nodes {
            if let Some(node) = graph.nodes.get(&id) {
                draw_filled_circle(&mut img, node.position, 4, [255, 50, 50]);
                let r = node.radius as i32;
                if r > 3 {
                    draw_circle_outline(&mut img, node.position, r, [100, 30, 30]);
                }
                draw_label(&mut img, node.position, &format!("N{} r{:.0}", id, node.radius), [255, 50, 50]);
            }
        }
        // Unmarked nodes
        let all_marked: HashSet<usize> = graph.region_nodes.union(&graph.chokepoint_nodes).copied().collect();
        for (&id, node) in &graph.nodes {
            if !all_marked.contains(&id) {
                draw_filled_circle(&mut img, node.position, 2, [120, 120, 120]);
                draw_label(&mut img, node.position, &format!("N{} r{:.0}", id, node.radius), [90, 90, 90]);
            }
        }

        self.save("06_classified", &img);
    }

    fn on_contracted(
        &self,
        graph: &RegionGraph,
        simplified: &[Contour],
        label_map: &[Vec<i32>],
        w: u32,
        h: u32,
    ) {
        let mut img = self.make_base(label_map, w, h);

        // Draw contours dim
        for contour in simplified {
            for j in 0..contour.len() {
                let k = (j + 1) % contour.len();
                draw_line(&mut img, contour.points[j], contour.points[k], [120, 110, 0]);
            }
        }

        // Draw edges
        for (&id, neighbors) in &graph.adjacency {
            if let Some(node) = graph.nodes.get(&id) {
                for &nid in neighbors {
                    if nid > id {
                        if let Some(neighbor) = graph.nodes.get(&nid) {
                            let color = if graph.region_nodes.contains(&id) && graph.chokepoint_nodes.contains(&nid)
                                || graph.chokepoint_nodes.contains(&id) && graph.region_nodes.contains(&nid) {
                                [180, 180, 180]
                            } else {
                                [80, 80, 80]
                            };
                            draw_line(&mut img, node.position, neighbor.position, color);
                        }
                    }
                }
            }
        }

        // Draw region nodes
        for &id in &graph.region_nodes {
            if let Some(node) = graph.nodes.get(&id) {
                draw_filled_circle(&mut img, node.position, 5, [64, 224, 208]);
                draw_label(&mut img, node.position, &format!("R"), [64, 224, 208]);
            }
        }
        // Draw chokepoint nodes
        for &id in &graph.chokepoint_nodes {
            if let Some(node) = graph.nodes.get(&id) {
                draw_filled_circle(&mut img, node.position, 5, [255, 50, 50]);
                draw_label(&mut img, node.position, &format!("C"), [255, 50, 50]);
            }
        }

        self.save("07_contracted", &img);
    }
}

// ---- Drawing helpers ----

fn obstacle_color(id: usize) -> [u8; 3] {
    let hue = ((id as f32 * 0.618033988) % 1.0 + 0.1) % 1.0;
    hsv_to_rgb(hue, 0.85, 0.95)
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

pub fn draw_line(img: &mut image::RgbImage, p1: Point, p2: Point, color: [u8; 3]) {
    let (w, h) = img.dimensions();
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

fn draw_line_alpha(img: &mut image::RgbImage, p1: Point, p2: Point, color: [u8; 3], alpha: f32) {
    let (w, h) = img.dimensions();
    let dx = (p2.x - p1.x).abs();
    let dy = -(p2.y - p1.y).abs();
    let sx = if p1.x < p2.x { 1 } else { -1 };
    let sy = if p1.y < p2.y { 1 } else { -1 };
    let mut err = dx + dy;
    let mut x = p1.x;
    let mut y = p1.y;

    loop {
        if x >= 0 && x < w as i32 && y >= 0 && y < h as i32 {
            let existing = img.get_pixel(x as u32, y as u32).0;
            let blended = [
                (existing[0] as f32 * (1.0 - alpha) + color[0] as f32 * alpha) as u8,
                (existing[1] as f32 * (1.0 - alpha) + color[1] as f32 * alpha) as u8,
                (existing[2] as f32 * (1.0 - alpha) + color[2] as f32 * alpha) as u8,
            ];
            img.put_pixel(x as u32, y as u32, image::Rgb(blended));
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

pub fn draw_filled_circle(img: &mut image::RgbImage, center: Point, radius: i32, color: [u8; 3]) {
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

fn draw_circle_outline(img: &mut image::RgbImage, center: Point, radius: i32, color: [u8; 3]) {
    let (w, h) = img.dimensions();
    let steps = (radius as f64 * 6.28).max(16.0) as i32;
    for i in 0..steps {
        let angle = (i as f64 / steps as f64) * std::f64::consts::TAU;
        let x = center.x + (radius as f64 * angle.cos()).round() as i32;
        let y = center.y + (radius as f64 * angle.sin()).round() as i32;
        if x >= 0 && x < w as i32 && y >= 0 && y < h as i32 {
            img.put_pixel(x as u32, y as u32, image::Rgb(color));
        }
    }
}

/// Tiny 3x5 bitmap font for labels (digits + uppercase letters).
/// Draw label offset to the upper-right of a marker point.
pub fn draw_label(img: &mut image::RgbImage, pos: Point, text: &str, color: [u8; 3]) {
    draw_label_at(img, pos.x + 7, pos.y - 8, text, color);
}

/// Draw label centered on a point.
pub fn draw_label_centered(img: &mut image::RgbImage, pos: Point, text: &str, color: [u8; 3]) {
    let text_width = text.len() as i32 * 5 - 1; // 4px glyph + 1px gap per char
    draw_label_at(img, pos.x - text_width / 2, pos.y - 2, text, color);
}

fn draw_label_at(img: &mut image::RgbImage, start_x: i32, start_y: i32, text: &str, color: [u8; 3]) {
    let (w, h) = img.dimensions();
    let mut cx = start_x;
    let cy = start_y;
    for ch in text.chars() {
        let glyph = char_glyph(ch);
        for (row, &bits) in glyph.iter().enumerate() {
            for col in 0..4 {
                if bits & (1 << (3 - col)) != 0 {
                    let x = cx + col;
                    let y = cy + row as i32;
                    if x >= 0 && x < w as i32 && y >= 0 && y < h as i32 {
                        img.put_pixel(x as u32, y as u32, image::Rgb(color));
                    }
                }
            }
        }
        cx += 5;
    }
}

fn char_glyph(ch: char) -> [u8; 5] {
    match ch {
        '0' => [0b0110, 0b1001, 0b1001, 0b1001, 0b0110],
        '1' => [0b0010, 0b0110, 0b0010, 0b0010, 0b0111],
        '2' => [0b0110, 0b1001, 0b0010, 0b0100, 0b1111],
        '3' => [0b1110, 0b0001, 0b0110, 0b0001, 0b1110],
        '4' => [0b1001, 0b1001, 0b1111, 0b0001, 0b0001],
        '5' => [0b1111, 0b1000, 0b1110, 0b0001, 0b1110],
        '6' => [0b0110, 0b1000, 0b1110, 0b1001, 0b0110],
        '7' => [0b1111, 0b0001, 0b0010, 0b0100, 0b0100],
        '8' => [0b0110, 0b1001, 0b0110, 0b1001, 0b0110],
        '9' => [0b0110, 0b1001, 0b0111, 0b0001, 0b0110],
        'A' => [0b0110, 0b1001, 0b1111, 0b1001, 0b1001],
        'B' => [0b1110, 0b1001, 0b1110, 0b1001, 0b1110],
        'C' => [0b0111, 0b1000, 0b1000, 0b1000, 0b0111],
        'D' => [0b1110, 0b1001, 0b1001, 0b1001, 0b1110],
        'E' => [0b1111, 0b1000, 0b1110, 0b1000, 0b1111],
        'F' => [0b1111, 0b1000, 0b1110, 0b1000, 0b1000],
        'G' => [0b0111, 0b1000, 0b1011, 0b1001, 0b0110],
        'H' => [0b1001, 0b1001, 0b1111, 0b1001, 0b1001],
        'I' => [0b0111, 0b0010, 0b0010, 0b0010, 0b0111],
        'O' => [0b0110, 0b1001, 0b1001, 0b1001, 0b0110],
        'R' => [0b1110, 0b1001, 0b1110, 0b1010, 0b1001],
        'S' => [0b0111, 0b1000, 0b0110, 0b0001, 0b1110],
        'V' => [0b1001, 0b1001, 0b1001, 0b0110, 0b0110],
        '(' => [0b0010, 0b0100, 0b0100, 0b0100, 0b0010],
        ')' => [0b0100, 0b0010, 0b0010, 0b0010, 0b0100],
        'v' => [0b0000, 0b1001, 0b1001, 0b0110, 0b0110],
        _ => [0b0000, 0b0000, 0b0000, 0b0000, 0b0000],
    }
}

pub fn draw_filled_polygon(img: &mut image::RgbImage, polygon: &[Point], color: [u8; 3]) {
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
