//! Spring map data format (.zktamap) and move-tier obstacle predicates.
//!
//! Binary format (version 2):
//!   magic:        b"ZKTA" (4 bytes)
//!   version:      u32 = 2
//!   map_width:    u32   (heightmap squares)
//!   map_height:   u32   (heightmap squares)
//!   slope_width:  u32   (map_width / 2)
//!   slope_height: u32   (map_height / 2)
//!   slope_map:    [f32; slope_width * slope_height]
//!   height_map:   [f32; map_width * map_height]   (center heights, one per square)

use std::io::{self, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 4] = b"ZKTA";
const VERSION: u32 = 2;

/// Spring map data extracted by the SAI.
pub struct SpringMapData {
    pub map_width: u32,
    pub map_height: u32,
    pub slope_width: u32,
    pub slope_height: u32,
    pub slope_map: Vec<f32>,
    pub height_map: Vec<f32>,
}

impl SpringMapData {
    /// Write to .zktamap binary format.
    pub fn write_to<W: Write>(&self, w: &mut W) -> io::Result<()> {
        w.write_all(MAGIC)?;
        w.write_all(&VERSION.to_le_bytes())?;
        w.write_all(&self.map_width.to_le_bytes())?;
        w.write_all(&self.map_height.to_le_bytes())?;
        w.write_all(&self.slope_width.to_le_bytes())?;
        w.write_all(&self.slope_height.to_le_bytes())?;
        for &v in &self.slope_map {
            w.write_all(&v.to_le_bytes())?;
        }
        for &v in &self.height_map {
            w.write_all(&v.to_le_bytes())?;
        }
        Ok(())
    }

    /// Read from .zktamap binary format.
    pub fn read_from<R: Read>(r: &mut R) -> io::Result<Self> {
        let mut magic = [0u8; 4];
        r.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "bad magic: expected ZKTA"));
        }

        let version = read_u32(r)?;
        if version != VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unsupported version {}, expected {}", version, VERSION),
            ));
        }

        let map_width = read_u32(r)?;
        let map_height = read_u32(r)?;
        let slope_width = read_u32(r)?;
        let slope_height = read_u32(r)?;

        let slope_count = (slope_width * slope_height) as usize;
        let height_count = (map_width * map_height) as usize;

        let slope_map = read_f32_vec(r, slope_count)?;
        let height_map = read_f32_vec(r, height_count)?;

        Ok(SpringMapData {
            map_width,
            map_height,
            slope_width,
            slope_height,
            slope_map,
            height_map,
        })
    }

    /// Load from a .zktamap file on disk.
    pub fn load(path: &Path) -> io::Result<Self> {
        let mut file = std::fs::File::open(path)?;
        Self::read_from(&mut file)
    }

    /// Save to a .zktamap file on disk.
    pub fn save(&self, path: &Path) -> io::Result<()> {
        let mut file = std::fs::File::create(path)?;
        self.write_to(&mut file)
    }

    /// Water depth at a heightmap square center. Negative height = underwater.
    /// Returns 0.0 for above-water cells.
    fn water_depth_at_hm(&self, hm_x: u32, hm_y: u32) -> f32 {
        let idx = (hm_y * self.map_width + hm_x) as usize;
        let h = self.height_map[idx];
        if h < 0.0 { -h } else { 0.0 }
    }

    /// Average water depth for a slope-map cell.
    /// Each slope cell covers a 2×2 block of heightmap squares.
    fn water_depth_at_slope(&self, sx: u32, sy: u32) -> f32 {
        let hx = sx * 2;
        let hy = sy * 2;
        let d00 = self.water_depth_at_hm(hx, hy);
        let d10 = self.water_depth_at_hm(hx + 1, hy);
        let d01 = self.water_depth_at_hm(hx, hy + 1);
        let d11 = self.water_depth_at_hm(hx + 1, hy + 1);
        (d00 + d10 + d01 + d11) * 0.25
    }

    /// Compute a boolean obstacle bitmap for the given move tier.
    /// Returns (obstacle_map, width, height) where width = slope_width, height = slope_height.
    pub fn compute_obstacle_map(&self, tier: MoveTier) -> (Vec<bool>, u32, u32) {
        let w = self.slope_width;
        let h = self.slope_height;
        let mut obstacles = vec![false; (w * h) as usize];

        for sy in 0..h {
            for sx in 0..w {
                let idx = (sy * w + sx) as usize;
                let slope = self.slope_map[idx];
                let depth = self.water_depth_at_slope(sx, sy);
                obstacles[idx] = tier.is_obstacle(slope, depth);
            }
        }

        (obstacles, w, h)
    }
}

fn read_u32<R: Read>(r: &mut R) -> io::Result<u32> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf)?;
    Ok(u32::from_le_bytes(buf))
}

fn read_f32_vec<R: Read>(r: &mut R, count: usize) -> io::Result<Vec<f32>> {
    let mut buf = vec![0u8; count * 4];
    r.read_exact(&mut buf)?;
    Ok(buf.chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect())
}

/// Movement tier — each defines which slope-map cells are obstacles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MoveTier {
    Spider,
    Bot,
    Vehicle,
    Hover,
    Ship,
    Amphib,
}

/// Water depth threshold for land units (spider, bot, vehicle).
const LAND_WATER_DEPTH_MAX: f32 = 17.0;
/// Minimum water depth for ships.
const SHIP_MIN_DEPTH: f32 = 6.0;

impl MoveTier {
    /// All six tiers in canonical order.
    pub const ALL: &[MoveTier] = &[
        MoveTier::Spider,
        MoveTier::Bot,
        MoveTier::Vehicle,
        MoveTier::Hover,
        MoveTier::Ship,
        MoveTier::Amphib,
    ];

    /// Slope threshold for this tier (engine slope = 1 - cos(degrees * pi / 180)).
    pub fn slope_threshold(self) -> f32 {
        match self {
            MoveTier::Spider => 0.66,
            MoveTier::Bot | MoveTier::Amphib => 0.19,
            MoveTier::Vehicle | MoveTier::Hover => 0.049,
            MoveTier::Ship => 0.0, // ships don't care about slope on water
        }
    }

    /// Whether this slope+depth cell is an obstacle for this tier.
    pub fn is_obstacle(self, slope: f32, water_depth: f32) -> bool {
        match self {
            // Land units: blocked by steep slope OR deep water
            MoveTier::Spider | MoveTier::Bot | MoveTier::Vehicle => {
                slope >= self.slope_threshold() || water_depth > LAND_WATER_DEPTH_MAX
            }
            // Hover: on land check slope, on water always passable
            MoveTier::Hover => {
                water_depth <= 0.0 && slope >= self.slope_threshold()
            }
            // Ship: only water with sufficient depth is passable
            MoveTier::Ship => {
                water_depth < SHIP_MIN_DEPTH
            }
            // Amphib: slope applies everywhere, ignores water depth
            MoveTier::Amphib => {
                slope >= self.slope_threshold()
            }
        }
    }

    /// Parse tier name (case-insensitive).
    pub fn from_name(name: &str) -> Option<MoveTier> {
        match name.to_lowercase().as_str() {
            "spider" => Some(MoveTier::Spider),
            "bot" => Some(MoveTier::Bot),
            "vehicle" | "veh" => Some(MoveTier::Vehicle),
            "hover" => Some(MoveTier::Hover),
            "ship" => Some(MoveTier::Ship),
            "amphib" => Some(MoveTier::Amphib),
            _ => None,
        }
    }

    /// Tier name for output directories and display.
    pub fn name(self) -> &'static str {
        match self {
            MoveTier::Spider => "spider",
            MoveTier::Bot => "bot",
            MoveTier::Vehicle => "vehicle",
            MoveTier::Hover => "hover",
            MoveTier::Ship => "ship",
            MoveTier::Amphib => "amphib",
        }
    }
}
