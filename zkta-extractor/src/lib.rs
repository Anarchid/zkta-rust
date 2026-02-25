//! ZKTA Extractor SAI — extracts slope and height map data from Spring/Recoil
//! engine and writes it to a .zktamap file for offline terrain analysis.
//!
//! Exports init(), release(), handleEvent() as C functions.

#![allow(non_snake_case)] // crate name must be SkirmishAI for the engine

mod callbacks;

use callbacks::EngineCallbacks;
use std::ffi::{c_int, c_void};
use std::path::PathBuf;
use std::sync::Mutex;
use zkta::spring_map::SpringMapData;

/// Per-AI instance state (we only need one, but support the array pattern).
struct AiInstance {
    callbacks: EngineCallbacks,
}

static INSTANCES: Mutex<Vec<Option<AiInstance>>> = Mutex::new(Vec::new());

/// Called by the engine when this AI is instantiated.
///
/// Reads slope map + height map and writes a .zktamap file, then returns.
///
/// # Safety
/// Called by the Recoil engine with valid parameters.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn init(
    skirmish_ai_id: c_int,
    callback: *const callbacks::SSkirmishAICallback,
) -> c_int {
    let cb = unsafe { EngineCallbacks::new(skirmish_ai_id, callback) };
    cb.log("[ZKTA Extractor] Initializing...");

    // Query map dimensions
    let map_width = cb.map_width();
    let map_height = cb.map_height();
    let map_name = cb.map_name().unwrap_or_else(|| "unknown".to_string());
    cb.log(&format!(
        "[ZKTA Extractor] Map: {} ({}x{} heightmap squares)",
        map_name, map_width, map_height
    ));

    // Read height map: (map_width+1) x (map_height+1)
    let height_map = cb.map_get_heightmap();
    cb.log(&format!(
        "[ZKTA Extractor] Height map: {} values",
        height_map.len()
    ));

    // Read slope map: (map_width/2) x (map_height/2)
    let slope_map = cb.map_get_slopemap();
    let slope_width = (map_width / 2) as u32;
    let slope_height = (map_height / 2) as u32;
    cb.log(&format!(
        "[ZKTA Extractor] Slope map: {} values ({}x{})",
        slope_map.len(), slope_width, slope_height
    ));

    // Determine output directory
    let output_dir = cb
        .get_option_value("output_dir")
        .unwrap_or_else(|| "/tmp/zkta".to_string());
    let output_path = PathBuf::from(&output_dir);
    if let Err(e) = std::fs::create_dir_all(&output_path) {
        cb.log(&format!(
            "[ZKTA Extractor] Failed to create output dir {}: {}",
            output_dir, e
        ));
        return -1;
    }

    // Write .zktamap file
    let map_data = SpringMapData {
        map_width: map_width as u32,
        map_height: map_height as u32,
        slope_width,
        slope_height,
        slope_map,
        height_map,
    };

    let file_path = output_path.join(format!("{}.zktamap", map_name));
    match map_data.save(&file_path) {
        Ok(()) => {
            cb.log(&format!(
                "[ZKTA Extractor] Written {}",
                file_path.display()
            ));
        }
        Err(e) => {
            cb.log(&format!(
                "[ZKTA Extractor] Failed to write {}: {}",
                file_path.display(),
                e
            ));
            return -1;
        }
    }

    // Store instance
    let instance = AiInstance { callbacks: cb };
    let mut instances = INSTANCES.lock().unwrap();
    let id = skirmish_ai_id as usize;
    while instances.len() <= id {
        instances.push(None);
    }
    instances[id] = Some(instance);

    cb_log_from_instances(skirmish_ai_id, "[ZKTA Extractor] Done — map data extracted.");
    0
}

/// Log via a stored instance (after moving cb into the instance).
fn cb_log_from_instances(ai_id: c_int, msg: &str) {
    let instances = INSTANCES.lock().unwrap();
    if let Some(Some(inst)) = instances.get(ai_id as usize) {
        inst.callbacks.log(msg);
    }
}

/// Called by the engine when this AI is removed.
///
/// # Safety
/// Called by the Recoil engine with valid parameters.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn release(skirmish_ai_id: c_int) -> c_int {
    let mut instances = INSTANCES.lock().unwrap();
    let id = skirmish_ai_id as usize;
    if let Some(Some(inst)) = instances.get(id) {
        inst.callbacks.log("[ZKTA Extractor] Releasing...");
    }
    if id < instances.len() {
        instances[id] = None;
    }
    0
}

/// Main event handler — we don't need to handle any events.
///
/// # Safety
/// Called by the Recoil engine.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn handleEvent(
    _skirmish_ai_id: c_int,
    _topic: c_int,
    _data: *const c_void,
) -> c_int {
    0
}
