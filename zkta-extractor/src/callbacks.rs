//! Minimal type-safe bindings for SSkirmishAICallback.
//!
//! Only the callbacks needed for map data extraction.

#[allow(non_snake_case, non_camel_case_types, non_upper_case_globals, dead_code, unused_unsafe)]
mod bindings {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
pub use bindings::SSkirmishAICallback;

use std::ffi::{c_char, c_int, CStr, CString};

/// Safe wrapper around the raw callback pointer table.
pub struct EngineCallbacks {
    ai_id: c_int,
    raw: *const SSkirmishAICallback,
}

unsafe impl Send for EngineCallbacks {}

macro_rules! call {
    ($self:expr, $field:ident $(, $arg:expr)*) => {
        unsafe {
            ((*$self.raw).$field.unwrap())($($arg),*)
        }
    };
}

impl EngineCallbacks {
    /// # Safety
    /// `raw` must be a valid, non-null pointer that remains valid until release().
    pub unsafe fn new(ai_id: c_int, raw: *const SSkirmishAICallback) -> Self {
        Self { ai_id, raw }
    }

    pub fn map_width(&self) -> i32 {
        call!(self, Map_getWidth, self.ai_id)
    }

    pub fn map_height(&self) -> i32 {
        call!(self, Map_getHeight, self.ai_id)
    }

    pub fn map_name(&self) -> Option<String> {
        let ptr: *const c_char = call!(self, Map_getName, self.ai_id);
        if ptr.is_null() {
            None
        } else {
            Some(unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned())
        }
    }

    /// Height map: (map_width+1) x (map_height+1) floats.
    pub fn map_get_heightmap(&self) -> Vec<f32> {
        let w = self.map_width() + 1;
        let h = self.map_height() + 1;
        let size = (w * h) as usize;
        let mut buf = vec![0.0f32; size];
        call!(self, Map_getHeightMap, self.ai_id, buf.as_mut_ptr(), size as c_int);
        buf
    }

    /// Slope map: (map_width/2) x (map_height/2) floats.
    /// Values: (1 - face_normal_y). 0.0 = flat, 1.0 = vertical cliff.
    pub fn map_get_slopemap(&self) -> Vec<f32> {
        let w = self.map_width() / 2;
        let h = self.map_height() / 2;
        let size = (w * h) as usize;
        let mut buf = vec![0.0f32; size];
        call!(self, Map_getSlopeMap, self.ai_id, buf.as_mut_ptr(), size as c_int);
        buf
    }

    /// Read an AI option from the startscript [Options] block.
    pub fn get_option_value(&self, key: &str) -> Option<String> {
        let c_key = CString::new(key).ok()?;
        let ptr: *const c_char = call!(
            self,
            SkirmishAI_OptionValues_getValueByKey,
            self.ai_id,
            c_key.as_ptr()
        );
        if ptr.is_null() {
            None
        } else {
            let s = unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned();
            if s.is_empty() { None } else { Some(s) }
        }
    }

    /// Write a log message to the engine's AI log.
    pub fn log(&self, msg: &str) {
        if let Ok(c_msg) = CString::new(msg) {
            call!(self, Log_log, self.ai_id, c_msg.as_ptr());
        }
    }
}
