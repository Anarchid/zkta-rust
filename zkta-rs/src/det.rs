//! Hash collections with a fixed hasher.
//!
//! `std`'s default `RandomState` seeds every map differently, which made the
//! analysis (region/chokepoint ids, and occasionally region counts) vary
//! between runs on the same map. With a fixed-key hasher, iteration order
//! depends only on the data, so results are reproducible.

use std::collections::hash_map::DefaultHasher;
use std::hash::BuildHasherDefault;

pub type HashMap<K, V> = std::collections::HashMap<K, V, BuildHasherDefault<DefaultHasher>>;
pub type HashSet<T> = std::collections::HashSet<T, BuildHasherDefault<DefaultHasher>>;
