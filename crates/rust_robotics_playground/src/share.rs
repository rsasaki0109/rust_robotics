//! Small, dependency-free helpers for reproducible playground links.

pub const LIVE_PLAYGROUND_URL: &str = "https://rsasaki0109.github.io/rust_robotics/playground/";

pub fn value<'a>(query: &'a str, key: &str) -> Option<&'a str> {
    query
        .trim_start_matches('?')
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find_map(|(candidate, value)| (candidate == key).then_some(value))
}

pub fn bounded_f32(query: &str, key: &str, min: f32, max: f32) -> Option<f32> {
    value(query, key)
        .and_then(|value| value.parse::<f32>().ok())
        .filter(|value| value.is_finite() && (min..=max).contains(value))
}

pub fn bounded_usize(query: &str, key: &str, max: usize) -> Option<usize> {
    value(query, key)
        .and_then(|value| value.parse::<usize>().ok())
        .filter(|value| *value <= max)
}

pub fn boolean(query: &str, key: &str) -> Option<bool> {
    match value(query, key)? {
        "1" | "true" => Some(true),
        "0" | "false" => Some(false),
        _ => None,
    }
}

#[cfg(target_arch = "wasm32")]
pub fn current_query() -> String {
    web_sys::window()
        .and_then(|window| window.location().search().ok())
        .unwrap_or_default()
        .trim_start_matches('?')
        .to_owned()
}

#[cfg(not(target_arch = "wasm32"))]
pub fn current_query() -> String {
    String::new()
}

#[cfg(target_arch = "wasm32")]
pub fn share_url(query: &str) -> String {
    let Some(window) = web_sys::window() else {
        return format!("{LIVE_PLAYGROUND_URL}?{query}");
    };
    let location = window.location();
    let base = match (location.origin(), location.pathname()) {
        (Ok(origin), Ok(pathname)) => format!("{origin}{pathname}"),
        _ => LIVE_PLAYGROUND_URL.to_owned(),
    };
    let relative = format!("?{query}");
    if let Ok(history) = window.history() {
        let _ = history.replace_state_with_url(
            &wasm_bindgen::JsValue::NULL,
            "RustRobotics Playground",
            Some(&relative),
        );
    }
    format!("{base}{relative}")
}

#[cfg(not(target_arch = "wasm32"))]
pub fn share_url(query: &str) -> String {
    format!("{LIVE_PLAYGROUND_URL}?{query}")
}

#[cfg(test)]
mod tests {
    use super::{boolean, bounded_f32, bounded_usize, value};

    #[test]
    fn reads_exact_query_keys() {
        let query = "tab=grid&planner=theta&map=0f";
        assert_eq!(value(query, "tab"), Some("grid"));
        assert_eq!(value(query, "planner"), Some("theta"));
        assert_eq!(value(query, "plan"), None);
    }

    #[test]
    fn validates_typed_query_values() {
        let query = "noise=1.25&frame=8&playing=1&bad=nan";
        assert_eq!(bounded_f32(query, "noise", 0.2, 3.0), Some(1.25));
        assert_eq!(bounded_f32(query, "bad", 0.0, 2.0), None);
        assert_eq!(bounded_usize(query, "frame", 10), Some(8));
        assert_eq!(bounded_usize(query, "frame", 4), None);
        assert_eq!(boolean(query, "playing"), Some(true));
    }
}
