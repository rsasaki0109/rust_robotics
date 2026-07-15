//! Small, dependency-free helpers for reproducible playground links.

pub const LIVE_PLAYGROUND_URL: &str = "https://rsasaki0109.github.io/rust_robotics/playground/";

pub fn value<'a>(query: &'a str, key: &str) -> Option<&'a str> {
    query
        .trim_start_matches('?')
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find_map(|(candidate, value)| (candidate == key).then_some(value))
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
    use super::value;

    #[test]
    fn reads_exact_query_keys() {
        let query = "tab=grid&planner=theta&map=0f";
        assert_eq!(value(query, "tab"), Some("grid"));
        assert_eq!(value(query, "planner"), Some("theta"));
        assert_eq!(value(query, "plan"), None);
    }
}
