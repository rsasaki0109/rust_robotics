//! Privacy-preserving engagement hooks and local experiment history.

#[cfg(target_arch = "wasm32")]
const HISTORY_KEY: &str = "rust-robotics.recent-experiments.v1";
#[cfg(target_arch = "wasm32")]
const LAST_QUERY_KEY: &str = "rust-robotics.last-query.v1";
#[cfg(target_arch = "wasm32")]
const ONBOARDING_KEY: &str = "rust-robotics.onboarding-complete.v1";
#[cfg(any(target_arch = "wasm32", test))]
const MAX_HISTORY: usize = 5;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Experiment {
    pub label: String,
    pub query: String,
    pub saved_at_ms: u64,
}

#[cfg(any(target_arch = "wasm32", test))]
fn sanitize(value: &str) -> String {
    value.replace(['\t', '\r', '\n'], " ")
}

#[cfg(any(target_arch = "wasm32", test))]
fn encode_history(entries: &[Experiment]) -> String {
    entries
        .iter()
        .map(|entry| {
            format!(
                "{}\t{}\t{}",
                entry.saved_at_ms,
                sanitize(&entry.label),
                sanitize(&entry.query)
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[cfg(any(target_arch = "wasm32", test))]
fn decode_history(value: &str) -> Vec<Experiment> {
    value
        .lines()
        .filter_map(|line| {
            let mut fields = line.splitn(3, '\t');
            Some(Experiment {
                saved_at_ms: fields.next()?.parse().ok()?,
                label: fields.next()?.to_owned(),
                query: fields.next()?.to_owned(),
            })
        })
        .take(MAX_HISTORY)
        .collect()
}

#[cfg(target_arch = "wasm32")]
fn storage() -> Option<web_sys::Storage> {
    web_sys::window()?.local_storage().ok().flatten()
}

#[cfg(target_arch = "wasm32")]
fn now_ms() -> u64 {
    js_sys::Date::now().max(0.0) as u64
}

/// Emits a DOM event that a privacy-friendly analytics script may subscribe to.
/// No network request is made by the playground itself.
#[cfg(target_arch = "wasm32")]
pub fn track(event: &str) {
    let event_name = format!("rust-robotics:{event}");
    if let (Some(window), Ok(event)) = (web_sys::window(), web_sys::Event::new(&event_name)) {
        let _ = window.dispatch_event(&event);
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn track(_event: &str) {}

#[cfg(target_arch = "wasm32")]
pub fn recent_experiments() -> Vec<Experiment> {
    storage()
        .and_then(|storage| storage.get_item(HISTORY_KEY).ok().flatten())
        .map(|value| decode_history(&value))
        .unwrap_or_default()
}

#[cfg(not(target_arch = "wasm32"))]
pub fn recent_experiments() -> Vec<Experiment> {
    Vec::new()
}

#[cfg(target_arch = "wasm32")]
pub fn save_experiment(label: &str, query: &str) -> Vec<Experiment> {
    let mut entries = recent_experiments();
    entries.retain(|entry| entry.query != query);
    entries.insert(
        0,
        Experiment {
            label: label.to_owned(),
            query: query.to_owned(),
            saved_at_ms: now_ms(),
        },
    );
    entries.truncate(MAX_HISTORY);
    if let Some(storage) = storage() {
        let _ = storage.set_item(HISTORY_KEY, &encode_history(&entries));
        let _ = storage.set_item(LAST_QUERY_KEY, query);
    }
    entries
}

#[cfg(not(target_arch = "wasm32"))]
pub fn save_experiment(label: &str, query: &str) -> Vec<Experiment> {
    vec![Experiment {
        label: label.to_owned(),
        query: query.to_owned(),
        saved_at_ms: 0,
    }]
}

#[cfg(target_arch = "wasm32")]
pub fn last_query() -> Option<String> {
    storage()?.get_item(LAST_QUERY_KEY).ok().flatten()
}

#[cfg(not(target_arch = "wasm32"))]
pub fn last_query() -> Option<String> {
    None
}

#[cfg(target_arch = "wasm32")]
pub fn onboarding_complete() -> bool {
    storage()
        .and_then(|storage| storage.get_item(ONBOARDING_KEY).ok().flatten())
        .is_some()
}

#[cfg(not(target_arch = "wasm32"))]
pub fn onboarding_complete() -> bool {
    false
}

#[cfg(target_arch = "wasm32")]
pub fn mark_onboarding_complete() {
    if let Some(storage) = storage() {
        let _ = storage.set_item(ONBOARDING_KEY, "1");
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn mark_onboarding_complete() {}

#[cfg(test)]
mod tests {
    use super::{decode_history, encode_history, Experiment, MAX_HISTORY};

    #[test]
    fn history_round_trips_and_drops_malformed_rows() {
        let entries = vec![Experiment {
            label: "A* comparison".to_owned(),
            query: "tab=grid&planner=astar".to_owned(),
            saved_at_ms: 42,
        }];
        assert_eq!(decode_history(&encode_history(&entries)), entries);
        assert!(decode_history("bad row").is_empty());
    }

    #[test]
    fn history_decoder_enforces_limit() {
        let encoded = (0..10)
            .map(|i| format!("{i}\titem {i}\ttab=grid&item={i}"))
            .collect::<Vec<_>>()
            .join("\n");
        assert_eq!(decode_history(&encoded).len(), MAX_HISTORY);
    }
}
