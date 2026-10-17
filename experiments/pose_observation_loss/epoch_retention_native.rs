// A std-only harness appended to unchanged, pinned simulator source slices.
// Pose and steering are dummy zero inputs; only epoch metadata is verified.

use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};

fn read_lines(path: &str) -> Vec<String> {
    BufReader::new(File::open(path).unwrap())
        .lines()
        .map(Result::unwrap)
        .collect()
}

fn epochs(groups: &VecDeque<StationaryEpoch>) -> Vec<usize> {
    groups
        .iter()
        .map(|g| (g.time / DT).round() as usize)
        .collect()
}

fn joined(values: &[usize]) -> String {
    values
        .iter()
        .map(usize::to_string)
        .collect::<Vec<_>>()
        .join(";")
}

fn boolean(value: &str) -> bool {
    assert!(value == "true" || value == "false");
    value == "true"
}

fn grid(value: &str) -> usize {
    let time: f64 = value.parse().unwrap();
    let n = (time / DT).round();
    assert!(time.is_finite() && (0.0..=9000.0).contains(&n));
    assert!((time - n * DT).abs() <= 1e-7);
    n as usize
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 4, "native RAW_CSV CASES_TSV QUERIES_TSV");
    // Full planned membership, including held cases, is independent of queries.
    let mut members = BTreeMap::new();
    for line in read_lines(&args[2]) {
        let cols: Vec<_> = line.split('\t').collect();
        assert_eq!(cols.len(), 5);
        let key = (cols[1].to_string(), cols[2].to_string());
        let value = (
            cols[0].to_string(),
            boolean(cols[3]),
            cols[4].parse::<usize>().unwrap(),
        );
        assert!(members.insert(key, value).is_none());
    }
    assert_eq!(members.len(), 9);
    let mut queries = BTreeMap::<(String, String, usize), Vec<(String, String)>>::new();
    let mut request_ids = BTreeSet::new();
    for line in read_lines(&args[3]) {
        let cols: Vec<_> = line.split('\t').collect();
        assert_eq!(cols.len(), 5);
        let identity = (cols[2].to_string(), cols[3].to_string());
        assert!(members[&identity].1);
        assert!(request_ids.insert((cols[0].to_string(), cols[1].to_string())));
        let n = cols[4].parse::<usize>().unwrap();
        assert!((1..=9000).contains(&n));
        queries
            .entry((identity.0, identity.1, n))
            .or_default()
            .push((cols[0].to_string(), cols[1].to_string()));
    }
    let mut input = BufReader::new(File::open(&args[1]).unwrap()).lines();
    let header = input.next().unwrap().unwrap();
    assert!(!header.contains('"'));
    let columns: Vec<_> = header.split(',').collect();
    assert_eq!(columns.len(), columns.iter().collect::<BTreeSet<_>>().len());
    let positions: BTreeMap<_, _> = columns.iter().enumerate().map(|(i, k)| (*k, i)).collect();
    let mut histories = BTreeMap::<(String, String), StationaryEpochFusion>::new();
    let mut last = BTreeMap::new();
    let mut out = BufWriter::new(std::io::stdout().lock());
    writeln!(out,"event_id,role,case_id,time_s,current_epoch_time_s,measured_speed,observation_age,received,precision_activation,moving,fusion_available,retained_groups,epoch_steps,sample_counts,prior_epoch_steps,prior_sample_counts,cleared_epoch_steps,expired_epoch_steps,capacity_evicted_epoch_steps").unwrap();
    let mut decisions = 0usize;
    let mut timeout_grid_rows = 0usize;
    let mut emitted = 0usize;
    for line in input {
        let line = line.unwrap();
        // The pinned numeric simulator format has no quoted fields. Reject a
        // different format instead of approximating general CSV quoting.
        assert!(!line.contains('"'));
        let row: Vec<_> = line.split(',').collect();
        assert_eq!(row.len(), columns.len());
        let field = |name| row[positions[name]];
        let identity = (field("scenario").to_string(), field("planner").to_string());
        let (cid, timeout, end) = &members[&identity];
        let n = grid(field("time_s"));
        assert_eq!(n, last.get(&identity).map_or(0, |j| j + 1));
        assert!(n <= *end);
        last.insert(identity.clone(), n);
        if !timeout {
            continue;
        }
        timeout_grid_rows += 1;
        if n == 0 {
            continue;
        }
        assert!(boolean(field("stationary_epoch_fusion")));
        let requests = queries.remove(&(identity.0.clone(), identity.1.clone(), n));
        let fusion = histories.entry(identity.clone()).or_default();
        let prior_epochs = if requests.is_some() {
            epochs(&fusion.groups)
        } else {
            vec![]
        };
        let prior_counts: Vec<_> = if requests.is_some() {
            fusion.groups.iter().map(|g| g.samples.len()).collect()
        } else {
            vec![]
        };
        let speed: f64 = field("measured_speed").parse().unwrap();
        let age: f64 = field("observation_age").parse().unwrap();
        assert!(speed.is_finite() && age.is_finite() && age >= 0.0);
        let received = boolean(field("correction_received"));
        let precision = boolean(field("precision_activation"));
        let moving = speed.abs() > 0.02;
        let cleared = if moving || precision {
            prior_epochs.clone()
        } else {
            vec![]
        };
        let time: f64 = field("time_s").parse().unwrap();
        let expired: Vec<_> = if moving || precision || requests.is_none() {
            vec![]
        } else {
            fusion
                .groups
                .iter()
                .filter(|g| time - g.time > 15.0 + 1e-9)
                .map(|g| (g.time / DT).round() as usize)
                .collect()
        };
        let available = fusion
            .observe(
                VehicleState::new(0.0, 0.0, 0.0, speed, WHEELBASE),
                0.0,
                ObservationEvent {
                    time,
                    correction: boolean(field("correction")),
                    received,
                    precision_activation: precision,
                },
                age,
            )
            .is_some();
        assert_eq!(
            available,
            boolean(field("epoch_fusion_available")),
            "availability {cid} {n}"
        );
        assert_eq!(
            fusion.groups.len(),
            field("epoch_fusion_group_count").parse::<usize>().unwrap(),
            "group count {cid} {n}"
        );
        decisions += 1;
        if let Some(requests) = requests {
            let current_epochs = epochs(&fusion.groups);
            let current_counts: Vec<_> = fusion.groups.iter().map(|g| g.samples.len()).collect();
            let capacity: Vec<_> = prior_epochs
                .iter()
                .copied()
                .filter(|e| {
                    !cleared.contains(e) && !expired.contains(e) && !current_epochs.contains(e)
                })
                .collect();
            for (eid, role) in requests {
                let epoch = fusion.epoch.map_or(String::new(), |t| format!("{t:.17e}"));
                writeln!(out,"{eid},{role},{cid},{time:.17e},{epoch},{speed:.17e},{age:.17e},{received},{precision},{moving},{available},{},{},{},{},{},{},{},{}",
                    fusion.groups.len(),joined(&current_epochs),joined(&current_counts),joined(&prior_epochs),joined(&prior_counts),joined(&cleared),joined(&expired),joined(&capacity)).unwrap();
                emitted += 1;
            }
        }
    }
    assert!(queries.is_empty(), "unfulfilled native snapshot request");
    assert_eq!(last.len(), members.len());
    for (key, (_, timeout, end)) in &members {
        assert_eq!(last[key], *end);
        if *timeout {
            assert_eq!(*end, 9000);
        }
    }
    assert_eq!(emitted, request_ids.len());
    out.flush().unwrap();
    eprintln!("{{\"verified_decisions\":{decisions},\"timeout_grid_rows\":{timeout_grid_rows},\"snapshot_rows\":{emitted}}}");
}
