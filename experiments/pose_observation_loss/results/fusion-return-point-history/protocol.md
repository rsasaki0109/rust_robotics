# Frozen local point-gap histories for the original 25 references

Freeze from main 9cc0426a527c3b200a0c7312e9d964a7b06932ae before new
context gaps or run lengths are read. Select ALL 25 preceding references with
prior_certainty=unknown, retaining their original order. They are the fixed
nominal-only conjunction failures: 15 reset references, four raw-away references,
23 initial-unsafe and two first-chord cases are inherited subgroups. Do not
select only the four away cases or choose a horizon after viewing outcomes.
Preserve all 3,751 decisions, 1,096 inherited zero-probe audits, 3,997 anchors,
137 returns, 972 cases, 324 families, 570 timeouts and 336 summary cells.
No controller, parameter, contact/outcome label or rollout changes.

Plan exactly 51 slots per selected reference, offsets -25..25 at 50 Hz
(-0.5..+0.5 seconds), including current offset zero. Source steps outside 1..9000
have before_source/after_source status, in that priority. For a positive offset
at or beyond the original first next-loss step, use at_or_after_next_loss and
leave all new raw/mean fields blank. Censored returns have no observed loss;
keep source steps through 9000. All other slots are observed. Negative offsets
may precede the first return and must be marked before_return context rather
than promoted to available follow-up. Zero is reference; remaining included
slots are available_context. Future queries must stay strictly before loss.
Never read/export a missing slot's geometry or treat it as a failed condition.
All 1,275 planned slots remain in exports, including unavailable ones.

Pin preceding sources/proof/protocol/freeze and full transitive input/helper/
Rust/raw chain. Both readers freshly hash/stream 108 raw files, validate complete
case grids and 570 noisy initial branches, and replay full histories containing
a selected reference, checking all original recovery counts. Query included
slots only, preserving duplicate role-specific reference/offset queries and
counting unique case/step contexts separately. Replaying full histories to
validate source logic does not extend the exported follow-up.

Reuse the pinned collector deque and standalone admissions-list replay, source
pose addition order, exact speed/age/pause/clear/append/mean rules, logged mean
enclosures and original native window fragment. At offset zero and -1 compare
the preceding point-gap fields, counts, raw strings and centers; all earlier
values remain inherited. Record original raw truth/noise/bias strings for each
included point, exact logged speed_margin, count and last-clear metadata,
reconstructed raw/mean coordinate bounds and gap/distance intervals.

For each reference use its ENTIRE inherited definite obstacle ID set at every
slot, retaining global map row IDs and duplicate coordinates. Do not select a
new nearest obstacle from the whole map or drop fixed-set members. Reuse the
pinned point distance/interval functions: sqrt(dx^2+dy^2), H=5e-10, P=1e-12,
R=3H+2P, nonempty mean radius R+2P and raw fallback radius R. Each slot uses its
own logged margin +/-H for BOTH raw and mean gaps. Save every obstacle witness,
fixed-set minimum lower/upper distances, center ties and possible minimum IDs.
Clear requires gap lower>0; blocked requires upper<=0; otherwise unknown.
These are conditional rounded-input point diagnostics, distinct from curved
first chords, straight recovery segments, map boundaries and whole probes.

For each raw/mean point, count observed clear/blocked/unknown and missing slots
separately for the 25 negative and 25 positive offsets. Record current state.
Scan backward from zero through consecutive observed slots with that same state.
Save inclusive checkpoint count L, start offset, and checkpoint span (L-1)/50 s.
If the scan reaches -25 or an unavailable slot without seeing a different state,
mark left_censored=true and leave predecessor-state bracket blank. Otherwise
save the immediately preceding observed different state and left_censored=false.
This is a finite-grid run and span, not exact continuous transition time.
Unknown is a separate state and breaks a clear/blocked run.

Define raw_new_blocked only for current blocked with -1 observed clear;
raw_persisting_blocked only for current blocked with -1 observed blocked;
keep prior unknown explicit rather than forcing either label. For current
blocked only, find the FIRST certain clear at included positive offsets.
Record its offset/time as first_later_clear and state clear_observed, regardless
of preceding unknowns. If none: unresolved if any included positive state is
unknown; else no_future if none are included; else no_clear_observed.
For current clear/unknown use current_not_blocked and blank first-clear fields.
Keep included/missing positive counts so a truncated no-clear result cannot
claim an entire half-second of observation. Do not call a later clear point
controller recovery, motion, safety, or an outcome improvement.

Save history-decisions.csv retaining every prior field, a history_audited flag
and new per-reference counts, runs, censorship and later-clear fields. The other
3,726 references keep new fields blank. Save history-slots.csv in original
selected-reference then offset order, all planned slots and explicit statuses;
missing slots retain only identity, timing and status metadata. Save included
history-witnesses.csv in original fixed-obstacle order. Save anchor-history.csv
and history-summary.csv retaining all prior columns and original exact weights.
Add phase/branch selected, raw/mean current three-state, left-censored run,
raw-new/persisting-blocked, raw-away-blocked and five later-clear status counts.
Add available anchor flags for raw-new-blocked, raw-persisting-blocked and
raw-later-clear, with raw/case-mass shares. Original weights are formed before
selection: retain 10,995 RP records and masses 570/570/157, 336 cells and
additive mode/category closures. Nonreturns and undefined shares remain blank;
flags may overlap. New slots/witnesses are not new decision references.

Freeze one 5x5 figure of all 25 raw/mean signed gap histories, original order,
zero reference line and gap boundary, fixed x range +/-0.5 s and one shared y
range spanning all included interval bounds. Unavailable slots break lines;
before-return context and the four inherited raw-away labels remain explicit.
Export all 1,275 plot slots, 252 original-mode summary cells and 137 returns.
Track SVG/CSV/provenance, inspect scratch PNG and repeat every collector and
plot artifact byte for byte. Report current run-length distributions, censoring,
later-clear states/offsets, pre/post state counts for all 25 and fixed raw-away,
reset and initial/first-chord subgroups, plus the two predeclared chord examples.

The standalone verifier imports no analyzers/plotters. Reuse pinned independent
raw/window/point helpers; independently reconstruct planning, run scanning,
aggregation and every export. New point arithmetic uses 60-digit Decimal,
abs 1e-9 / rel 1e-10 numeric comparison and exact states/IDs/counts/strings.
Tolerance never narrows intervals. Compile unchanged original Rust window and
Point2D.distance, tracing included contexts and all rounded raw/mean point-
obstacle pairs. Check native distance/gap bounds and minima, saving small
queries/traces and hashing full scratch replay inputs. This checks rounded
contexts, not original unrounded values or a closed-loop controller replay.

Test horizon endpoint priority, exact loss exclusion, censored source end,
before-return tagging, duplicate shared queries, unknown/missing run breaks,
left censorship, checkpoint span/count distinction, first certain later clear,
future unknown/no-future/truncation, fixed duplicate obstacle IDs, raw strings,
original measures, nonreturns and closures. Run full Python/native suites,
save report and README instructions, then merge/push under standing authority.
