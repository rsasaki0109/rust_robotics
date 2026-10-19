# Fixed-bound clearance components after joint release

Freeze from main262868eaba4f2e1725be198a78a148448b8555a5 before inspecting new
component distributions. Known design observations: seed1003 has108 cases,
54 pairs,251 resumes,12801 lag0..50 points and89607 seven-variant evaluations.
All13 candidate lag2 re-guards have worse carried position error; physical
x/y and full-pose references enter in12/13. A handoff extension is unsupported.

Use BOTH complete13-file pose-accuracy evaluations, each exactly matching its
retained repeat manifest. Bind all inherited inputs, source and validation.
Raw physics, contact/hold, estimator replay, pre/post-state alignment and native
geometry are inherited by this chain of hashes, NOT claimed as newly rerun.
No new simulation or Rust geometry query is needed for this scalar annotation.
Retain every89607 row, all seven variants,108 cases/54 pairs,251 resumes,
held/timeout, no-loss zero-resume controls, zero targets and invalid prefixes.
Keep the fixed50 following decisions clipped only at packet endpoint, with
overlapping anchors separate. Truth stays evaluation-only.

For each row let S=recorded full-precision stopping lower bound, T=terminal-arc
clearance, B=preview_margin, G=release_margin, R=0.5. Require exact native
identities E=min(S,T-0.1), L=min(S,T-0.05), recorded E/L, G=B+0.05 and original
ordered B=0.5+0+2*sqrt(2)*effective_sigma+0.02*observation_age. Never derive B
by subtracting0.05 from G or rearrange threshold subtraction into addition.
Validate every finite numeric field, original case/point/window/variant order,
all inherited booleans and exact same-point inputs across variants.

Entry uses E<=B. Classify stop_only, terminal_only, both or clear using S<=B
and T-0.1<=B. Report the binding entry component separately: stop, terminal or
tie when |S-(T-0.1)|<=1e-12m. Tie tolerance affects labels only; all entry and
release predicates remain exact. Release geometry is clear only if L>G;
classify blocked stop_only, terminal_only, both or clear via the two strict
component tests. Do not turn a geometric release into an actual timed release.

Decompose allowance on the SAME fixed S/T bounds with four exact checks:
min(S,T)<=R; min(S,T)<=B; E<=R; E<=B. For an actual entry classify radius_level
if the first is true; otherwise margin_only if only the second is true,
reserve_only if only the third is true, either_sufficient if both are true,
jointly_required if neither is true. Non-entry is clear. These six exclusive
labels describe fixed-score threshold checks, not new policies. In particular,
S uses B in its far-obstacle shortcut; do not claim to recompute S at radius R.
S is a conservative bound, and T is a virtual0.6m fixed-steering arc after
nominal0.2s motion plus braking. Neither is an observed collision or a prediction
of changed closed-loop motion. Do not retune margins, duration or steering.

Export stop/terminal/combined entry and release gaps to B/G, raw-minimum gaps to
R/B, entry gap to R and B-R allowance. Count all mechanism, binding, layer and
release-block labels; retain mean/min/max entry gap, component mean gaps,
release mean gap and allowance. Empty cells have zero counts and EMPTY gap
statistics. Report entry-ready rows separately via inherited inactive-prior-
guard/no-joint-pause flag; retain already-active and non-entry controls.

Produce108 case rows and54 pair rows per variant; retain all noise
(all/white/correlated),mode(all/none/random/blackout),arm,outcome(all/held/timeout),
variant cells for scopes lag0,lag1,lag2,within5,within25,within50,
triggers_within50. within scopes exclude lag0; trigger scope uses every original
actual trigger row. Keep every251-resume record with first physical-reference
entry and lag2 component/layer/gaps, and report all13 candidate lag2 trigger
events alongside the complete lag2 cohort. Do not select favorable mechanisms,
held cases, positive targets or only the known13 events.

Freeze/commit protocol and input hashes before new distributions. Commit
implementation and meaningful boundary, native-identity, corruption, complete-
matrix and end-to-end tests before annotation. Reject Python-O, changed hashes,
nonfinite values, missing/mixed/reordered joins, existing/internal output and
uncommitted implementation. Run twice in fresh external directories; require
all exports and provenance byte identical. Preserve failed artifacts. Track
compact case/pair/summary/verification/resume exports, complete manifests,
validation and report, keeping bulk annotations outside Git. Run the complete
suite without skips and related CI, then merge under standing authorization.
