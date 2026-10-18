# Frozen adjacent-step recovery-window comparison

Freeze from main bd57feeb517c31034e78e703ff77048c6a25104d before extracting
new predecessor-window, membership-change, mean-step/raw-step or gap-change
outcomes. Keep the preceding 1,096 zero-probe references as the entire audited
selection, including available and next-loss joint/guard branches. The preceding
25 nominal-only conjunction failures and 15 nominal-only resets are fixed
subgroups. Preserve all 3,751 decision references, 3,997 anchors, 137 returns,
972 cases, 324 families, 570 timeouts and 336 summary cells. Earlier outcomes
remain inherited. No controller, parameter, rollout or contact label changes.

Pin preceding sources/proof, protocol/freeze and complete transitive input/
helper/code/raw chain. Both readers freshly hash/stream all 108 files, validate
complete unique case grids and the 570 noisy initial branches, and replay the
63 selected histories (567,000 steps). Reconstruct each selected step n and
its predecessor n-1. A predecessor is historical input context, even when it
precedes the first return; it is not a newly available decision or an extension
of follow-up. Preserve native queries for both roles and original reference IDs.

Reuse the pinned independent window implementations: the collector's deque
and standalone verifier's admissions-since-clear list with last-25 selection.
Their source-order pose reconstruction, roundtrip speed, binary64 exact age,
pre-window pause logic, clear/append/eviction rules and count checks remain
unchanged. Freshly compare all replayed sample counts and both contexts' logged
mean enclosures. Retain the inherited current membership and mean fields.

Partition every selected reference by exact ordered membership:
fallback if current count is zero; new if current is nonempty and previous is
empty; reset if both are nonempty and a clear occurs at n; grow for append of
the current sample with all previous members retained and count increased by
one; slide for full 25-to-25 windows replacing the oldest with the current
sample; held for identical memberships without a current clear; otherwise
other. Keep other explicit rather than forcing an expected class. Class order
is the order above. Save all added/removed/shared step IDs and counts. Do not
assume samples are contiguous or treat full capacity as proof of an eviction.

Let m be the reconstructed x/y mean (raw fallback when empty), s the current
sensed x/y, and g=m-s. Record adjacent changes dm=m_n-m_prev, ds=s_n-s_prev,
dg=g_n-g_prev. Check dg=dm-ds with absolute rounded-center residual<=P=1e-12.
When both memberships are nonempty, independently predict
dm=[sum_added(s_i-m_prev)-sum_removed(s_i-m_prev)]/current_count.
Check its rounded-center residual against the direct mean step at <=P. This
includes growth, slide, held and reset; empty-side membership-law fields stay
blank. For a slide this reduces to (newest-removed_oldest)/25. Preserve general
form and never assume that identity for another transition class.

Use the preceding nine-decimal half-unit H=5e-10 and sensed radius R=3H+2P.
Mean radius is R+2P when nonempty and R for raw fallback. For each coordinate,
dm gets radius mean_radius_n+mean_radius_prev+P, ds gets 2R+P, and dg gets
mean_radius_n+mean_radius_prev+2R+3P. Propagate the membership-law sum/subtract/
division intervals with P at each operation. Check its interval intersects the
direct mean-step interval and includes the rounded native mean difference.
These conditional arithmetic bounds do not recover unrounded original inputs.

Compute Euclidean norm enclosures from coordinate rectangles, retaining P
on norm bounds and clipping lower norms at zero. If both means are nonempty,
classify raw_larger only when raw-step lower norm exceeds mean-step upper norm;
mean_larger uses the opposite strict comparison; otherwise overlap. Empty-side
comparisons are not_comparable. Keep this distinction from membership class.
Record signed steps and norm intervals rather than selecting a fitted threshold
or calling this a causal attribution, statistical noise reduction, successful
recovery, or a proposed pose substitution.

Save turnover-decisions.csv for all 3,751 references, keeping prior fields and
adding an audit flag; positive probes have blank new fields. Save previous
membership/mean/raw centers, counts and change IDs, current clear flag, class,
signed step/enclosure/norm fields, membership prediction/enclosures/residuals
and comparison state. Save turnover-members.csv in original reference order,
then union-of-members step order, with retained/added/removed roles, both
membership flags and exact original raw input strings. Empty unions have no
fictional member rows. Current raw input and predecessor raw input remain in
the decision-context fields, including fallback cases.

Save anchor-turnover.csv and turnover-summary.csv retaining preceding columns
and original exact weights. Add phase x branch audited/seven-class/four-
comparison counts and prior-unknown/nominal-only-reset raw_larger counts.
Add available grow/slide/raw_larger anchor flags and original raw/case-mass
shares. Reconstruct original measures before selection, retaining 10,995 RP
records and masses 570/570/157; check additive mode/category closures. Keep
nonreturns and empty share denominators blank; flags can overlap.

Freeze one 2x3 figure of available membership-class counts by original
partition and joint/guard branch, with all seven classes and explicit empty
branches/denominators. Export all 252 original-mode cells and 137 returned
anchors. Track SVG/CSV/provenance; inspect scratch PNG; reproduce all collector
and plot artifacts byte for byte. Report predeclared counts and min/max step
norm bounds for the entire selection, comparable pairs and original 25 subgroup.
Use the two already identified first-chord references as fixed examples only.

The standalone verifier imports no analyzers/plotters. Reuse pinned standalone
window/hash/identity/mean and native helpers; independently build new vector
arithmetic, intervals, membership identities, aggregation and every export
with 60-digit Decimal. Compare new numeric fields at abs 1e-9, rel 1e-10;
exact states, IDs, counts, flags and inherited/raw strings. Tolerance never
narrows intervals. Compile the unchanged original Rust window fragment via
the pinned native helper, check all selected-history counts and trace both
contexts for every original reference. Count duplicate context queries
separately from unique source points; do not call them additional references.

Test every membership class including reset/held/noncontiguous/other cases,
growth and slide formulas, duplicate/shared membership consistency, fallback
blanks, interval containment, inclusive overlap and strict norm comparisons,
large simultaneous removal/addition, signed gap closure, native context mapping,
raw string fidelity, censoring/nonreturns, unequal measures and closures. Run
the full Python suite and pinned native source tests, save report/README/
evidence, then merge/push this bounded audit under the standing authorization.
