# Fixed decomposition of filtered position error

Reuse all 497 paired aligned-arcs checkpoints on the same 81 failed trajectories.
Keep controller, trajectory, filter window (25), reset rules and margins fixed.
No alternative estimator is selected from the results and no new drive is run.

Reconstruct each filter sample by transporting its original pre-command true
position plus logged position noise and bias, with its own measured heading,
through every subsequent recorded applied motion. Use measured speed and
steering, matching the existing filter's time convention. Reset at correction
(including missed correction) or precision activation. Cross-check reconstructed
positions and circular mean headings against logged filter output on every step.

Track the last received correction/precision anchor: its logged position bias,
or zero before the first such event. At each checkpoint decompose filtered minus
true position exactly into four vectors:

- noise: mean original position noise over retained samples;
- anchor: mean last-received anchor position bias over retained samples;
- drift: mean logged position bias minus that anchor;
- transport: mean original true position plus propagated displacement minus
  the current pre-command true position.

Check the vector sum against observed error; preserve signed projections on the
total error direction, whose sum equals its magnitude. Zero-error projections
are zero. Report norms/RMS and signed mean projections by final/fresh checkpoint
and white/correlated cohort, and identify the largest projection. Norms/RMS of
components do not add, and selected failed-run samples are not independent.

For the noise process alone, report theoretical effective sample count
n^2 / (n + 2*sum((n-k)*rho^k, k=1..n-1)). Use logged stationary AR(1) rho;
do not fit it or convert it into pose confidence. This does not reduce the
common anchor variance or bound drift/transport. Sigma is constant within each
retained window. Compare no new steering choice or acceptance criterion.

Pin all raw trajectory hashes to aligned-arcs and save retained sample details,
checkpoint decomposition, four cohort/reason summaries and provenance. Re-run
and compare all CSVs and sources.json byte for byte. No controller improvement,
hold outcome, safety probability or estimator deployment claim is established.
