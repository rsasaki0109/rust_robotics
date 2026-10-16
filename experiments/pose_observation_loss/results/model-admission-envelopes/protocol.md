# Fixed model-based recovery-path admission envelopes

Start from main a55bdfb. Keep the full 648-case accounting, all 368 timeouts,
2,120 checkpoints, both actual recovery/preview inputs and both straight/
commanded .6 m paths from commanded-arc-components. Join the corresponding
epoch-position-components position-model rows without selecting by error,
coverage, movement or availability. Preserve all / first-usable-epoch / age-.5 /
endpoint subsets and known/new, independent/correlated cohorts.

Compare exactly three predeclared rules at fixed estimated poses and nominal
steering (zero for the original line):

- central: reproduce the original estimated gate exactly, with no envelope.
- noise_envelope: position disk r=2sqrt(2)*model_noise_sigma_per_axis and
  heading within +/-4 degrees of the estimated heading.
- noise_anchor_envelope: same heading interval, but position disk
  r=2sqrt(2)*model_noise_anchor_sigma_per_axis. Use the actual weighted,
  temporally correlated noise/shared-anchor model already exported.

The coefficient 2sqrt(2) and heading range follow existing fixed conventions;
do not tune them to these results. Model inputs are weights, sample timing,
configured sigmas, reset identity and correlation, not realized truth/noise/bias.
Keep M=.6+2sqrt(2)*sigma+.02*age on both estimated and true evaluations.
These envelopes add a requirement to meet that unchanged true-side margin;
they do not replace its uncertainty term or establish a calibrated probability.

For each enveloped rule sample heading offsets -4..4 degrees in 1-degree steps.
Keep obstacle and wall minima separate. A nearest heading sample differs by
at most .5 degrees, moving any .6 m path point by at most
E=2*.6*sin(radians(.5)/2). Lower component bounds are the minimum over heading
samples minus E minus r; upper bounds are their maximum plus E plus r.
Central uses its exact distances as both bounds, with E=r=0.

Classify permit if obstacle_lower>M and wall_lower>=M. Classify blocked if
obstacle_upper<=M or wall_upper<M. Otherwise classify unresolved; do not count
unresolved as blocked, resumed or held. This preserves inclusive wall equality
and strict obstacle clearance. A wider envelope can turn a definite decision
into unresolved, never into the opposite definite decision. Record ambiguity
within 2e-8 m and verify nested rules and the central baseline.

Truth is used only after bounds/classification: report true/false permits,
true/false blocks, unresolved true-clear/true-blocked, baseline true permits
deferred, and empirical position/heading/joint coverage. When truth is within
the disk and heading interval, verify both component bounds contain the true
distances and definite decisions agree with truth. Coverage is descriptive on
outcome-selected, repeated checkpoints, not a confidence/coverage guarantee.
The model excludes drift/transport uncertainty and Gaussian tails are unbounded.

Audit all distinct estimated heading/steering geometries using exact obstacle
and wall extrema, the existing combined helper and independent 120 chords.
Pin tracked inputs, protocol, map, simulator/helpers and prior provenance.
No raw trajectories are needed; inherit their validation and the previous full
window reconstruction rather than claiming new physics or observation replay.
Export accounting, per-rule bounds/decisions, unique audits, subgroup summaries
and fingerprints; repeat byte for byte. If plotted use all age-.5 three-way
decisions for both inputs, paths and arms, retaining unresolved outcomes.
Do not change control, timers, default margins or published held-goal outcomes.
