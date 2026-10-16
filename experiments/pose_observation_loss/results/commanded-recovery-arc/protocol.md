# Fixed comparison of the recovery line and commanded-steering arc

Start from main a6a9507 and the published joint-admission-geometry tables.
Keep its full accounting of 648 live cases, including all 368 timeouts, and
exactly its 2,120 checkpoints and six poses per checkpoint. Do not reselect by
geometry, paired command differences or final movement. Preserve the overlapping
all / first-usable-epoch / age-0.5 / endpoint subsets and their denominators.
This follow-up consumes tracked CSVs and hashes, not new trajectories.

Compare the original forward .6 m straight recovery segment with a .6 m
constant-steering arc from the same starting pose. Curvature is tan(delta)/.4;
delta is exactly the logged nominal steering. Do not search or optimize steering,
select it using truth, shrink margins, reduce sigma or assume independent samples.
Keep M=.6+2sqrt(2)*sigma+.02*age and M0=.6+2sqrt(2)*sigma.
Retain obstacle and wall distances separately: obstacles must be strictly farther
than M, while wall distance may equal M, matching the recovery gate. Evaluate
current-age and zero-age classifications, flagging boundaries within 2e-8 m.

Reconstruct the exact arc by endpoint/radial projections and interior wall extrema.
Check the straight geometry and decisions against every previous pose row.
Cross-check combined minima with the existing arc helper and independently audit
straight and commanded curves using 120 incremental chords and their sagitta
bound. Cache identical geometry inputs without dropping any recorded rows.
Pin all consumed tables, prior provenance, protocol, map, simulator and helpers;
the prior hash chain retains the original full raw-record validation.

For each checkpoint compare actual recovery input and stopping-preview input
against full truth under both path rules, with the same nominal steering and
margins. Separate optimistic permissions, unnecessary blocks, heading effects
where an existing true-xy/actual-heading pose is available, and physical radius
.5 violations. Count estimated straight-to-arc gains and losses alongside true
gains and losses at the same checkpoint; never equate these with live recoveries.
Record the previously measured positive-speed and base-usability conditions,
without recomputing a substituted target or combined confirmation timer.

Highlight the same three curved lost holds: known-white/random/270/maze/Theta*,
known-correlated/blackout/270/maze/Theta*, new-white/blackout/274/maze/Theta*.
Report all six age-.5 points for each, including negative results. Keep the .05 m
speed chord distinct from the full .6 m recovery arc and from the 2 m speed probe.

Export accounting, per-pose geometry/audits, per-checkpoint decisions and paired
transitions, subgroup summaries and source fingerprints. Repeat byte for byte.
If plotted, use all age-.5 decisions for both arms and inputs, including every
truth/estimate outcome category. Controller code, defaults, previous reports and
live held-goal outcomes remain unchanged in this diagnostic study. A geometric
curve alone cannot establish safe estimator-based release, acceleration behavior,
one-second confirmation or arrival/hold improvements.
