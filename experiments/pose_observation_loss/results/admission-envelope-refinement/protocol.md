# Fixed diagnosis and refinement of the existing pose envelopes

Start from main 85cbf00. Keep the entire model-admission-envelopes accounting:
648 cases, all 368 timeouts, 2,120 checkpoints, both inputs, both .6 m paths
and all three original rules (25,440 rows). Preserve all existing subsets,
cohorts, arms, fallbacks and fixed optimistic-arc-gain groups. Do not select
using truth, error, movement or availability. Pin input and helper hashes.

Keep the existing radius, heading interval, nominal steering and full true/
estimated margin M exactly. Copy prior definite central/enveloped decisions.
For each prior unresolved row, distinguish actual ambiguity inside the same
pose set from conservative bounds. Do not alter control or infer new holds.

First evaluate two predeclared axis stencils in full, independent of truth:
- Heading: original -4..4 degree offsets at the estimated position.
- Position: estimated position plus eight directions k*pi/4 (k=0..7), at
  normalized radii .5 and 1-1e-9, keeping estimated heading. Include the origin.
Record separately whether each stencil contains both a clear and blocked
pose. Only use witnesses separated from either predicate boundary by >2e-8 m.
Finding both is a sufficient example of ambiguity, not proof that an axis
is necessary. Absence of witnesses is not a certificate.

If these witnesses have not already demonstrated both outcomes, cover the
normalized disk x*x+y*y<=1 and heading [-4,4] with the root box
[-1,1] x [-1,1] x [-4,4]. Process binary cells breadth first, lower child first.
Discard a cell only if its closest xy point to the origin has norm >1+1e-12.
At its centre evaluate exact obstacle/wall minima for the fixed path. Bound
each distance by +/- allowance, where allowance is r times the normalized
xy half-diagonal plus 2*.6*sin(radians(heading_half_width)/2) plus 2e-8 m.
Keep strict obstacle and inclusive wall predicates from the prior study.

A cell is entirely permitted or blocked when these bounds prove that state.
Otherwise split its largest physical half-width contribution: r*hx, r*hy,
or the heading displacement term; break ties x, y, heading. Intersecting
cell visits have a fixed budget of 256 per prior unresolved row. Discarded
outside-disk cells do not consume that budget. Cell-centre witnesses must
lie within the disk; if outside, try the cell's closest xy point to the origin
at the same heading. Evaluate that feasible point exactly before using it.
Retain normalized witness parameters and check mapped-pose roundoff <=2e-8 m.

Stop the cover search as soon as both witness outcomes exist: classify mixed.
If the entire disk/heading cover is certified with one state, classify permit
or blocked and retain every terminal cell including outside-disk leaves.
Otherwise leave unresolved with the exhausted budget recorded. Mixed and
budget-unresolved are both unresolved admission outcomes, not blocked actions.
Do not infer absence of a clear pose from a finite stencil or exhausted search.

Save two witnesses per mixed row and complete leaf covers for newly definite
rows. Independently verify prefix-complete binary cover reconstruction, cell
discard criteria, bounds, state and witness-domain membership. Audit every
published witness/certificate geometry with the existing combined exact
helper and independent 120 chords. Explored but unpublished geometries use
exact component calculations; do not claim full chord audits for them.

Only after refinement use saved truth labels/coverage to count true/false
permits/blocks and unresolved true-clear/true-blocked. Any refined definite
state must agree with truth when the previous pose envelope covers truth.
Report all four refinement states, prior-unresolved partitions, finite-axis
witness categories and budget use. Preserve the conditional, non-statistical
interpretation: model coverage is not guaranteed; drift/transport are excluded.

Export accounting, per-row results, paired witnesses, proof leaves, published
geometry audits, subgroup summaries and provenance. Repeat byte for byte.
If plotted show the complete prior-unresolved partition at age .5 for both
inputs, paths, uncertainty models and recorded arms. Keep raw/physics replay
inherited, make no new controller/held-goal claims, and do not tune this method
or its budget after seeing the resulting counts.
