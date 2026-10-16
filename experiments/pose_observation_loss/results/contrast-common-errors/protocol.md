# Linear contrasts and remaining common error

Start from main 01083df. Pin all 19,830 simultaneous raw/recovery/preview
pairs, position-error components and the 550/509 loss links; all 20,664
requested correction pre/post pairs and their original snapshots; both source
provenances and event tables. Keep unchanged 648-case accounting, 368 timeout
histories, 2,583 opportunities and all 6,610 audited simultaneous states.
Tracked inputs suffice. No replay, controller, geometry or physics change.

For each available pair define A/B errors eA/eB, contrast D=eA-eB and midpoint
M=(eA+eB)/2. Simultaneous D is an observable pose difference because the same
truth cancels. Lagged D subtracts known truth displacement and is an offline
oracle diagnostic, not an available online measurement of error change.
Keep both families, all three simultaneous pairings, both lagged inputs,
all four post phases, all event kinds, cohorts, arms and censoring.

Use only the pinned partial per-axis noise-plus-anchor covariance matrix
(VA,VB,C) to choose a linear coefficient. VD=VA+VB-2*C,
VM=(VA+VB+2*C)/4 and CMD=(VA-VB)/2. When VD>1e-12 m^2, set k=CMD/VD and
R=M-k*D. Its model variance VR=VM-CMD^2/VD. This is the linear projection
residual of the midpoint onto the contrast, not a calibrated posterior,
empirically fitted correction or proposed online estimator. No Gaussian
assumption is needed for these second-moment identities. Retain negative
weights: wA=.5-k, wB=.5+k. Do not clip them or select coefficients from truth.

For VD within 1e-12 m^2 of zero require abs(CMD)<=1e-12 m^2, set k=0 and
R=M, VR=VM; keep a zero-or-roundoff-contrast flag and the original VD.
Do not divide by a near-zero variance or assign zero common variance to
identical inputs. Check PSD/nonnegative constraints within 1e-12 m^2,
variance decomposition, weighted-variance closure, wA+wB=1, and residual/
contrast orthogonality outside the degenerate class. Report that class
separately and do not silently reinterpret small positive variance as exact
information identity. Drift/nonlinear transport remain outside the model.

Transform every actual error and each noise/anchor/drift/transport component
by the same fixed weights. Preserve component vector closure. Adding the
same position error to A and B leaves D unchanged and shifts M/R by that
error; swapping A/B leaves M/R unchanged and reverses D/k. These identities
must be tested, along with independent, shared-offset, singular and negative-
weight cases. Truth is used only for observed offline vectors and labels.

Keep the prior five simultaneous agreement bins at closed .05 m, each input's
closed .10 m accuracy disk, and the lagged four accuracy transitions plus
unavailable. No point is relabeled from R and no minimum duration, agreement
or variance threshold is tuned to observed results. Keep every requested
lagged pair; missing projections have blank model/error fields.

Keep original loss transition IDs and 509 flags. Simultaneous summaries retain
all_states, raw_zero_loss and command_only_zero_loss separately, including
repeated source references; lagged summaries retain each post phase/input.
Both summary families include all cohort/arm/event-kind groups, including
empty ones. Categories and estimator modes remain explicit counts and row
metadata. Pool variances over matched references then take square roots;
observed midpoint/residual RMS is a separate two-dimensional diagnostic.
No confidence intervals or independent-trial interpretation.

Export unchanged accounting/events/loss links, every simultaneous projection,
every requested lagged projection, both complete summaries and provenance.
Repeat all seven CSVs/provenance byte for byte. Any figure keeps both arms,
all simultaneous pairings at the complete 550-loss denominator and every
lagged phase/input at the complete requested denominator, with missing counts
explicit. Include all event-kind numeric series; repeat plot CSV/SVG/PNG/
provenance. Source observation/physics checks are inherited, and recorded
control and held goals remain 140->140.
