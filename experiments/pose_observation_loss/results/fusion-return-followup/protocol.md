# Frozen follow-up after logged Fusion loss

Freeze from main f7a57444599acb4c0f543d702760fa4f19e9f49b before reading new
post-window return outcomes. Follow the previously fixed 152 observed preview
losses (128 TTL expiry, 24 motion clear) until the original 180 s log endpoint.
No new rollout, controller setting, horizon, error label, AUC, fitting or
causal performance estimate. Preserve all preceding planned cases/accounting.

Pin epoch-retention-mechanisms/sources.json, its complete input/helper chain,
all 108 original raw hashes, simulator/VehicleState code, and the preceding
standalone Rust verifier/native harness helpers. Retain 972 planned cases,
324 families, all 570 timeout histories, 3,997 census events, 10,995 RP records
and their 157 parent cases. Historical held goals remain 140->140, latest
62->60. Full case identity includes loss mode. Existing group identities,
loss times/operations, event kinds and original parent weights stay fixed.

Reconstruct original epoch metadata at every timeout decision 1..9000 from
logged speed, age, received and precision inputs. Validate complete unique
0..9000 grids, full raw trajectory membership and every decision's logged
Fusion availability/group count. No pose or gate-selection replay. Use the
unchanged pinned Metadata helper; confirm native parity independently.

Save availability-runs.csv for all 570 timeout histories at inclusive logged
150..180 s, not only RP cases or cases with losses. Maximal boolean runs must
reconstruct all 855,570 late checkpoint references without gaps, overlaps or
adjacent equal runs. Store full case identity, ordinal, inclusive start/end,
checkpoint count and Fusion availability. This is lossless for availability
only; checkpoint counts are not independent physical trials or time exposure.

Save loss-followup.csv for all 3,997 census anchors, retaining preceding full
identity, horizon, flags, loss operation/offset, original all/received census
weights and RP counts/masses. Nonloss anchors have blank return/wait fields.
Flag the 18 preceding TTL-limit anchors without a next step separately; they
are unobserved losses, not returned or right-censored observed-loss episodes.

For each of the 152 observed losses, search every original later grid step
through 180 s for the first available Fusion. A return at exactly 180 s is
observed. If no return is logged, label right_censored and save elapsed
180-loss_time as an observed lower bound; leave return time/wait blank.
Do not treat censoring as no eventual return or use its bound as an observed
return duration. If returned, save first-return time/wait, group identities,
sample counts, oldest remaining TTL, received/precision flags and matching
original census event ID if present (a return need not be event-aligned).

Continue through the first subsequent unavailable checkpoint or 180 s. Save
the first returned spell's endpoint and logged duration, its right-censoring
flag, and one_checkpoint only for a fully observed return-to-loss difference
of exactly one 20 ms step. A return at 180 s has a censored spell of zero
observed span, not a demonstrated one-checkpoint return. Logged spans do not
locate unobserved continuous-time transitions. Do not extrapolate after 180 s.

Save opportunity-followup.csv for each observed-loss anchor and every original
future census opportunity at loss_time < event_time <=180 s, including events
after the first return. Preserve original source IDs, scheduled/received/
precision flags/kinds, raw same-time availability/groups and relation to first
return. Identify the next opportunity and next received opportunity, if any,
plus all/received/missed/precision opportunity counts through the observed
first return or censoring endpoint. These are anchor-opportunity references;
different anchors can reuse a physical event. No independence or new weights.

Save followup-summary.csv with 528 cells: 12 cohort/noise/arm groups x three
original loss modes plus all_modes x all plus ten preceding operation selectors
(nine planned loss operations and no_observed_loss). Keep empty operations and
all planned case/status counts. Summarize original census/RP counts/masses,
anchor horizons, observed-loss/returned/censored anchors, next-opportunity
references and four disjoint outcomes: returned with one observed checkpoint,
returned longer with observed end, returned with censored spell, or no return
before censoring. Save raw and original census-case-mass return fractions,
conditioning on observed losses. Also retain observed return-wait ranges,
with blanks for no observed returns. Distinct case counts are not additive.

Each census anchor retains 1/n_original_case_census_events; received anchors
also retain 1/n_original_case_received_events. Each original RP record retains
1/n_original_RP_parent_case. These measures precede filtering and remain
separate. Sum exact Fraction masses before division. Do not reweight by the
number of selected losses, average case rates or mix censored durations with
observed waits. Empty denominators remain blank. Check additive mode and
operation closures and prior total counts/masses.

Freeze two figures: (1) four disjoint return/censoring outcomes in all 36
original cohort/noise/arm/loss conditions, including empty conditions;
(2) every one of the 152 loss anchors, plotting loss time versus observed
return wait or censoring lower bound in three cohort panels, distinguishing
return/censoring and operation. Export 396 all-operation original-mode summary
rows and all 152 full loss rows, with plot hashes. Track SVGs; PNGs stay scratch.
Inspect both final figures; repeat collector and figure artifacts byte for byte.

The standalone verifier imports no analyzers. Reuse only pinned preceding
standalone original-census/native-compiler helpers; extract original Rust
observe/types/mean_pose/VehicleState/tests unchanged and run its six tests.
Independently read every raw file in Rust, validating every timeout decision
against raw availability/counts. Request every 150..180 s checkpoint in all
570 timeout cases, not collector-chosen returns. Independently reconstruct
runs, first returns, returned-spell endpoints, opportunity references and all
summary/plot fields with exact parent weights. Dummy zero geometry remains
limited to metadata validation; ignore fused positions. Record native source,
helper, compiler, input/protocol/script/output/plot hashes and all field checks.

Test first return at the last step, no future data, censoring bounds, immediate
versus later return, one-checkpoint and censored returned spells, non-event
returns, repeated loss anchors, full-case/event isolation, duplicate/missing
grids, maximal runs, opportunity boundaries, fixed unequal parent measures,
blank versus zero denominators, exact closure and nonmutation. Run the full
Python suite. Save report/README/evidence, then merge this bounded study under
the standing instruction.
