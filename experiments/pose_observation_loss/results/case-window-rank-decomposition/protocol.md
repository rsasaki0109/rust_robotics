# Case, window and seed comparison protocol

Frozen before computing the new results, from main
e90294f51b85ad24c4de2756a3e78da83470feeb. Reuse the same 32,985 reconstructed
pair references, 10,995 Recovery/Preview states, 157 represented cases, 972
planned cases and 324 planner families. No new simulation, fitting, cutoff,
score direction, estimator or controller changes. This extends the latest-only
case influence study to all three cohorts and the preceding two populations.

Retain separate known 269/270/271, previous additional 274/275/276 and latest
additional 277/278/279 cohorts; both noises/arms, all five strata, both targets,
all three integer-tick scores and all E/P/U labels. Retain mixed planners only;
the preceding study already saves individual-planner results. Reuse original
case weights 1/n_case before all stratum/population selection, without any
renormalization. Reuse source and common_family populations, with eligibility
defined by all three planners represented within that same stratum/family,
using availability only. Keep every missing planned case/family explicit.

Use full case identities and prior exact whole-window/family IDs. A family
removes only planner from the full key, and necessarily retains seed. Whole
windows are identical only under the preceding exact typed diagnostic signature;
partial overlap is not equality. Assign each ordered positive-case/negative-case
comparison to exactly one of these five bins, in priority order:

1. same_case: identical full case key;
2. other_case_same_window: different cases with the same family-scoped exact
   complete window ID;
3. other_window_same_family: different exact windows in the same family;
4. other_family_same_seed: different families with the same seed;
5. other_seed: different seeds.

Compute exact weighted concordant, tied, discordant, numerator and denominator
mass for every bin. High score predicts outside, with half credit for ties;
outside remains norm > .10 m and the inside disk is closed. Save bin AUC,
full-denominator share and numerator contribution, candidate ordered case pairs
and nonzero-denominator case pairs. These pair counts are descriptive, not
independent observations. Every empty bin remains present with zero mass and
blank AUC/share when the corresponding denominator is absent.

Assert that bins reconstruct all full pair masses exactly. Cumulative bins
define same-case, same-window, same-family and same-seed comparisons. Also
save the complementary between-case result. Sum numerators/denominators before
division, never average case AUCs or give supported cases/seeds new equal
weights. Compare reconstructed full, same-seed, same-planner and joint
same-seed/planner statistics with the existing exact helper and every original
mixed-planner ranking field. Match latest source same-case/between-case masses
against the earlier case influence bundle, without mixing its older case IDs
with the current 972-case audit IDs.

Save case-stratum availability and both targets' E/P/U class counts/masses for
all 972 planned cases in all five strata, including zero-support cells and
inherited row weights/common-family eligibility. Save every individual case's
fixed-direction AUC, class support and pair masses for all 157 represented
parent cases, five strata, both targets, three scores and E/P/U labels, including
empty strata. One source case row serves both populations: eligibility says
whether its unchanged states survive common_family selection. Do not fabricate
rankings for cases with no parent history; they remain in the full audit.

Freeze plots to residual_sigma and Filter/Fusion. For each cohort plot residual
observed-E full/within-seed/within-family/within-window/within-case AUCs for both
populations, retaining NA cells and support counts. Plot the latest modeled-P
equivalent. A fifth figure shows all five bins' comparison-mass shares for both
E/P labels, all cohorts/noises/arms and both populations, on residual targets.
Numeric figure exports retain both targets, all displayed scopes, support and
shares. Individual case values remain fully available in CSV, without selecting
cases by their outcomes for panels or omitting inconvenient cohorts.

Preserve original accounting/support/events/cutoffs, historical 550/509 and
latest 101/94 loss links, and complete case/family/stratum eligibility audits.
Pin inputs, helpers, protocol and output hashes. Save code, meaningful grouping,
weight, tie, empty-class, disjoint-bin, cumulative-scope and baseline tests,
CSV/JSON/SVG, README and report. Repeat aggregation and plotting byte for byte,
inspect all figures, independently reconstruct and verify the new case support,
rankings, pair categories and plotted values without importing analyzers, and
run the complete Python suite. Keep verification runnable from the repository.
Merge this completed bounded study under the standing instruction.

The three already observed cohorts are retrospective. Case/window conditioning
can remove class support and alter the comparison population. Repeated states,
windows and planners do not supply independent noise streams, confidence
intervals, causal effects, a winning score direction or real-world calibration.
Live outcomes remain historical 140->140 and latest 62->60.
