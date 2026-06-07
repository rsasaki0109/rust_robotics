# Distributed Formation Consensus via ADMM

A pure-Rust reproduction slice for the coordination layer of distributed MPC, in
the spirit of ACLM-style consensus control. It is the first distributed/
optimization-control target in this repo.

A team of agents must agree on a common formation center `z` while each agent
settles at `z + offset_i`, trades off staying near its own preferred position
`a_i`, and respects a per-agent box constraint. The agreement is reached by the
classic *consensus ADMM* iteration: each agent solves a tiny local proximal
problem, the team averages to a consensus center, and scaled dual variables drive
the per-agent residuals to zero.

## Problem

Minimize `sum_i (w_i/2) ||x_i - a_i||^2` subject to `x_i - offset_i = z` for all
agents and `lower_i <= x_i <= upper_i`.

With equal weights and no box constraints the consensus center has the closed
form `z* = mean(a_i - offset_i)`, which the solver recovers exactly. Box
constraints break the closed form and make the problem genuinely iterative — that
is where ADMM earns its keep.

## Algorithm

Consensus ADMM (scaled dual form), iterated to convergence:

1. **x-update (local).** Each agent independently solves its proximal step and
   projects onto its box:
   `x_i = clamp( (w_i a_i + rho (z + offset_i - u_i)) / (w_i + rho) )`.
2. **z-update (consensus).** The team averages: `z = mean_i (x_i - offset_i + u_i)`.
3. **dual update.** `u_i += x_i - z - offset_i`.

The primal residual is `sqrt(sum_i ||x_i - z - offset_i||^2)` and the dual
residual is `rho * sqrt(N) * ||z - z_prev||`; the iteration stops once both fall
below the tolerance.

- [`AgentSpec`] — one agent's preferred position, formation offset, weight, and
  box bounds (`with_box`, `with_weight`).
- [`AdmmConfig`] — penalty `rho`, iteration cap, residual tolerance.
- [`solve_formation_consensus`] — runs the iteration and returns the agent
  positions, the consensus center, and the primal/dual residual history.

## Benchmark

`benchmark_admm_formation` places six agents that want a regular hexagon around a
shared center, each pulled toward a scattered preferred position. The two
right-side agents are confined to a left corridor (a box constraint), so their
formation slots cannot sit where the unconstrained average would put them and the
whole consensus center is pulled left to keep them inside it. The benchmark writes
`docs/assets/admm-formation.csv` and `.svg` (preferred positions, the final
formation polygon and center, the corridor, and a log-scale residual inset):

```bash
cargo run -p rust_robotics --example benchmark_admm_formation --no-default-features --features control
```

On the bundled instance the consensus center settles at about `(-1.20, -0.03)` —
shifted left by the corridor — and both residuals fall below `1e-7` in about 91
iterations (versus ~25 when the corridor does not bind).

## Decentralized Graph Consensus

The centralized solver above uses a global average in its z-update. The
*decentralized* variant ([`solve_graph_consensus`]) removes the central
coordinator: agents agree only with their neighbors on a communication graph,
using edge-based ADMM. For `minimize sum_i (w_i/2)||x_i - a_i||^2` subject to
`x_i - offset_i = x_j - offset_j` on every edge, each agent keeps an aggregated
dual `alpha_i` over its incident edges and updates (on shifted `y_i = x_i -
offset_i`, with `a_i' = a_i - offset_i`):

`y_i = (w a_i' - alpha_i + rho * sum_{j in N_i}(y_i + y_j)) / (w + 2 rho deg_i)`,
then `alpha_i += rho * sum_{j in N_i}(y_i - y_j)`,

using only neighbor values `y_j`. On a connected graph it converges to the same
weighted-average consensus as the centralized solver; the *rate* is set by the
graph connectivity.

`benchmark_admm_graph_consensus` runs the same eight agents over a line graph, a
ring, and a complete graph:

```bash
cargo run -p rust_robotics --example benchmark_admm_graph_consensus --no-default-features --features control
```

All three reach the same center, but the complete graph converges in about 36
iterations versus ~80 for the sparse line/ring graphs — better connectivity
(larger algebraic connectivity) means faster agreement. (For a small graph the
line and ring are close and can swap; the robust ordering is the complete graph
being far faster.)

## Receding-Horizon Trajectory Consensus

The two solvers above agree on a single static center. The *receding-horizon*
variant ([`solve_horizon_consensus`]) agrees on a shared center **trajectory**
`z[0..H]` over a short horizon. Each agent tracks a per-step reference `a_i[t]`,
sits at `z[t] + offset_i`, and the shared center carries a temporal-smoothness
(acceleration) penalty:

`minimize sum_{i,t} (w_i/2)||x_i[t]-a_i[t]||^2 + (lambda/2) sum_t
||z[t+1]-2z[t]+z[t-1]||^2  s.t.  x_i[t] - offset_i = z[t]` (plus per-step boxes).

The x- and dual-updates mirror the static solver, but the smoothness term couples
the center across time, so the z-update is no longer a per-step average — it is a
banded (pentadiagonal) symmetric-positive-definite linear solve
`(rho N I + lambda D^T D) z = rho sum_i (x_i - offset_i + u_i)` per coordinate,
with `D` the second-difference operator. The system matrix is constant across
iterations, so it is Cholesky-factorized once and back-solved each iteration. An
optional `anchor` fixes `z[0]` (the current team center) — exactly the inner step
of a receding-horizon MPC loop: solve over the horizon, apply `z[1]`, shift, and
re-solve. With `lambda = 0` and a one-step horizon this reduces to the static
centralized consensus.

`benchmark_admm_horizon_consensus` runs that MPC loop: a four-agent square
formation follows a moving goal that turns a sharp L-corner, but each agent only
sees a *noisy* perceived goal:

```bash
cargo run -p rust_robotics --example benchmark_admm_horizon_consensus --no-default-features --features control
```

The honest result has two regimes. Under noisy perception (the rendered scenario)
the consensus rejects noise both spatially (averaging across agents) and
temporally (smoothing), so smoothing cuts the executed jerk by ~66% *and* lowers
the tracking error (≈0.055 → 0.031) — the per-agent noise, not the corner, is the
dominant error and smoothing wins on both axes. With near-perfect perception the
classic trade-off returns: smoothing still cuts jerk but the corner-cutting lag
*raises* the tracking error (≈0.000 → 0.021). The benchmark prints both regimes.
