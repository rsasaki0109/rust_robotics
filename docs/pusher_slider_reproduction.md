# Quasi-Static Planar Pushing (Pusher-Slider)

A pure-Rust reproduction slice for contact-rich planar manipulation, in the
spirit of "Push Anything" and the classic pusher-slider literature
(Goyal/Howe/Mason limit surface; Lynch motion cone; Hogan & Rodriguez reactive
pushing). It is the first manipulation target in this repo.

A point pusher that may contact any of the slider's four faces shoves a rigid
square slider across a table to a goal pose. Under the quasi-static assumption,
motion is set by the contact rather than by inertia, and the contact either
*sticks* (moves with the pusher) or *slides* (the friction cone saturates and the
contact slips along the face). Letting the controller switch faces is what makes
a *pure rotation* (no net translation) reachable — impossible for a single face.

Implemented slice:

1. `SliderState` — the planar pose `[x, y, theta]` of the slider.
2. `PusherSliderParams` — slider half-extent, the ellipsoidal limit-surface
   characteristic length `c`, and the pusher/slider Coulomb friction.
3. `PusherCommand` — a body-frame pusher motion on a chosen face (`0` = back,
   `1` = `+y`, `2` = front, `3` = `-y`): the face index, a contact offset along
   it, a normal push speed, and a tangential slip.
4. `PusherSliderParams::step` — one quasi-static update returning the new pose and
   the realized `ContactMode` (stick / slide-up / slide-down / separated).
5. `PusherSliderMppiController` / `simulate_push` — a seeded, deterministic
   controller that runs MPPI per face and executes the command from the
   lowest-cost face, modulating the contact offset, slip, and push speed (so it
   can brake near the goal) and switching faces when that helps.

Run:

```bash
cargo run -p rust_robotics --example benchmark_pusher_slider --no-default-features --features control
```

## Model

The slider obeys the ellipsoidal limit surface `L = diag(1, 1, 1/c^2)`: a contact
wrench `w = (fx, fy, m)` with torque `m = px*fy - py*fx` produces a body twist
parallel to `L w`, i.e. `(vx, vy, omega) = (fx, fy, (px*fy - py*fx)/c^2)`. Each
face contributes a contact frame `(p, d, t)` — point, inward normal, tangent —
and the limit-surface solve is identical for every face; only the frame rotates,
so the friction cone is evaluated along the face's own normal and tangent rather
than the body axes. (The back face recovers `d = (1,0)`, `t = (0,1)`,
`p = (-b, contact)`.)

- **Sticking.** The contact point moves with the pusher. Equating the
  contact-point velocity `v + omega x r` to the commanded pusher velocity
  `(vn, vt)` gives a 2x2 system for the contact force; if that force lies inside
  the friction cone (`|fy| <= mu*fx`), the contact sticks and the twist is the
  limit-surface image of the wrench.
- **Sliding.** If the required tangential force leaves the friction cone, the
  force saturates on the cone edge `f = k*(1, +-mu)` and the contact slides along
  the face. The scale `k` is chosen so the normal contact-point speed still
  matches the commanded push.

Simultaneous multi-contact pushing is left as an extension; the model is exact
for the single-point stick/slide regime on each face.

## Controller

`PusherSliderMppiController` is the seeded MPPI used elsewhere in the repo,
specialized to pushing and made *face-aware*: it keeps a warm-started nominal
plan per face, runs MPPI within each face, and executes the command from
whichever face yields the lowest-cost rollout. Switching faces is what makes a
pure rotation reachable — push the back face to spin one way and translate `+x`,
then the front face to spin the same way while translating `-x`, cancelling the
drift.

Each rollout integrates the quasi-static model and is scored by a pose cost
(squared position error plus a heading term). Because the slider moves only a few
centimetres, the squared position errors are tiny, so the pose weights are
deliberately large — the softmax temperature needs cost values of order 1-100 to
discriminate rollouts, which is also what lets the controller *brake*: sampling
the push speed (down to zero) means it can stop on the goal instead of plowing
through it.

## Benchmark

`benchmark_pusher_slider` pushes the slider to four goal poses and writes
`docs/assets/pusher-slider.csv` and `.svg` (start box in gray, goal outline in
green, final box in blue, with the CoM path):

- `forward` — a straight push; a mix of sticking and sliding to null the offset.
- `veer` — a lateral goal reached by steering: the controller slides the contact
  along the face to walk the slider sideways.
- `spin` — a pure rotation (theta only, no net translation). The controller
  switches between the back and front faces to rotate while cancelling the
  translation each face induces; unreachable by a single face.
- `park` — a combined translate-and-rotate to a negative heading.

All four reach the goal within about a centimetre, with the stick/slide split
varying by task. The `spin` case in particular resolves the limitation of the
single-face model: rotation without coupled lateral drift is now reachable by
switching faces.

## Multi-Object Pushing

`simulate_multi_push` reproduces the multi-object setting of "Push Anything": it
pushes several sliders to their goal poses one at a time, treating the *other*
objects (at their current poses) as keep-out discs for the active one. The
keep-out is a soft penalty in the controller cost (`obstacle_weight`,
`obstacle_radius`), so the active slider is routed around blocks that sit in its
straight-line path instead of plowing through them. Objects are pushed in index
order, and each object's resting pose becomes an obstacle for the objects pushed
after it — no simultaneous multi-contact solve is required.

`benchmark_pusher_slider_multi` arranges three sliders, with object 0's straight
path to its goal blocked by object 1; object 0 detours around it (about 57 steps
versus ~40 for an unobstructed push), then object 1 moves up and object 2 slides
across. All three settle within about a centimetre of their slots:

```bash
cargo run -p rust_robotics --example benchmark_pusher_slider_multi --no-default-features --features control
```

## Two-Contact Pushing

`PusherSliderParams::two_contact_twist` / `two_contact_step` solve *two*
simultaneous point contacts contact-implicitly. Each contact keeps its commanded
normal speed; its tangential degree of freedom is either sticking (tangential
velocity matches the pusher) or sliding (the tangential force sits on the
friction-cone edge). Two independent pushers generically cannot both stick — the
relative velocity of two points on a rigid body is constrained — so the realized
regime is found by enumerating the per-contact stick/slide modes, solving the
resulting 4x4 contact-force system (`solve4`, Gaussian elimination with partial
pivoting), and keeping the first combination whose forces push (`fn >= 0`),
respect the cone (stick), or sit on the correct edge with a consistent slip
direction (slide). When both contacts are rigid-redundant the force solution is
non-unique (a squeeze degree of freedom), so the solver lands on a cone-edge
combination; the resulting *motion* is still the correct rigid-consistent twist.

`benchmark_pusher_slider_two_contact` is a scripted demo (no controller) showing
what a second contact buys:

```bash
cargo run -p rust_robotics --example benchmark_pusher_slider_two_contact --no-default-features --features control
```

- `single` — one off-center contact pushing forward: the slider rotates as it
  translates, so its path curves sharply (about 127 degrees of spin over the run).
- `two-point` — two contacts on the same face at +/-h pushing forward: the second
  contact cancels the off-center torque, and the slider tracks dead straight
  (0.20 m forward, ~0 degrees).
- `couple` — a back-face contact high and a front-face contact low push in
  opposite directions, forming a couple: the slider spins ~278 degrees with
  essentially zero net translation — a single contact cannot produce zero-net-force
  rotation.

The remaining extension is a contact-implicit MPPI/MPC controller that *chooses*
the two contacts and their motions (rather than the scripted commands here),
plus a least-norm resolution of the squeeze degree of freedom.
