# Quasi-Static Planar Pushing (Pusher-Slider)

A pure-Rust reproduction slice for contact-rich planar manipulation, in the
spirit of "Push Anything" and the classic pusher-slider literature
(Goyal/Howe/Mason limit surface; Lynch motion cone; Hogan & Rodriguez reactive
pushing). It is the first manipulation target in this repo.

A single point pusher shoves a rigid square slider across a table to a goal pose.
Under the quasi-static assumption, motion is set by the contact rather than by
inertia, and the contact either *sticks* (moves with the pusher) or *slides*
(the friction cone saturates and the contact slips along the face).

Implemented slice:

1. `SliderState` — the planar pose `[x, y, theta]` of the slider.
2. `PusherSliderParams` — slider half-extent, the ellipsoidal limit-surface
   characteristic length `c`, and the pusher/slider Coulomb friction.
3. `PusherCommand` — a body-frame pusher motion on the slider's back face: a
   contact offset along the face, a normal push speed, and a tangential slip.
4. `PusherSliderParams::step` — one quasi-static update returning the new pose and
   the realized `ContactMode` (stick / slide-up / slide-down / separated).
5. `PusherSliderMppiController` / `simulate_push` — a seeded, deterministic
   sampling controller that drives the slider to a goal pose, modulating the
   contact offset, slip, and push speed (so it can brake near the goal).

Run:

```bash
cargo run -p rust_robotics --example benchmark_pusher_slider --no-default-features --features control
```

## Model

The slider obeys the ellipsoidal limit surface `L = diag(1, 1, 1/c^2)`: a contact
wrench `w = (fx, fy, m)` with torque `m = px*fy - py*fx` produces a body twist
parallel to `L w`, i.e. `(vx, vy, omega) = (fx, fy, (px*fy - py*fx)/c^2)`. The
contact sits on the back face at `(px, py) = (-b, contact)`.

- **Sticking.** The contact point moves with the pusher. Equating the
  contact-point velocity `v + omega x r` to the commanded pusher velocity
  `(vn, vt)` gives a 2x2 system for the contact force; if that force lies inside
  the friction cone (`|fy| <= mu*fx`), the contact sticks and the twist is the
  limit-surface image of the wrench.
- **Sliding.** If the required tangential force leaves the friction cone, the
  force saturates on the cone edge `f = k*(1, +-mu)` and the contact slides along
  the face. The scale `k` is chosen so the normal contact-point speed still
  matches the commanded push.

Switching faces and multi-contact pushing are left as extensions; the model is
exact for the single-face stick/slide regimes.

## Controller

`PusherSliderMppiController` is the same seeded MPPI used elsewhere in the repo,
specialized to pushing. Each rollout integrates the quasi-static model and is
scored by a pose cost (squared position error plus a heading term). Because the
slider moves only a few centimetres, the squared position errors are tiny, so the
pose weights are deliberately large — the softmax temperature needs cost values
of order 1-100 to discriminate rollouts, which is also what lets the controller
*brake*: sampling the push speed (down to zero) means it can stop on the goal
instead of plowing through it.

## Benchmark

`benchmark_pusher_slider` pushes the slider to four goal poses and writes
`docs/assets/pusher-slider.csv` and `.svg` (start box in gray, goal outline in
green, final box in blue, with the CoM path):

- `forward` — a straight push; nearly pure sticking on the approach with sliding
  used to null the final offset.
- `veer` — a lateral goal reached by steering: the controller slides the contact
  along the face to walk the slider sideways.
- `reorient` — a rotation paired with the lateral drift it naturally induces, so
  a single back-face pusher can realize it.
- `park` — a combined translate-and-rotate to a negative heading.

All four reach the goal within about a centimetre, with the stick/slide split
varying by task (heavier sliding for the lateral and steering goals). A goal that
demands rotation *without* the coupled lateral motion is not reachable by a
single back-face pusher — switching faces or adding a second contact would be
needed, and that is the natural next extension.
