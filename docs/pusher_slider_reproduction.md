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
switching faces. Simultaneous multi-contact pushing (two pushers at once) remains
the natural next extension.
