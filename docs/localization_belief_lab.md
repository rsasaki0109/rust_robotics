# Localization belief update lab

[Open the experiment](https://rsasaki0109.github.io/rust_robotics/playground/?tab=localization&loc_view=belief&belief_event=resample).

![A recorded particle-filter update](assets/playground/belief-lab.png)

The lab records 24 range-sensor updates from the library's particle filter.
Its 96 hypotheses pass through four inspectable phases:

1. **Prior** carries the preceding scan's final population and weights.
2. **Predict** propagates each hypothesis through the noisy motion model.
3. **Weight** multiplies its previous weight by Gaussian likelihoods for the
   enabled landmark ranges, then normalizes the population.
4. **Resample** uses the library's systematic sampler when effective sample
   size is below `96 × 0.55 = 52.8`. Each actual copy receives weight `1/96`.
   Above that threshold, the population and its weights stay unchanged.

The white ring is the weighted position mean; green shows the true robot.
Blue points are hypotheses, sized by the current weight. Cyan circles show
actual noisy ranges. Copies overlap at the same positions: resampling does
not move particles or create additional information.

Try **Before resampling**, **Highest weight**, then **Resample**. The selected
output's gold readout identifies its actual weighted parent and that parent's
copy count. Particle indices refer to the current population; the same index
need not describe the same hypothesis after resampling.

Tap or drag the weight bars to select a hypothesis. Compare **A only**,
**A + B**, and **All five** to see how landmark evidence changes the posterior.
Compare **Quiet**, **Normal**, and **Noisy** to change range standard deviation
to 0.15, 0.25, or 0.65 metres. A single range can leave ambiguous positions;
a finite cloud can also lose useful hypotheses. This display is an explanation
of the sampled estimator, not an accuracy guarantee.

**+1 phase** and scan navigation read stored results. **Play phases** advances
two recorded phases per second; the experiment's sensor interval is 0.25 s.
Changing evidence, noise, or seed rebuilds a paused experiment. Sensor and
filter RNGs are separate: landmark subsets retain the same initial cloud,
true trajectory and common sensor measurements. Later filter predictions can
diverge after their resampling decisions differ.

**Share** restores the seed, exact noise value, landmark subset, phase,
selected particle and range overlay, paused. **Live PF / EKF** opens the
existing driving demo; its original links retain their behavior.

## Reproduce and validate

```bash
cargo test -p rust_robotics_localization particle_filter
cargo test -p rust_robotics_playground localization::belief
cargo test -p rust_robotics_playground
cargo clippy -p rust_robotics_playground -p rust_robotics_localization --all-targets -- -D warnings
cd crates/rust_robotics_playground
RUSTFLAGS='--cfg getrandom_backend="wasm_js"' trunk build --release --public-url /rust_robotics/playground/
cd ../..
npm --prefix scripts/web_smoke install
npx --prefix scripts/web_smoke playwright install chromium
xvfb-run -a node scripts/web_smoke/web_smoke.mjs crates/rust_robotics_playground/dist /tmp/belief-lab-shots docs
```

The browser suite exercises actual mouse/touch input on desktop, 320 px phone,
900 × 700 desktop, and a phone capped at 2.5 paints/s. It independently checks
Gaussian weight products, normalization, the ESS threshold, real parent copies,
prediction motion, painted point/range/bar geometry, pause, and exact shares.
Each new scenario records 28 acknowledged actions. Read-only canvas diagnostics
come from the actual drawing loops; the fixture never writes application state.

Recorded interaction reports:
[desktop](assets/belief-lab.json), [phone](assets/belief-lab-phone.json),
[compact desktop](assets/belief-lab-compact.json),
[slow phone](assets/belief-lab-phone-slow.json).

Implementation: [`localization/belief.rs`](../crates/rust_robotics_playground/src/localization/belief.rs),
[`view.rs`](../crates/rust_robotics_playground/src/localization/belief/view.rs),
and [`particle_filter.rs`](../crates/rust_robotics_localization/src/particle_filter.rs).
The seeded library methods share the existing initialization, motion and
resampling implementations. The ancestry API returns the indices actually
copied by that sampler.
