RustRobotics
====

This package is a rust implementation of [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics).

Build
```
git clone https://github.com/rsasaki0109/RustRobotics.git
cd RustRobotics
cargo build
```

Run (Example)
```
cargo run --bin ekf
```

## Localization
### Extended Kalman Filter localization
<img src="./img/ekf.svg" width="640px">   

Red:GPS, Brue:Ground Truth, Green:EKF, Yellow:Dead Reckoning
