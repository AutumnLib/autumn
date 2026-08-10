# Autumn

Autumn is a PROS C++ template for VEX V5 autonomous programming. It combines compile-time dependency wiring (Spring-style `Autumn` + `AutumnContext`), LemLib-inspired motion chaining (`MotionHandler` + free-function motions), and a control stack from odometry through Pure Pursuit, optional LTV unicycle tracking, and MCL.

## Features

- **Robot shell**: `AbstractConfig`, `AbstractAutonomous`, `AbstractDriver`, `AutumnContext<Config>`
- **MotionHandler**: async motion task with `move`, `cancel`, `waitUntilPoint`, `waitUntilDistance`
- **Motions**: `moveToPoint`, `moveToPose`, `turnToHeading`, `turnToPoint`, `follow` (PP + optional LTV)
- **Odometry**: tracking wheels + optional IMU heading fusion
- **Hardware HAL**: `Encoder` / `IMU` / `MotorGroup` facades with micro-disconnect resilience (last-good + reconnect rebase)
- **Composable controllers**: `Controller<In,Out>` with `PidController` / `FeedforwardController` / `SlewController` / `SlewedController` + `ChainedController` / `AdditiveController`; motions honor `MotionSettings` controller hooks
- **Chassis facade**: plain-inch overloads, arcade/curvature drive, motion wrappers
- **Multi-task driver**: `AbstractDriver::registerTask()` runs driver functions on their own `pros::Task`
- **Logging**: LemLog-style topic logger (`TerminalSink`) + brain error codes (`E001`-`E004`) raised on disconnect / timeout / IMU cal
- **Auton selector**: liblvgl owns the brain screen (`showLvglMenu()`); LLEMU (`showLlemuMenu()`) is fallback only — do not mix
- **Config DI bag**: `config().put<T>/get<T>/has` for arbitrary motors/pistons/sensors
- **Paths**: Path.jerry.io-compatible `static/*.txt` + `ASSET()` macro
- **Hardware**: single `templates/config.hpp` registry accessed via `config()`; `Odometry` is non-copyable (ref only)

## Quick start

1. Install [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/index.html) and apply the autumn template (`pros conduct apply autumn` when using a remote depot).
2. Configure hardware in [`templates/config.hpp`](templates/config.hpp).
3. Define auton routes in [`templates/autonomous.hpp`](templates/autonomous.hpp) via `getRoutes()` and driver code in [`templates/driver.hpp`](templates/driver.hpp).
4. Build the library and project:

```bash
make libautumn   # produces firmware/libautumn.a
make             # builds hot/cold packages
```

5. Upload with PROS or VS Code.

## Project layout

```
include/autumn/     Library headers
src/autumn/         Library implementations (compiled into libautumn.a)
templates/          User config, auton, driver
static/             Pure pursuit path files
firmware/           libautumn.a + hot-cold-asset.mk
depot/              Remote Conductor depot manifest
docs/               Documentation (ReadTheDocs)
```

## Example auton

```cpp
chassis_.setPose(0, 0, 0);
chassis_.moveToPoint(24, 0, 4000);
chassis_.waitUntilDone();
chassis_.turnToHeading(90, 2000);
chassis_.follow(example_txt, 15, 6000);
```

## LemLib comparison

| Area | LemLib | Autumn |
|------|--------|--------|
| Config | Globals or chassis ctor blob | `AutumnContext` + `AbstractConfig` |
| Motions | v0.6 free functions + MotionHandler | Same pattern |
| Distribution | `liblemlib.a` | `libautumn.a` |
| Hardware | User-owned | `templates/config.hpp` via `config()` |
| Advanced | PP, boomerang | + optional LTV, MCL, profiling, SysId |

## Documentation

See [`docs/index.md`](docs/index.md). Remote depot manifest is hosted on the `depot` branch (not yet published).

## License

MIT — see [LICENSE](LICENSE).
