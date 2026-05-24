# Autumn

Autumn is a PROS C++ template for VEX V5 autonomous programming. It combines compile-time dependency wiring (Spring-style `Robot` + `RobotContext`), LemLib-inspired motion chaining (`MotionHandler` + free-function motions), and a control stack from odometry through Pure Pursuit, optional LTV unicycle tracking, and MCL.

## Features

- **Robot shell**: `BaseConfig`, `BaseAutonomous`, `BaseDriver`, `RobotContext<Config>`
- **MotionHandler**: async motion task with `move`, `cancel`, `waitUntilPoint`, `waitUntilDistance`
- **Motions**: `moveToPoint`, `moveToPose`, `turnToHeading`, `turnToPoint`, `follow` (PP + optional LTV)
- **Odometry**: tracking wheels + optional IMU heading fusion
- **Chassis facade**: plain-inch overloads, arcade/curvature drive, motion wrappers
- **Paths**: Path.jerry.io-compatible `static/*.txt` + `ASSET()` macro
- **Safety**: stall watchdog aborts stuck motions
- **Hardware**: single `team/config.hpp` registry accessed via `config()`

## Quick start

1. Install [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/index.html) and apply the autumn template (`pros conduct apply autumn` when using a remote depot).
2. Configure hardware in [`team/config.hpp`](team/config.hpp).
3. Define auton routes in [`team/autonomous.hpp`](team/autonomous.hpp) via `getRoutes()` and driver code in [`team/driver.hpp`](team/driver.hpp).
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
team/               User config, auton, driver
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
| Config | Globals or chassis ctor blob | `RobotContext` + `BaseConfig` |
| Motions | v0.6 free functions + MotionHandler | Same pattern |
| Distribution | `liblemlib.a` | `libautumn.a` |
| Hardware | User-owned | `team/config.hpp` via `config()` |
| Advanced | PP, boomerang | + optional LTV, MCL, profiling, SysId |

## Documentation

See [`docs/index.md`](docs/index.md). Remote depot setup: [`depot/README.md`](depot/README.md).

## License

MIT — see [LICENSE](LICENSE).
