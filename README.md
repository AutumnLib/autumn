# Autumn

Autumn is a PROS C++ template for VEX V5 autonomous programming. It combines compile-time dependency wiring (Spring-style `Autumn` + `AutumnContext`), LemLib-inspired motion chaining (`MotionHandler` + free-function motions), and a control stack from odometry through Pure Pursuit and boomerang `moveToPose`. A discrete LTV unicycle tracker and MCL are deferred.

## Features

- **Robot shell**: `AbstractConfig`, `AbstractAutonomous`, `AbstractDriver`, `AutumnContext<Config>`
- **MotionHandler**: async motion task with `move`, `cancel`, `waitUntilPoint`, `waitUntilDistance`
- **Motions**: `moveToPoint`, boomerang `moveToPose`, `turnToHeading`, `turnToPoint`, `swingToHeading`, `swingToPoint`, `follow` (pure pursuit)
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

1. Install [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/index.html), add the Autumn depot, and apply the template:

   ```bash
   # Stable releases only
   pros conduct add-depot Autumn https://raw.githubusercontent.com/AutumnLib/autumn/depot/stable.json
   # Or, to track prereleases/alphas instead:
   pros conduct add-depot Autumn https://raw.githubusercontent.com/AutumnLib/autumn/depot/beta.json

   pros conduct apply autumn
   ```

   Until the first non-prerelease version ships, `stable.json` is empty — use the `beta.json` depot to try alpha/beta builds.

   > `Autumn` in `add-depot` is just a local nickname for the depot — call it whatever you want. The template name
   > in `apply`/`upgrade` must be exactly `autumn` (lowercase); pros-cli matches template names case-sensitively,
   > so `apply Autumn` will fail with a misleading `does not support kernel version ...` error.
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
| Advanced | PP, boomerang | PP + boomerang `moveToPose`. LTV and MCL are deferred. Jerk-limited S-curve exists as a profile primitive. |

## Documentation

See [`docs/index.md`](docs/index.md). Remote depot manifest (`stable.json`/`beta.json`) is hosted on the `depot` branch, populated automatically on each release.

## License

MIT — see [LICENSE](LICENSE).
