# Autumn documentation

Autumn is a VEX V5 PROS autonomous library template.

## Getting started

1. Edit `templates/config.hpp` for ports and drivetrain geometry (plain inches/degrees).
2. Implement `TemplateAuto::getRoutes()` in `templates/autonomous.hpp`.
3. Run `make libautumn` then `make`.

## Team API

- **Units**: inches, degrees, and milliseconds at the team boundary (`chassis.moveToPoint(24, 0, 4000)`).
- **Hardware**: register all devices in `templates/config.hpp`; access via `config()` in auton/driver.
- **Auton menu**: override `getRoutes()` — no manual `addRoutine` in `main.cpp`.

## Architecture

- **Core**: `Autumn`, `AutumnContext`, `AbstractConfig`, `AbstractAutonomous`, `AbstractDriver`
- **Motion**: `motion_handler`, free-function motions, `Chassis` facade
- **Localization**: `Odometry` (non-copyable — pass by reference). `ParticleFilter` is a deferred MCL stub and is not the pose source.
- **Control**: dt-based PID (LemLib 0.5 per-tick gains convert at 10 ms as kI×100 and kD/100), feedforward, trapezoid and jerk-limited S-curve profiles. `LtvUnicycleController` is a documented stub, not used by `follow`. Composable `Controller<In,Out>` hooks remain.
- **Hardware HAL**: `Encoder` / `IMU` / `MotorGroup` / `AdiEncoder` facades with disconnect resilience + mutex
- **Logging**: `logger` (terminal/SD sinks) + brain `ErrorCode`s raised at HAL/motion sites
- **UI**: `AutonSelector` liblvgl menu owns screen (LLEMU fallback; do not mix)

## Motion API

```cpp
autumn::motion_handler::move([&] {
    autumn::moveToPoint(target, timeout, params, settings);
});
autumn::motion_handler::waitUntilDistance(12_in, settings.poseGetter);
```

`follow` is pure pursuit. Boomerang is `moveToPose`. Discrete LTV tracking is deferred; see `LtvUnicycleController`.

```cpp
chassis.moveToPose(24, 24, 90, 4000);
chassis.follow(path, 15, 6000, false);
```

## Paths

Export paths from [Path.jerry.io](https://path.jerry.io) as `static/my_path.txt`:

```
x,y,speed
0,0,24
24,0,24
```

Declare with `ASSET(my_path_txt);` and pass to `chassis.follow(my_path_txt, ...)`.

## PROS Conductor

**Maintainer release:** dispatch the `Create Release` GitHub Actions workflow (`.github/workflows/release.yml`)
with a `version` (and `prerelease` flag if it's an alpha/beta). It builds `libautumn.a`, packages the template
via `make template-release` (`pros conduct create-template`), publishes a GitHub Release with the zip attached,
and dispatches `.github/workflows/depot.yml` to refresh `stable.json`/`beta.json` on the `depot` branch.

Locally, use `make template` (not `-release`) when developing the library in this monorepo — both build the
same way (`IS_LIBRARY=1`); `template-release` just also bumps `VERSION` and produces the distributable zip.

**Teams (remote depot):**

```bash
pros conduct add-depot Autumn https://raw.githubusercontent.com/AutumnLib/autumn/depot/stable.json
# or .../depot/beta.json to track prereleases

pros conduct apply autumn
pros conduct upgrade autumn
```

`Autumn` in `add-depot` is just a local nickname for the depot — call it whatever you want. The template name in
`apply`/`upgrade` must be exactly `autumn` (lowercase); pros-cli matches template names case-sensitively, so
`apply Autumn` fails with a misleading `does not support kernel version ...` error instead of a clear "not found".

User files (`templates/`, `src/main.cpp`, `static/`) are never overwritten on upgrade. System files (`include/autumn/`, `firmware/libautumn.a`) are upgraded.

The remote depot manifest (`stable.json`/`beta.json`) lives on the `depot` branch, regenerated automatically
from GitHub Releases by `depot.yml` — `stable.json` stays empty until the first non-prerelease version ships.
